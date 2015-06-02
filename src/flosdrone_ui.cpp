/*
 * flosdrone UI class implementation.
 * 
 * Copyright 2014 Tampere University of Technology.
 * Copyright 2015 Ansse Saarimäki.
 * 
 * This file is part of flosdrone.
 *
 * flosdrone is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * flosdrone is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with flosdrone.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include "flosdrone_ui.h"
#include "flosdrone_estimator.h"

#include "opencv2/core/core.hpp"

#include "glm/gtc/quaternion.hpp"

const double PI = 3.14159265;

const int IMGWIDTH = 1280; const int IMGHEIGHT = 360;

const double GRID_BIG_SPACING = 1;
const double GRID_SMALL_SPACING = 0.2;
const double GRID_BIG_MAX = 30;
const double GRID_SMALL_MAX = 5;
const float LINE_WIDTH_FACTOR = 1.0;

FlosdroneUI* monPtr;

//###########################################################################//

// Wrapper functions for glfw callbacks (C<3)
void framebufferResizeCbWrapper(GLFWwindow* window, int width, int height)
{
    monPtr->framebufferResizeCb(width, height);
}

void mouseClickCbWrapper(GLFWwindow* window, int button, int action, int mods)
{
    monPtr->mouseClickCb(button, action, mods);
}

void mouseScrollCbWrapper(GLFWwindow* window, double xoffset, double yoffset)
{
    monPtr->mouseScrollCb(xoffset, yoffset);
}

//###########################################################################//

// Function turn a cv::Mat into a texture, and return the texture ID as a GLuint for use
// Original from http://r3dux.org/2012/01/how-to-convert-an-opencv-cvmat-to-an-opengl-texture/
GLuint matToTexture(const cv::Mat &mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter)
{
    // Generate a number for our textureID's unique handle
    GLuint textureID;
    glGenTextures(1, &textureID);

    // Bind to our texture handle
    glBindTexture(GL_TEXTURE_2D, textureID);

    // Check for for correct parameters
    if(magFilter != GL_NEAREST && magFilter != GL_LINEAR)
    {
        std::cerr << "Incorrect GL_TEXTURE_MAG_FILTER - setting it to GL_LINEAR" << std::endl;
        magFilter = GL_LINEAR;
    }

    if(minFilter != GL_NEAREST && minFilter != GL_LINEAR)
    {
        std::cerr << "Incorrect GL_TEXTURE_MIN_FILTER - setting it to GL_LINEAR" << std::endl;
        minFilter = GL_LINEAR;
    }

    if(wrapFilter != GL_CLAMP_TO_EDGE && wrapFilter != GL_CLAMP_TO_BORDER &&
            wrapFilter != GL_MIRRORED_REPEAT && wrapFilter != GL_REPEAT &&
            wrapFilter != GL_MIRROR_CLAMP_TO_EDGE)
    {
        std::cerr << "Incorrect GL_TEXTURE_WRAP - setting it to GL_REPEAT" << std::endl;
        wrapFilter = GL_REPEAT;
    }

    // Set texture interpolation methods for minification and magnification
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);

    // Set texture clamping method
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapFilter);

    // Set incoming texture format to:
    // GL_BGR       for CV_CAP_OPENNI_BGR_IMAGE,
    // GL_LUMINANCE for CV_CAP_OPENNI_DISPARITY_MAP,
    // Work out other mappings as required ( there's a list in comments in main() )
    GLenum inputColourFormat = GL_BGR;
    if(mat.channels() == 1)
    {
        inputColourFormat = GL_LUMINANCE;
    }

    // Create the texture
    glTexImage2D(GL_TEXTURE_2D,     // Type of texture
                 0,                 // Pyramid level (for mip-mapping) - 0 is the top level
                 GL_RGB,            // Internal colour format to convert to
                 mat.cols,          // Image width  i.e. 640 for Kinect in standard mode
                 mat.rows,          // Image height i.e. 480 for Kinect in standard mode
                 0,                 // Border width in pixels (can either be 1 or 0)
                 inputColourFormat, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                 GL_UNSIGNED_BYTE,  // Image data type
                 mat.ptr());        // The actual image data itself


    return textureID;
}

//###########################################################################//


FlosdroneUI::FlosdroneUI(FlosdroneEstimator* est): estimator_(est),
        run_(false), threadPtr_(0),
        imgVpWidth_(0), imgVpHeight_(0), fbWidth_(0),
        fbHeight_(0), leftClick_(false), rightClick_(false),
        mouseXref_(0.0), mouseYref_(0.0), zrot_(180.0), xrot_(90.0),
        camRadius_(5.0), centerX_(0.0), centerY_(0.0)
{
    monPtr = this;
}

FlosdroneUI::~FlosdroneUI()
{
    stop();
}


void FlosdroneUI::framebufferResizeCb(int width, int height)
{
    fbWidth_ = width;
    fbHeight_ = height;
    float ratio = width / (float) height;
    float imgRatio = IMGWIDTH / (float) IMGHEIGHT;

    if(width >= IMGWIDTH && height >= 2*IMGHEIGHT)
    {
        imgVpWidth_ = IMGWIDTH;
        imgVpHeight_ = IMGHEIGHT;
    }
    else if(ratio < imgRatio)
    {
        imgVpWidth_ = width / 2;
        imgVpHeight_ = imgVpWidth_ / imgRatio;
    }
    else
    {
        imgVpHeight_ = height / 2;
        imgVpWidth_ = imgVpHeight_ * imgRatio;
    }
}

void FlosdroneUI::mouseClickCb(int button, int action, int mods)
{
    if(button == GLFW_MOUSE_BUTTON_LEFT && !rightClick_)
    {
        if(action == GLFW_PRESS)
        {
            glfwGetCursorPos(window_, &mouseXref_, &mouseYref_);
            if(mouseXref_ < imgVpWidth_ && mouseYref_ > (fbHeight_ - imgVpHeight_))
            {
                estimator_->imageClicked();
            }
            else
            {
                leftClick_ = true;
            }
        }
        else if(action == GLFW_RELEASE)
        {
            leftClick_ = false;
        }
    }
    else if(button == GLFW_MOUSE_BUTTON_RIGHT && !leftClick_)
    {
        if(action == GLFW_PRESS)
        {
            glfwGetCursorPos(window_, &mouseXref_, &mouseYref_);
            rightClick_ = true;
        }
        else if(action == GLFW_RELEASE)
        {
            rightClick_ = false;
        }
    }
}

void FlosdroneUI::mouseScrollCb(double xoffset, double yoffset)
{
    // Zoom in.
    if(yoffset > 0 && camRadius_ > 0.1)
    {
        camRadius_ /= 1.2;
    }
    // Zoom out.
    else if(yoffset < 0 && camRadius_ < 50)
    {
        camRadius_ *= 1.2;
    }
}


void FlosdroneUI::start()
{
    if(threadPtr_ == 0)
    {
        runMutex_.lock();
        run_ = true;
        runMutex_.unlock();
        threadPtr_ = new boost::thread(
                boost::bind(&FlosdroneUI::run, this));
    }
}

void FlosdroneUI::stop()
{
    if(threadPtr_ != 0)
    {
        runMutex_.lock();
        run_ = false;
        runMutex_.unlock();

        threadPtr_->join();
        delete threadPtr_;
        threadPtr_ = 0;
    }
}

void FlosdroneUI::run()
{
    if (!glfwInit())
        exit(EXIT_FAILURE);

    window_ = glfwCreateWindow(1280, 720, "Flosdrone", NULL, NULL);
    if (!window_)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window_);

    glfwSwapInterval(1);

    glfwSetFramebufferSizeCallback(window_, &framebufferResizeCbWrapper);
    glfwSetMouseButtonCallback(window_, &mouseClickCbWrapper);
    glfwSetScrollCallback(window_, &mouseScrollCbWrapper);

    framebufferResizeCb(1280, 720);

    imgTex_ = matToTexture(
            cv::Mat(IMGWIDTH, IMGHEIGHT, CV_8UC3, cv::Scalar(0,0,255)),
            GL_LINEAR, GL_LINEAR, GL_MIRRORED_REPEAT);

    runMutex_.lock();
    while (run_)
    {
        runMutex_.unlock();

        renderMap();
        renderImage();

        glfwSwapBuffers(window_);
        glfwPollEvents();

        runMutex_.lock();
    }
    runMutex_.unlock();

    glfwDestroyWindow(window_);
    glfwTerminate();
}


void FlosdroneUI::renderMap()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    // Use the whole window for map rendering.
    glViewport(0, 0, fbWidth_, fbHeight_);

    // Set up perspective projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float ratio = fbWidth_ / (float) fbHeight_;
    gluPerspective(90.0, ratio, 0.1, 100.0);

    // Rotate the camera around the center point.
    if(leftClick_)
    {
        // Mouse position in this frame
        double xNew = 0.0;
        double yNew = 0.0;
        glfwGetCursorPos(window_, &xNew, &yNew);

        // Calculate the rotation from the mouse movement
        zrot_ += 0.5 * (xNew - mouseXref_);
        xrot_ += 0.5 * (yNew - mouseYref_);

        // Limit the rotation around x axis.
        if(xrot_ > 90.0)
        {
            xrot_ = 90;
        }
        else if(xrot_ < -90.0)
        {
            xrot_ = -90;
        }

        // Update the reference coordinates.
        mouseXref_ = xNew;
        mouseYref_ = yNew;
    }


    // Set up the camera.
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();
    double x = camRadius_ * sin(2*PI*(zrot_)/360) * cos(2*PI*(xrot_)/360);
    double y = camRadius_ * cos(2*PI*(zrot_)/360) * cos(2*PI*(xrot_)/360);
    double z = camRadius_ * sin(2*PI*(xrot_)/360);
    gluLookAt(x + centerX_, y + centerY_, z, centerX_, centerY_, 0.0, 0.0, 0.0, 1.0);

    // Render a dot in the point the camera rotates around.
    glPointSize(5.0);
    glBegin(GL_POINTS);
    glColor3f(0.1,0.1,1);
    glVertex3d(centerX_, centerY_, 0);
    glEnd();

    // Move the center point for the next frame. Needs to be done after the
    // gluLookAt since modelview and projection matrices are used in the
    // calculations.
    if(rightClick_)
    {
        // Mouse coordinate on previous frame
        double dispXref = 0.0;
        double dispYref = 0.0;
        getDisplayCoordOnXYPlane(mouseXref_, mouseYref_, dispXref, dispYref);

        // Mouse coordinate on this frame
        double xNew = 0.0;
        double yNew = 0.0;
        glfwGetCursorPos(window_, &xNew, &yNew);
        double dispXnew = 0.0;
        double dispYnew = 0.0;
        getDisplayCoordOnXYPlane(xNew, yNew, dispXnew, dispYnew);

        // Limit the clickable area to big grid to prevent weird behavior
        if(abs(dispXref) < GRID_BIG_MAX && abs(dispYref) < GRID_BIG_MAX)
        {
            // Calculate the desired movement of center point
            centerX_ -= dispXnew - dispXref;
            centerY_ -= dispYnew - dispYref;

            // Limit the the center point location to big grid
            if(centerX_ > GRID_BIG_MAX)
            {
                centerX_ = GRID_BIG_MAX;
            }
            else if(centerX_ < -GRID_BIG_MAX)
            {
                centerX_ = -GRID_BIG_MAX;
            }

            if(centerY_ > GRID_BIG_MAX)
            {
                centerY_ = GRID_BIG_MAX;
            }
            else if(centerY_ < -GRID_BIG_MAX)
            {
                centerY_ = -GRID_BIG_MAX;
            }
        }

        // Update the reference mouse coordinates
        mouseXref_ = xNew;
        mouseYref_ = yNew;
    }

    renderGrid();
    renderDrone();
}

void FlosdroneUI::renderGrid()
{
    // Antialiasing
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);

    // Render the grids.

    // Fine grid
    glLineWidth(1*LINE_WIDTH_FACTOR);
    glBegin(GL_LINES);
    for(double x=-GRID_SMALL_MAX; x<=GRID_SMALL_MAX;x+=GRID_SMALL_SPACING)
    {
        if(x != 0 && (x-floor(x)) > 0.1)
        {
            glColor3f(0.2f,0.2f,0.2f);
            glVertex3d(x, -GRID_SMALL_MAX, 0.0);
            glVertex3d(x, GRID_SMALL_MAX, 0.0);
        }
    }
    for(double y=-GRID_SMALL_MAX; y<=GRID_SMALL_MAX;y+=GRID_SMALL_SPACING)
    {
        if(y != 0 && (y-floor(y)) > 0.1)
        {
            glColor3f(0.25f,0.25f,0.25f);
            glVertex3d(-GRID_SMALL_MAX,y, 0.0);
            glVertex3d(GRID_SMALL_MAX,y, 0.0);
        }
    }
    glEnd();


    // Big grid
    glLineWidth(2*LINE_WIDTH_FACTOR);
    glBegin(GL_LINES);
    for(double x=-GRID_BIG_MAX; x<=GRID_BIG_MAX;x+=GRID_BIG_SPACING)
    {
        if(x != 0)
        {
            glColor3f(0.6f,0.6f,0.6f);
            glVertex3d(x, -GRID_BIG_MAX, 0.0);
            glVertex3d(x, GRID_BIG_MAX, 0.0);
        }
    }
    for(double y=-GRID_BIG_MAX; y<=GRID_BIG_MAX;y+=GRID_BIG_SPACING)
    {
        if(y != 0)
        {
            glColor3f(0.6f,0.6f,0.6f);
            glVertex3d(-GRID_BIG_MAX,y, 0.0);
            glVertex3d(GRID_BIG_MAX,y, 0.0);
        }
    }
    glEnd();

    // xy-lines
    glLineWidth(2.5*LINE_WIDTH_FACTOR);
    glBegin(GL_LINES);

    glColor3f(1,1,1);
    glVertex3d(-GRID_BIG_MAX,0,0);
    glVertex3d(0,0,0);
    // Leave space for the colored x-line
    glVertex3d(GRID_BIG_MAX,0,0);
    glVertex3d(1,0,0);

    glVertex3d(0,-GRID_BIG_MAX,0);
    glVertex3d(0,0,0);
    // Leave space for the colored y-line
    glVertex3d(0,GRID_BIG_MAX,0);
    glVertex3d(0,1,0);

    // Colored xyz-lines
    glColor3f(1,0,0);
    glVertex3d(0,0,0);
    glVertex3d(1,0,0);
    glColor3f(0,1,0);
    glVertex3d(0,0,0);
    glVertex3d(0,1,0);
    glColor3f(1,1,1);
    glVertex3d(0,0,0);
    glVertex3d(0,0,1);
    glEnd();

    glDisable(GL_LINE_SMOOTH);
    glDisable(GL_BLEND);

}

void FlosdroneUI::renderDrone()
{
    geometry_msgs::Pose pose;
    estimator_->getPose(pose);

    // GLM quaternion for easier calculations
    glm::quat ori(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

    // Vectors representing the drone's local coordinate axes
    float length = 0.4;
    glm::vec3 xAxis(length, 0.0f, 0.0f);
    glm::vec3 yAxis(0.0f, length, 0.0f);
    glm::vec3 zAxis(0.0f, 0.0f, length);

    // Rotate them according to orientation
    xAxis = xAxis * ori;
    yAxis = yAxis * ori;
    zAxis = zAxis * ori;

    // Drone's position in world coordinates
    geometry_msgs::Point* posi = &(pose.position);

    // Antialiasing
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);

    glLineWidth(4*LINE_WIDTH_FACTOR);
    glBegin(GL_LINES);

    glColor3f(1,0,0);
    glVertex3d(posi->x,posi->y,posi->z);
    glVertex3d(posi->x + xAxis.x, posi->y + xAxis.y, posi->z + xAxis.z);

    glColor3f(0,1,0);
    glVertex3d(posi->x,posi->y,posi->z);
    glVertex3d(posi->x + yAxis.x, posi->y + yAxis.y, posi->z + yAxis.z);

    glColor3f(0,0,1);
    glVertex3d(posi->x,posi->y,posi->z);
    glVertex3d(posi->x + zAxis.x, posi->y + zAxis.y, posi->z + zAxis.z);
    glEnd();

    estimator_->getTrail(trail_, trailOffset_);

    glLineWidth(2*LINE_WIDTH_FACTOR);
    glBegin(GL_LINES);
    glColor3f(1.0,1.0,1.0);

    for(int i = 0; i < (trail_.size() - 1); ++i)
    {
        int idx = (i + trailOffset_ + 1) % trail_.size();
        int idx2 = (i + trailOffset_ + 2) % trail_.size();
        if(glm::distance(trail_.at(idx), trail_.at(idx2)) <= 1.0 ){
            glVertex3d(trail_.at(idx).x, trail_.at(idx).y, trail_.at(idx).z);
            glVertex3d(trail_.at(idx2).x, trail_.at(idx2).y, trail_.at(idx2).z);
        }
    }
    glEnd();
}


void FlosdroneUI::renderImage()
{
    cv::Mat img;
    if(estimator_->getImage(img))
    {
        glDeleteTextures(1, &imgTex_);
        // Convert image to GL texture
		imgTex_ = matToTexture(img, GL_LINEAR, GL_LINEAR, GL_MIRRORED_REPEAT);
    }
    // Disable depth test to make sure that the video will be on top
    glDisable(GL_DEPTH_TEST);

    // Limit the video display to part of the window
    glViewport(0, 0, imgVpWidth_, imgVpHeight_);

    // Orthogonal projection for 2d graphics
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    float imgRatio = IMGWIDTH / (float) IMGHEIGHT;
    glOrtho(-imgRatio, imgRatio, -1.f, 1.f, 1.f, -1.f);

    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();

    // Render two triangles and texture them with image from camera
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, imgTex_);

    // Upper left triangle
    glBegin(GL_TRIANGLES);
    glColor3f(1,1,1);

    glTexCoord2f(0, 0);
    glVertex2f(-imgRatio, 1);

    glTexCoord2f(1, 0);
    glVertex2f(imgRatio, 1);

    glTexCoord2f(1, 1);
    glVertex2f(imgRatio, -1);

    glEnd();

    // Lower right
    glBegin(GL_TRIANGLES);

    glTexCoord2f(0, 0);
    glVertex2f(-imgRatio, 1);

    glTexCoord2f(0, 1);
    glVertex2f(-imgRatio, -1);

    glTexCoord2f(1, 1);
    glVertex2f(imgRatio, -1);

    glEnd();

    glDisable(GL_TEXTURE_2D);

}


void FlosdroneUI::getDisplayCoordOnXYPlane(double dispX, double dispY, double &planeX, double &planeY)
{
    // Get the current matrices and viewport
    double modelview[16], projection[16];
    int viewport[4];

    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetIntegerv( GL_VIEWPORT, viewport );

    // Single point in 2d projects to line in 3d space. Get coordinates of
    // two points on that line (the closest and the furthest ones visible
    // (z=0.0/1.0)). Y-coordinate needs to be inverted because in opengl
    // the origo is in lower left corner instead of top right.
    double posX1, posY1, posZ1, posX2, posY2, posZ2;

    gluUnProject( dispX, viewport[3]-dispY, 0.0, modelview, projection, viewport, &posX1, &posY1, &posZ1);
    gluUnProject( dispX, viewport[3]-dispY, 1.0, modelview, projection, viewport, &posX2, &posY2, &posZ2);

    // Calculate the intersection of the line with XY-plane
    // k = (posZ2 - posZ1) / (posX2 - posX1);
    // b = (-k * posX1) + posZ1;
    // planeX = -b / k;
    // => planeX = posX1 + ((posX2-posX1) / (1 - (posZ2 / posZ1)));
    // and the same with Y-coordinates.
    double temp = 1 - (posZ2 / posZ1);
    planeX = posX1 + ((posX2-posX1) / temp);
    planeY = posY1 + ((posY2-posY1) / temp);

}

