/*
 * flosdrone UI class header file.
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


#ifndef FLOSDRONE_UI_H
#define FLOSDRONE_UI_H

#include <boost/thread.hpp>

#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#include "glm/glm.hpp"

class FlosdroneEstimator;

class FlosdroneUI
{
public:
    FlosdroneUI(FlosdroneEstimator* est);
    ~FlosdroneUI();

    void framebufferResizeCb(int width, int height);
    void mouseClickCb(int button, int action, int mods);
    void mouseScrollCb(double xoffset, double yoffset);

    void start();
    void stop();


private:
    FlosdroneEstimator* estimator_;

    bool run_;
    boost::mutex runMutex_;
    boost::thread* threadPtr_;

    GLFWwindow* window_;

    // Viewport size for video display.
    int imgVpWidth_, imgVpHeight_;
    // Whole window framebuffer size.
    int fbWidth_, fbHeight_;

    // Mouse button state and cursor location.
    bool leftClick_, rightClick_;
    double mouseXref_, mouseYref_;

    // Camera rotation, distance and center point location.
    double zrot_, xrot_;
    double camRadius_;
    double centerX_, centerY_;

    GLuint imgTex_;

    // Drone trail
    std::vector< glm::dvec3 > trail_;
    int trailOffset_;

    FlosdroneUI( const FlosdroneUI& other );
    FlosdroneUI& operator=( const FlosdroneUI& other );

    void run();

    void renderMap();
    void renderGrid();
    void renderDrone();

    void renderImage();

    void getDisplayCoordOnXYPlane(double dispX, double dispY, double &planeX, double &planeY);
};



#endif //FLOSDRONE_UI_H
