/*
 * flosdrone pose estimation class implementation.
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

#include "flosdrone_estimator.h"

#include "sensor_msgs/image_encodings.h"

#include "glm/gtx/quaternion.hpp"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"


const double PI = 3.14159265;
const double G = 9.80665;

const int CALIBCOUNT = 100;
const double FILTERFACTOR = 0.001;
const double FILTERFACTOR2 = 0.001;



FlosdroneEstimator::FlosdroneEstimator(std::string filenameTime, std::string filenameCoord): it_(nh_), imgUpdated_(false),
        trailPoints_(10000, glm::dvec3(0.0,0.0,0.0)), trailPointIdx_(0),
        position_(0.0,0.0,0.0), posiOffset_(0.0,0.0,0.0),
        imgDts_(30, 0.0), imgDtsOffset_(0),
        orbDetDesc_(1000, 1.412f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31)
{
    camera_sub_ = it_.subscribeCamera("ardrone/image_raw", 5,
            &FlosdroneEstimator::cameraCb, this,
            image_transport::TransportHints("compressed"));
    if(filenameTime != "")
    {
        timeFileStream_.open(filenameTime.c_str(), std::ios::binary);
        if(!timeFileStream_)
        {
            std::cerr << "Opening time output file " << filenameTime << "failed!" << std::endl;
        }
        else
        {
            std::cout << "Time output file " << filenameTime << " opened." << std::endl;
        }
    }
    if(filenameCoord != "")
    {
        coordFileStream_.open(filenameCoord.c_str(), std::ios::binary);
        if(!timeFileStream_)
        {
            std::cerr << "Opening coordinate output file " << filenameCoord << "failed!" << std::endl;
        }
        else
        {
            std::cout << "Coordinate output file " << filenameCoord << " opened." << std::endl;
        }
    }
}

FlosdroneEstimator::~FlosdroneEstimator()
{
    if(timeFileStream_.is_open())
    {
        timeFileStream_.close();
        std::cout << "Time output file closed." << std::endl;
    }
    if(coordFileStream_.is_open())
    {
        coordFileStream_.close();
        std::cout << "Coordinate output file closed." << std::endl;
    }

}

void FlosdroneEstimator::cameraCb(const sensor_msgs::ImageConstPtr& msg,
        const sensor_msgs::CameraInfoConstPtr& cam_info)
{
    // Load the camera calibration parameters to cv matrices
    cam_ = cv::Mat(3,3, CV_64FC1);
    dist_ = cv::Mat(5,1, CV_64FC1);

    double* ptr = cam_.ptr<double>(0);
    for(unsigned int i = 0; i < 9; ++i){
        ptr[i] = cam_info->K[i];
    }

    ptr = dist_.ptr<double>(0);
    for(unsigned int i = 0; i < 5; ++i){
        ptr[i] = cam_info->D[i];
    }

    // Create the maps for image undistortion
    cv::initUndistortRectifyMap(cam_, dist_, cv::Mat(), cam_,
            cv::Size(cam_info->width, cam_info->height), CV_32FC1, map1_,
            map2_);

    cam_.at<double>(2) = cam_info->width / 2.0;
    cam_.at<double>(5) = cam_info->height / 2.0;

    posiOffset_ = glm::dvec3(cam_.at<double>(2), cam_.at<double>(5), -cam_.at<double>(0));

    // Let's make sure this doesn't happen again
    image_sub_ = it_.subscribe("ardrone/image_raw", 5,
            &FlosdroneEstimator::imageCb, this,
            image_transport::TransportHints("compressed"));
    camera_sub_.shutdown();

}

void FlosdroneEstimator::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    ros::Time begin = ros::Time::now();

    ros::Duration t = msg->header.stamp - lastImgTimeStamp_;
    lastImgTimeStamp_ = msg->header.stamp;
    if(t.sec == 0)
    {
        double dt = (double)t.nsec/1000000000.0;
        ++imgDtsOffset_;
        imgDtsOffset_ %= imgDts_.size();
        imgDts_.at(imgDtsOffset_) = dt;
        if(imgDtsOffset_ == 0)
        {
            double hz = 0;
            for(unsigned int i = 0; i < imgDts_.size(); ++i)
            {
                hz += imgDts_.at(i);
            }
            hz /= imgDts_.size();
            std::cout << "Video freq: " << 1/hz << "Hz" << std::endl;
        }
    }

    imgMutex_.lock();

    // Translate imageMessage to cvImage
    try
    {
        cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Undistort the image based on calibration data (check cameraCb)
    cv::remap( cv_ptr_->image, cv_ptr_->image, map1_, map2_, cv::INTER_LINEAR );
    displayImage_ = cv_ptr_->image.clone();

    cv::Mat gray;
    // convert RGB image to gray
    cvtColor(cv_ptr_->image, gray, CV_BGR2GRAY);



    std::vector<cv::KeyPoint> keypointsScene;
    orbDetDesc_.detect(gray, keypointsScene);
    cv::Mat descriptorsScene;
    orbDetDesc_.compute(gray, keypointsScene, descriptorsScene);

    if(trackedObjImage_.dims != 0 && keypointsScene.size() > 0)
    {


        cv::BFMatcher matcher(cv::NORM_HAMMING, true);
        std::vector< std::vector<cv::DMatch> > matches;
        matcher.knnMatch(descriptorsObj_, descriptorsScene, matches,1);

        double max_dist = 0; double min_dist = 100;
        // Quick calculation of max and min distances between keypoints
        for( int i = 0; i < descriptorsObj_.rows; ++i )
        {
            if(!matches.at(i).empty()){
                double dist = matches.at(i).at(0).distance;
                if(dist < min_dist)
                    min_dist = dist;
                if(dist > max_dist)
                    max_dist = dist;
            }
        }

        // Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
        std::vector< cv::DMatch > good_matches;

        for( int i = 0; i < descriptorsObj_.rows; ++i )
        {
            if(!matches.at(i).empty() && matches.at(i).at(0).distance <= 2*(3+min_dist) )
            {
                good_matches.push_back( matches.at(i).at(0));
            }
        }
        
        cv::Mat img_matches;
        drawMatches(trackedObjImage_, keypointsObj_, cv_ptr_->image, keypointsScene, good_matches, img_matches, cv::Scalar(0,0,255), cv::Scalar(0,255,0));

        if(good_matches.size() >= 4)
        {
            //-- Localize the object
            std::vector<cv::Point2f> obj;
            std::vector<cv::Point3f> obj3d;
            std::vector<cv::Point2f> scene;

            for( unsigned int i = 0; i < good_matches.size(); i++ )
            {
                //-- Get the keypoints from the good matches
                obj.push_back( keypointsObj_[ good_matches[i].queryIdx ].pt );
                obj3d.push_back( cv::Point3f(keypointsObj_[ good_matches[i].queryIdx ].pt.x, keypointsObj_[ good_matches[i].queryIdx ].pt.y, 0.f) );

                scene.push_back( keypointsScene[ good_matches[i].trainIdx ].pt );
            }

            cv::Mat rvec;
            cv::Mat tvec;

            solvePnPRansac(obj3d, scene, cam_, cv::Mat(), rvec, tvec, false, 100, 2.0);

            cv::Mat rotMat;
            cv::Rodrigues(rvec, rotMat);
            cv::Mat transVec= rotMat.t()*tvec;
            cv::Mat rotVec = rvec;


            glm::dquat xRotQuat = glm::angleAxis((90 * PI / 180.0), glm::dvec3(1.0, 0.0, 0.0));


            glm::dvec3 position = glm::dvec3(transVec.at<double>(0), transVec.at<double>(1), transVec.at<double>(2));
            position += posiOffset_;
            position /= -100;
            position = position * xRotQuat;

            glm::dvec3 oriVec = glm::dvec3(rotVec.at<double>(0), rotVec.at<double>(1), rotVec.at<double>(2));
            oriVec = oriVec * xRotQuat;
            glm::dquat oriQuat = glm::angleAxis(glm::length(oriVec), glm::normalize(oriVec));

            // Update the "public" pose
            posMutex_.lock();

            position_ = position;
            orientation_ = oriQuat;

            posMutex_.unlock();

            // Update the trail
            trailMutex_.lock();

            ++trailPointIdx_;
            trailPointIdx_ %= trailPoints_.size();
            trailPoints_.at(trailPointIdx_) = position;

            trailMutex_.unlock();

            // Output position to binary file
            if(coordFileStream_.is_open())
            {
                struct Position
                {
                    double x;
                    double y;
                    double z;
                } posi;

                posi.x = position.x;
                posi.y = position.y;
                posi.z = position.z;

                coordFileStream_.write(reinterpret_cast<const char*>(&posi), sizeof(posi));
            }

            std::vector<cv::Point3f> obj_corners(4);
            obj_corners[0] = cv::Point3f(0,0,0);
            obj_corners[1] = cv::Point3f( trackedObjImage_.cols, 0, 0 );
            obj_corners[2] = cv::Point3f( trackedObjImage_.cols, trackedObjImage_.rows, 0 );
            obj_corners[3] = cv::Point3f( 0, trackedObjImage_.rows, 0 );

            std::vector<cv::Point2f> scene_corners(4);

            projectPoints(obj_corners, rvec, tvec, cam_, cv::Mat(), scene_corners);

            cv::Point2f trackImgWidth ( trackedObjImage_.cols, 0);

            // Draw lines surrounding tracked object
            cv::line( img_matches, scene_corners[0] + trackImgWidth, scene_corners[1] + trackImgWidth, cv::Scalar(0, 255, 0), 4 );
            cv::line( img_matches, scene_corners[1] + trackImgWidth, scene_corners[2] + trackImgWidth, cv::Scalar( 0, 255, 0), 4 );
            cv::line( img_matches, scene_corners[2] + trackImgWidth, scene_corners[3] + trackImgWidth, cv::Scalar( 0, 255, 0), 4 );
            cv::line( img_matches, scene_corners[3] + trackImgWidth, scene_corners[0] + trackImgWidth, cv::Scalar( 0, 255, 0), 4 );

        }

        displayImage_ = img_matches.clone();

        if(timeFileStream_.is_open())
        {
            ros::Time now = ros::Time::now();
            ros::Duration dur = now - begin;
            double time = dur.toSec();
            std::cout << "S: " << time << " F: " << 1/time << std::endl;
            timeFileStream_.write(reinterpret_cast<const char*>(&time), sizeof(time));
        }
    }
    else
    {
        // Draw only the live image with detected scene keypoints
        drawKeypoints(displayImage_, keypointsScene, displayImage_, cv::Scalar(0,0,255));

        // The image display is twice width of camera image. Let's fill the
        // left half with red and draw the camera image to the right half.
        cv::Mat doubleWidthImg;
        doubleWidthImg.create(displayImage_.rows, 2 * displayImage_.cols, displayImage_.type());
        doubleWidthImg.setTo(cv::Scalar(0,0,255));
        displayImage_.copyTo(doubleWidthImg( cv::Rect( displayImage_.cols, 0, displayImage_.cols, displayImage_.rows)));
        displayImage_ = doubleWidthImg;
    }

    imgUpdated_ = true;
    imgMutex_.unlock();
}

void FlosdroneEstimator::getPose(geometry_msgs::Pose& pose)
{
    oriMutex_.lock();
    pose.orientation.x = orientation_.x;
    pose.orientation.y = orientation_.y;
    pose.orientation.z = orientation_.z;
    pose.orientation.w = orientation_.w;
    oriMutex_.unlock();

    posMutex_.lock();
    pose.position.x = position_.x;
    pose.position.y = position_.y;
    pose.position.z = position_.z;
    posMutex_.unlock();
}

void FlosdroneEstimator::getTrail(std::vector< glm::dvec3 >& trail, int& offset)
{
    trailMutex_.lock();
    trail = trailPoints_;
    offset = trailPointIdx_;
    trailMutex_.unlock();
}

bool FlosdroneEstimator::getImage(cv::Mat& img)
{
    imgMutex_.lock();
    if(imgUpdated_)
    {
        img = displayImage_.clone();
        imgUpdated_ = false;
        imgMutex_.unlock();
        return true;
    }
    imgMutex_.unlock();
    return false;
}

void FlosdroneEstimator::imageClicked()
{
    imgMutex_.lock();
    if(cv_ptr_ != 0)
    {
        trackedObjImage_ = cv_ptr_->image.clone();
        cv::Mat grayTrack;
        cvtColor(trackedObjImage_, grayTrack, CV_BGR2GRAY);
        orbDetDesc_.detect(grayTrack, keypointsObj_);
        orbDetDesc_.compute(grayTrack, keypointsObj_, descriptorsObj_);
    }
    imgMutex_.unlock();
}
