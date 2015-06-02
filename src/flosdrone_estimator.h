/*
 * flosdrone pose estimation class header file.
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

#ifndef FLOSDRONE_ESTIMATOR_H
#define FLOSDRONE_ESTIMATOR_H

#include "ros/ros.h"
#include <boost/thread.hpp>
#include <fstream>

#include "image_transport/image_transport.h"
#include "ardrone_autonomy/Navdata.h"

#include "geometry_msgs/Pose.h"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/features2d/features2d.hpp"

#define GLM_FORCE_RADIANS
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"



class FlosdroneEstimator
{
public:
    FlosdroneEstimator(std::string filenameTime, std::string filenameCoord);
    ~FlosdroneEstimator();

    void cameraCb(const sensor_msgs::ImageConstPtr& msg,
            const sensor_msgs::CameraInfoConstPtr& cam_info);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    void getPose(geometry_msgs::Pose& pose);
    void getTrail(std::vector< glm::dvec3 >& trail, int& offset);
    bool getImage(cv::Mat& img);

    void imageClicked();
private:
    ros::NodeHandle nh_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::CameraSubscriber camera_sub_;

    cv_bridge::CvImagePtr cv_ptr_;
    cv::Mat cam_;
    cv::Mat dist_;

    cv::Mat map1_;
    cv::Mat map2_;

    cv::ORB orbDetDesc_;
    cv::Mat trackedObjImage_;
    std::vector<cv::KeyPoint> keypointsObj_;
    cv::Mat descriptorsObj_;

    cv::Mat displayImage_;

    bool imgUpdated_;
    boost::mutex imgMutex_;

    std::vector< glm::dvec3 > trailPoints_;
    unsigned int trailPointIdx_;
    bool trailUpdated_;
    boost::mutex trailMutex_;


    glm::dvec3 position_;
    glm::dvec3 posiOffset_;
    glm::dquat orientation_;

    boost::mutex posMutex_;
    boost::mutex oriMutex_;


    std::vector< double > imgDts_;
    unsigned int imgDtsOffset_;
    ros::Time lastImgTimeStamp_;

    std::ofstream timeFileStream_;
    std::ofstream coordFileStream_;
};

#endif //FLOSDRONE_ESTIMATOR_H
