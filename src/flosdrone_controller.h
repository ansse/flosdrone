/*
 * flosdrone joystick controller class header file.
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

#ifndef FLOSDRONE_CONTROLLER_H
#define FLOSDRONE_CONTROLLER_H

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

class FlosdroneController
{
public:
    FlosdroneController();
    ~FlosdroneController();

    void joyCb(const sensor_msgs::Joy::ConstPtr& joy);

private:
    ros::NodeHandle nh_;

    ros::Subscriber joy_sub_;

    ros::Publisher takeoff_pub_;
    ros::Publisher land_pub_;
    ros::Publisher reset_pub_;
    ros::Publisher vel_pub_;

    // Button numbers
    int reset_button_;
    int takeoff_button_;
    int land_button_;

    // Previous button states
    int last_reset_;
    int last_takeoff_;
    int last_land_;

    // Thumbsticks' axis numbers
    int roll_axis_;
    int pitch_axis_;
    int yaw_axis_;
    int z_axis_;
};

#endif // FLOSDRONE_CONTROLLER_H
