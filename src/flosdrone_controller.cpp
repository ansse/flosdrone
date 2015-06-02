/*
 * flosdrone joystick controller class implementation.
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

#include "flosdrone_controller.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"


FlosdroneController::FlosdroneController():
        reset_button_(1), takeoff_button_(3), land_button_(2),
        last_reset_(0), last_takeoff_(0), last_land_(0),
        roll_axis_(2), pitch_axis_(3), yaw_axis_(0), z_axis_(1)
{
    // Subscribe to joystick topic
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10,
            &FlosdroneController::joyCb, this);

    // Advertise control topics
    reset_pub_ = nh_.advertise<std_msgs::Empty>("ardrone/reset",1);
    takeoff_pub_ = nh_.advertise<std_msgs::Empty>("ardrone/takeoff",1);
    land_pub_ = nh_.advertise<std_msgs::Empty>("ardrone/land",1);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);
}


FlosdroneController::~FlosdroneController()
{
}


void FlosdroneController::joyCb(const sensor_msgs::Joy::ConstPtr& joy)
{
    // Debug messages

    // Message for velocity control topic
    geometry_msgs::Twist cmd;

    cmd.linear.y = joy->axes[roll_axis_];
    cmd.linear.x = joy->axes[pitch_axis_];
    cmd.angular.z = joy->axes[yaw_axis_];
    cmd.linear.z = joy->axes[z_axis_];

    // Drone enters auto-hover mode when all 6 components are 0
    cmd.angular.x = 0;
    cmd.angular.y = 0;

    vel_pub_.publish(cmd);

    // Publish the reset/takeoff/land, but don't repeat when held down
    if(!last_reset_ && joy->buttons[reset_button_])
    {
        //std::cout << "reset pressed" << std::endl;
        reset_pub_.publish(std_msgs::Empty());
    }

    if(!last_takeoff_ && joy->buttons[takeoff_button_])
    {
        //std::cout << "takeoff pressed" << std::endl;
        takeoff_pub_.publish(std_msgs::Empty());
    }

    if(!last_land_ && joy->buttons[land_button_])
    {
        //std::cout << "land pressed" << std::endl;
        land_pub_.publish(std_msgs::Empty());
    }

    // Last button state for filtering out held down buttons
    last_reset_ = joy->buttons[reset_button_];
    last_takeoff_ = joy->buttons[takeoff_button_];
    last_land_ = joy->buttons[land_button_];
}
