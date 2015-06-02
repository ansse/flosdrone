/*
 * flosdrone ROS node main file.
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

#include "ros/ros.h"

#include "flosdrone_controller.h"
#include "flosdrone_estimator.h"
#include "flosdrone_ui.h"


//###########################################################################//

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ardrone_test");

    std::string filenameTime = "";
    std::string filenameCoordinates = "";

    for(int i = 1; i < argc; ++i)
    {
        if(std::string("--time") == std::string(argv[i]) && i+1 < argc)
        {
            ++i;
            filenameTime = argv[i];
        }
        else if(std::string("--coord") == std::string(argv[i]) && i+1 < argc)
        {
            ++i;
            filenameCoordinates = argv[i];
        }
    }

    FlosdroneEstimator est(filenameTime, filenameCoordinates);
    FlosdroneUI ui(&est);
    FlosdroneController con;

    ui.start();
    ros::spin();
    ui.stop();

    return 0;
}
