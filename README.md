# flosdrone
Camera tracking with AR.Drone 2.0, ROS and OpenCV

## Installation and usage instructions

These instructions assume you don't have previous experience with ROS. If you do, you know what parts you can skip.

Install and configure ROS:

1. Install ROS Indigo Igloo according to instructions (http://wiki.ros.org/indigo/Installation/Ubuntu) and choose the Desktop-Full Install option.
2. Follow tutorial at http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment and create a catkin workspace. Reading the ROS tutorials further is higly recommended, but not required for running and installing this package. It's required only if you want to know what you're doing.

Install flosdrone build dependencies:

3. Install ardrone-autonomy package by running "apt-get install ros-indigo-ardrone-autonomy"
4. Install GLFW 3. First download the source package from http://www.glfw.org/download.html e.g. to your home directory. Extract package (e.g. with unzip commmand: "unzip glfw-3.1.1.zip" (your version may vary)). Checkout the compile instructions at www.glfw.org/docs/latest/compile.html and make sure you have all dependencies installed by running "apt-get install xorg-dev libglu1-mesa-dev". Go to the root of the extracted package and run "cmake ." to create makefiles for compiling. Run "make" to compile and "sudo make install" to install glfw. You can now remove the source files.
5. Install GLM library: "apt-get install libglm-dev".

Compile and test the software:

6. Clone flosdrone package to ~/catkin_ws/src/flosdrone, and run "catkin_make" at ~/catkin_ws/ to compile the project.
7. Open an extra terminal window/tab and run command "roscore" in it (in whatever directory). Now change back to the terminal still in ~/catkin_ws/ and run "rosrun flosdrone flosdrone_node" in it to start the flosdrone node (=program). A window with red box at the bottom and a grid at the should open. The position of drone is plotted to the grid and red box will show the image ffom the camera. You can rotate the grid view by holding down left mouse button, zoom with the scroll wheel and move the view by holding down the right mouse button.
8. Open a third terminal and go to ~/catkin_ws/src/flosdrone/bag directory and run "rosbag play test.bag -l" to start a playback of an Ar.Drone data recording. Now change back to the flosdrone window. Video with the found features higlighted in green should now be visible on the right half of the red block. Tracking the drone movement can be started with a left mouse click. It initializes the tracking algorithm to the latest video frame. The view used for tracking is now visible in the left side with red lines connecting the matching features between the tracked image and live image.

## License

This software is licensed under the GNU General Public License Version 3 (GPLv3), see http://www.gnu.org/licenses/gpl.html.
