################################ DOCUMENTATION #################################

Fork of the rviz::SelectionTool:
Drag with the left button to select objects in the 3D scene.
Hold the Alt key to change viewpoint as in the Move tool.

Additionally publishes selected points on /selected_points topic.

The tool can be loaded by clicking on the "Add new tool" button (plus sign) and
must be used to select points.

The plugin works starting with ROS/Rviz groovy.

################################ INSTALLATION ##################################

To install the Rviz plugin you basically put the sources into your catkin
workspace e.g. '~/catkin_ws/src' and run 'catkin_make install'

copy files to ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin_make install

If you encounter problems like a library cannot be found, consider changing your
workspace or changing the library path in the 'plugin_description.xml'. Take the
following documentations as well into account:

Creating a workspace for catkin
http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Building and using catkin packages in a workspace
http://wiki.ros.org/catkin/Tutorials/using_a_workspace

################################### LICENSE ####################################

 Copyright (C) 2013 Robotics & Biology Laboratory (RBO) TU Berlin

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

################################################################################

