/**

\page sot_tiago_xenial_install_simulation Software installation for Tiago on your Xenial Machine
Prepare your environment for robotpkg

You should create a file called robotpkg.list in directory /etc/apt/sources.lists.d. The contains of this file should be:

# cat /etc/apt/sources.list.d/robotpkg.list
deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub xenial robotpkg
deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub xenial robotpkg

It is important to put pal repository in first position so that all software versions work well together.

Please choose the appropriate distribution name. For instance if you are running Ubuntu 16.04 LTS you should use xenial

Download the key from robotpkg to recognize the apt-repository

curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key |
   sudo apt-key add -

Then update the list of available packages by typing:

sudo apt-get update

Prepare your environment for the PAL packages

If you do not already have it, install ROS

For Kinetic

You should install the following packages:

sudo apt-get install ros-kinetic-twist-mux ros-kinetic-joy-teleop ros-kinetic-moveit-ros-move-group ros-kinetic-humanoid-nav-msgs ros-kinetic-play-motion ros-kinetic-ompl ros-kinetic-moveit-planners-ompl ros-kinetic-moveit-simple-controller-manager ros-kinetic-control-toolbox ros-kinetic-four-wheel-steering-msgs ros-kinetic-urdf-geometry-parser ros-kinetic-gazebo-ros-control

PAL and Gepetto Software

From binaries

Please type:

sudo apt-get install robotpkg-sot-tiago robotpkg-tiago-dev

To have the simulation

sudo apt-get install robotpkg-tiago-simulation

The next step is to set your environment variables. You can automatize this by following the steps described here. Please read the instructions carefully. Setting bash variables might be tricky and lead to failure.

From source

The best way is to use robotpkg.

With robotpkg

    To install from source (still under testing too) please install robotpkg and robotpkg-wip.

    The instructions for installing robotpkg are given here. And the instructions for installing robotpkg-wip are given here. Once this is done you should go in the robotpkg-wip directory and then type:

     cd tiago-dev
     make install
 */
