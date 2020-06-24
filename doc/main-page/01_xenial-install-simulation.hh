/**

\page sot_tiago_xenial_01_install_simulation Software installation for Tiago on your Xenial Machine

<b> Prepare your environment for robotpkg </b>

You should create a file called <b> robotpkg.list </b>  in directory <b> /etc/apt/sources.lists.d. </b>  The contains of this file should be:

    \code
    # cat /etc/apt/sources.list.d/robotpkg.list
    deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub xenial robotpkg
    deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub xenial robotpkg
    \endcode

It is important to put pal repository in first position so that all software versions work well together.

Please choose the appropriate distribution name. For instance if you are running Ubuntu 16.04 LTS you should use xenial

Download the key from robotpkg to recognize the apt-repository

\code
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key |
   sudo apt-key add -
\endcode

Then update the list of available packages by typing:

\code sudo apt-get update \endcode

<b>  Prepare your environment for the PAL packages </b>

If you do not already have it, install ROS

<b>  For Kinetic </b>

You should install the following packages:

\code
sudo apt-get install ros-kinetic-twist-mux ros-kinetic-joy-teleop ros-kinetic-moveit-ros-move-group ros-kinetic-humanoid-nav-msgs ros-kinetic-play-motion ros-kinetic-ompl ros-kinetic-moveit-planners-ompl ros-kinetic-moveit-simple-controller-manager ros-kinetic-control-toolbox ros-kinetic-four-wheel-steering-msgs ros-kinetic-urdf-geometry-parser ros-kinetic-gazebo-ros-control
\endcode

<b>  PAL and Gepetto Software </b>

<b> From binaries </b>

Please type:

\code
sudo apt-get install robotpkg-sot-tiago robotpkg-tiago-dev
\end code

To have the simulation

\code sudo apt-get install robotpkg-tiago-simulation \endcode

The next step is to set your environment variables. You can automatize this by following the steps described here. Please read the instructions carefully. Setting bash variables might be tricky and lead to failure.

From source

The best way is to use robotpkg.

With robotpkg

To install from source (still under testing too) please install robotpkg and robotpkg-wip.

The instructions for installing robotpkg are given here. And the instructions for installing robotpkg-wip are given here. Once this is done you should go in the robotpkg-wip directory and then type:

    \code
    cd tiago-dev
    make install
    \endcode
*/
