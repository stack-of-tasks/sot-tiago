/**

\page sot_tiago_bionic_01_install_simulation Software installation for Tiago on Bionic Machine
<h3>Introduction </h3>
The complete installation is done according to the following steps:

<ul>
<li>Preparing the binary form of your software either from robotpkg binary packages or through compilation inside the directory of development machine:</li>
</ul>

    /opt/openrobots 

<ul>
<li>Synchronizing the directory on the robot.</li>
</ul>
<h3> Preparing development machine </h3>
<h4> Binaries </h4>
Setting a source.list file

    sudo tee /etc/apt/sources.list.d/robotpkg.list <<EOF
    deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub $(lsb_release -cs) robotpkg
    deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg
    EOF

Register the robotpkg authentication key

    curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -

Install the binary packages you want on akagisan. The following line installs all the needed packages for SoT on Tiago
    
    sudo apt-get update
    sudo apt-get install robotpkg-tiago-dev

In case the apt-get install failed, please try with aptitude install

Create the file setup-opt-robotpkg.sh to the home/bin directory.

    #!/bin/bash
    export ROBOTPKG_BASE=/opt/openrobots
    export PATH=$PATH:$ROBOTPKG_BASE/sbin:$ROBOTPKG_BASE/bin
    export LD_LIBRARY_PATH=$ROBOTPKG_BASE/lib:$ROBOTPKG_BASE/lib/plugin:$ROBOTPKG_BASE/lib64:$ROBOTPKG_BASE/lib/dynamic-graph-plugins:$LD_LIBRARY_PATH
    export PYTHONPATH=$PYTHONPATH:$ROBOTPKG_BASE/lib/python2.7/site-packages:$ROBOTPKG_BASE/lib/python2.7/dist-packages
    export PKG_CONFIG_PATH=$ROBOTPKG_BASE/lib/pkgconfig/:$PKG_CONFIG_PATH
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROBOTPKG_BASE/share:$ROBOTPKG_BASE/stacks
    export CMAKE_PREFIX_PATH=$ROBOTPKG_BASE:$CMAKE_PREFIX_PATH

source the setup-opt-robotpkg.sh file update the environment variables


<h4> Dealing with missing libraries and paths </h4>

In case some libraries are missising due to robotpkg's issue, please git the related package into to the following folder. Then use catkin to build the packages

    /home/pal/catkin_ws/src

It is prefered to run the following command to use install space before building

    catkin config --install

The environment paths are important to make sure that Sot will not use wrong library with same name from ros-melodic. for example, LD_LIBRARY_PATH should have the robotpkg's lib in priority

    echo $LD_LIBRARY_PATH
    /opt/openrobots/lib:/opt/openrobots/lib/plugin:/opt/openrobots/lib64:/opt/openrobots/lib/dynamic-graph-plugins:/home/pal/catkin_ws/install/lib:/opt/pal/ferrum/lib:/opt/pal/ferrum/lib/x86_64-linux-gnu:/opt/ros/melodic/lib:/opt/openrobots/lib

In order to do so, in /home/pal/.bashrc file, include these lines with respective order

    source /home/pal/catkin_ws/install/setup.bash
    source /home/pal/bin/setup-opt-robotpkg.sh

Some time, robotpkg packages will points to moveit libraries whose versions exceed Pal's installed package. In that case, please execute these steps to update

Setup development computer to accept software from packages.ros.org.

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    sudo apt update

Run the following command to upgrade the ros's packages

    sudo apt upgrade

*/
