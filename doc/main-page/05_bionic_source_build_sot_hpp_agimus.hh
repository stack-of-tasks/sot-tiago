/**
\page sot_tiago_bionic_05_build_sot_hpp_agimus Source build for Tiago to include SoT, Agimus and HPP

<b> Install prerequisites packages</b>

    sudo apt-get install -y libopenscenegraph-dev \
                            libzbar-dev \
                            ros-kinetic-visp-bridge ros-kinetic-visp \
                            ros-kinetic-eus-qpoases \
                            omniidl-python \
                            python3-vcstool \
                            robotpkg-gerard-bauzil

<b> Download the source codes</b>

In catkin_ws/src folder, download the packages using vcs:

    vcs import --recursive < sot_tiago.repos
    vcs import --recursive < hpp.repos
    vcs import --recursive < agimus.repos

<b> Build Ordering</b>

On the Development machine:

1. Run the following cmake config arguments

    catkin config -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python2 -DBUILD_TESTING=OFF



2. Build the packages in the following order and arguments

    - catkin build hpp-fcl
    - catkin build hpp-manipulation-corba -DHPP_MANIPULATION_HAS_WHOLEBODY_STEP=FALSE
    - catkin build agimus-hpp -DBUILD_HPP_PLUGIN=ON -DBUILD_ROS_INTERFACE=ON
    - catkin build

*/
