/**

\page sot_tiago_xenial_04_run_experiment_robot Running an Experiment

<h2> This page details how to run an experiment on Tiago </h2>

- The best practice is to test the movement on Gazebo before performing on the real robot \n

- Turn the Tiago on \n

- If necessary, transfer your packages from the development machine to the robot \n

- Check the Web Commander. Put the arm to the Home position \n

- On Tiago-48c, run: \n

        python /opt/openrobots/share/sot-tiago/tests/stop-controllers.py

        pal-stop deployer
        >> result: "deployer stopped successfull

        pal-start deployer
        >> result: "deployer started"
  
    Run SoT on Tiago without using the mobile base \n

        roslaunch roscontrol_sot_tiago sot_tiago_controller.launch use_mobile_base:=false
    
- Run your experiment as you do it in simulation. For instance, on development machine \n
    
        source ~/setup-opt-robotpkg.sh
        cd <test directory>
        python test.py

    and/or \n 

        source ~/setup-opt-robotpkg.sh
        rosrun dynamic_graph_bridge run_command
        

    The content of the script setup-opt-robotpkg.sh:

        #!/bin/bash 
        export ROBOTPKG_BASE=/opt/openrobots
        export PATH=$PATH:$ROBOTPKG_BASE/sbin:$ROBOTPKG_BASE/bin
        export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROBOTPKG_BASE/lib:$ROBOTPKG_BASE/lib/plugin:$ROBOTPKG_BASE/lib64
        export PYTHONPATH=$PYTHONPATH:$ROBOTPKG_BASE/lib/python2.7/site-packages:$ROBOTPKG_BASE/lib/python2.7/dist-packages
        export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$ROBOTPKG_BASE/lib/pkgconfig/
        export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROBOTPKG_BASE/share:$ROBOTPKG_BASE/stacks
        export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$ROBOTPKG_BASE

- Run the following command to restart PAL's deployer, in which SoT runs.
  This is required to restart SoT controller again

        pal-stop ros_bringup && pal-stop deployer && pal-start deployer && pal-start ros_bringup

- When you are done, turn the robot off.

- At the end of the day, please put Tiago back to its charging place.

*/


