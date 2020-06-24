/**

\page sot_tiago_xenial_03_install_robot Installation of the software on the robot

\b Introduction

The complete installation is done according to the following steps:

- Preparing the binary form of your software either from robotpkg binary packages or through compilation inside the directory of development machine:
    - /opt/openrobots

- Synchronizing the directory on the robot 

- Preparing development machine
    -Binaries
        - Setting a source.list file
        Robotpkg has two apt repository: a main repository and a work-in-progress repository: 

        sudo tee /etc/apt/sources.list.d/robotpkg.list <<EOF
        deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub $(lsb_release -cs) robotpkg
        deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg
        EOF

        Register the robotpkg authentication key
           curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -

        - Install the binary packages you want on akagisan. The following line installs all the needed packages for sot-tiago on akagisan:
           sudo apt-get update
           sudo apt-get install robotpkg-tiago-dev
           sudo apt-get install robotpkg-octomap

        - Verify the if the following file has been updated to use eigenpy
            /opt/openrobots/lib/python2.7/site-packages/dynamic_graph/sot/tiago/steel/prologue.py

        It should has the following lines to bridge eigenpy and numpy

           import eigenpy
           eigenpy.switchToNumpyMatrix()

    Copy the file setup-opt-robotpkg.sh to the home directory. You then should modify the file

        /home/pal/.bashrc

        such that it contains the following line:

        . /home/pal/setup-opt-robotpkg.sh

Synchronizing /opt/openrobots from development machine to the robot

    Connect to the robot with

        ssh pal@tiago-48c

    Become root:

        su

    Switch the filesystem to rw mode:

        rw
        chroot /ro

    Copy the files from development to tiago-48c:

        rsync -chavzP --stats pal@development:/opt/openrobots /opt

        Edit the following file in the robot

            /opt/pal/erbium/share/tiago_bringup/config/tiago_hardware.yaml

        Change the following line of yaml config:

           joint_mode_black_list: ['head_controller', 'whole_body_kinematic_controler', 'whole_body_kinematic_controller', 'torso_controller']

        to

           joint_mode_black_list: ['roscontrol_sot_tiago','tiago_roscontrol_test','head_controller', 'whole_body_kinematic_controler', 'whole_body_kinematic_controller', 'torso_controller']

        This change will allow the new controller works properly for tiago's torso 
    Switch the filesystem to ro mode:

        exit
        ro
        exit

        The second exit should give you back the prompt as pal user. 
    Reboot Tiago:

        ssh pal@tiago-48c
        sudo reboot

        This will log you out of tiago-48c. Or you can turn off and on again 


*/