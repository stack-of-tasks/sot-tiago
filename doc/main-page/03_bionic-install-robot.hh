/**

\page sot_tiago_bionic_03_install_robot Installation of the software on the
robot

<h2> Introduction </h2>

The complete installation is done according to the following steps:

- Preparing the binary form of your software either from robotpkg binary
packages or through compilation inside the directory of development machine:
    /opt/openrobots

- Synchronizing the directory on the robot

<h2> Preparing development machine </h2>
<h3> Binaries </h3>

Please follow the steps on the previous pages of installing binaries on
development computer for Tiago

<h3> Synchronizing /opt/openrobots from development machine to the robot </h2>

- Connect to the robot with

        ssh pal@tiago-48c

- Become root:

        su

- Switch the filesystem to rw mode:

        rw
        chroot /ro

- Copy the files from development to tiago-48c:

        rsync -chavzP --stats pal@development:/opt/openrobots /opt

- Switch the filesystem to ro mode:

        exit
        ro
        exit

- The second exit should give you back the prompt as pal user.

- Reboot Tiago:

        ssh pal@tiago-48c
        sudo reboot


*/
