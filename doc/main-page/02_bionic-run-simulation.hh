/**

\page sot_tiago_bionic_02_run_simulation Run Tiago's simulation on Ubuntu 18.04 Machine (Tiago's development computer and personal computer)

<h4> Testing on development machine with gazebo </h4>

In one termninal, run the following command to start tiago in gazebo:


- For Tiago's development computer:


    roslaunch /opt/pal/ferrum/share/tiago_gazebo/launch/tiago_gazebo.launch public_sim:=true robot:=steel end_effector:=schunk-wsg


- For personal computer: 


    roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=steel end_effector:=schunk-wsg
    
    
- Tiago also has other end effector: pal-gripper, pal-hey5 

In another, run the following command top stop the default controllers on Tiago:

    python /opt/openrobots/share/sot-tiago/tests/stop-controllers.py

On the third terminal, run the following command to start the SoT controllers on Tiago:

    roslaunch roscontrol_sot_tiago controller.launch use_mobile_base:=false simulation:=true end_effector:=schunk-wsg

Finally, on the second terminal, run the following command to test

    python /opt/openrobots/share/sot-tiago/tests/test.py


<b> To move the "wrist" to another position: </b>

First, Interacting with the dynamic_graph

    rosrun dynamic_graph_bridge run_command

Define the new target and command the wrist to the new position

    >> target = (0.5,0.5,0.5)
    >> gotoNd(taskRH,target,'111',(4.9,0.9,0.01,0.9))

<b> While interacting with the dynamic_graph, some informations of the robots could be accessed using these attributes: </b>

Base on the key - value pairs, the 'wrist' is actually 'arm_7_joint' which is the revolute joint between 'arm_6_link' and 'arm_7_link'

    >> robot.OperationalPointsMap

    {'left-wheel': 'wheel_left_joint', 'mobilebase': 'root_joint', 'gaze': 'head_2_joint', 'wrist': 'arm_7_joint', 'right-wheel': 'wheel_right_joint', 'footprint': 'base_footprint_joint'}

The command below show the current state of the joints

    >>robot.device.display()
    TIAGOSTEEL:  -0.0585965
      -0.285931
              0
              0
              -0
        0.932487
        0.300101
        0.057052
      -0.0107958
        -1.5708
        0.324973
          1.5708
        -1.41372
    4.19303e-12
      0.0306143
    -0.000700038
    -0.00907566

If Gazebo doesn't launch because you don't have a good graphic card, you can add this line to your .bashrc file :

    export LIBGL_ALWAYS_SOFTWARE=1

<h2> Possible errors: </h2>

<h3> Bound Violations: </h3>
These errors are the limits of either the angle positions of the revolute joints or the angular velocities.

    [ERROR] [1580719697.970259151, 86.019000000]: [TIAGOSTEEL]Robot position bound violation at DoF 6: requested -0.000525127 but set 0

The location of the value is set in the urdf file of the robot ("tiago_data/robots/tiago_steel.urdf")

For example, the configure of the revolute joint of arm 4.

    <joint name="arm_4_joint" type="revolute">
        <parent link="arm_3_link"/>
        <child link="arm_4_link"/>
        <origin rpy="-1.57079632679 -1.57079632679 0.0" xyz="-0.02 -0.027 -0.222"/>
        <axis xyz="0 0 1"/>
        <limit effort="17.86" lower="-0.392699081699" upper="2.35619449019" velocity="4.58"/>
        <dynamics damping="1.0" friction="1.0"/>
        <safety_controller k_position="20" k_velocity="20" soft_lower_limit="-0.322699081699" soft_upper_limit="2.28619449019"/>
      </joint>

<h3> Tiago's pose at start </h3>
The robot's pose is not zero in all axes. So the pose of the end-effector is affected a little. 
 */
