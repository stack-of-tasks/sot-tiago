/**

\page sot_tiago_bionic_04_run_experiment_robot Running an Experiment

<h2> This page details how to run an experiment on Tiago </h2>

- The best practice is to test the movement on Gazebo before performing on the real robot \n

- Turn the Tiago on \n

- If necessary, transfer your packages from the development machine to the robot \n

- Check the Web Commander. Put the arm to the Home position \n

- On Tiago-48c, run: \n

        python /opt/openrobots/share/sot-tiago/tests/stop-controllers.py
  
    Run SoT on Tiago without using the mobile base \n

        roslaunch roscontrol_sot_tiago sot_tiago_controller.launch use_mobile_base:=false end-effctor:=schunk-wsg
    
- Run your experiment as you do it in simulation. For instance, on development machine \n
    
       python /opt/openrobots/share/sot-tiago/tests/test.py

    and/or \n 

        rosrun dynamic_graph_bridge run_command
     
    and run 

        >> target = (0.5,0.5,0.5)
        >> gotoNd(taskRH,target,'111',(4.9,0.9,0.01,0.9))
   
        

- Run the following command to restart PAL's deployer, in which SoT runs.
  This is required to restart SoT controller again

        pal_restart_deployer

- When you are done, turn the robot off.

- At the end of the day, please put Tiago back to its charging place.

*/


