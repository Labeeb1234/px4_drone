# PX4 Drone - Autonomous Testing & Navigation Package

An open-source testing and development package for autonomous drone systems with advanced obstacle avoidance, path planning, and spatial understanding capabilities.


- **Got a bit bored today so trying this**
  [article/paper inside as article](https://mark-s-cleverley.medium.com/self-flying-drones-and-reinforcement-learning-dba670188c4c)
	- Keeping everything simple and training RL policy in isaaclab PPO algo
 	- Main Task: Flight controller using RL policy/RL Drone controller
  		- TO-DO List
    		- Model (prep USD file) --> using the x500 drone frame used for testing the custom [px4_sitl-isaac bridge]()
      		- Creating the Markov-Decision Problem/Env (MDP)
        		- Env Setup Done
          		- Create the "Reward Model Dense Model Required": Initially for (takeoff to a fixed height and stabilization)
            - May have to create custom force based action manager to simulate thrust force and reaction torque for propellers to lift the drone based on rotor angular velocity (need to test)
            	- the thrust force is simulated based on simple motor model using the thrust and torque constants and angular velocities of the drone motors
             		-[reference for drone motor dynamics modelling](https://medium.com/data-science/demystifying-drone-dynamics-ee98b1ba882f) --> note to self
               		- [another_note_to_self](https://andrew.gibiansky.com/downloads/pdf/Quadcopter%20Dynamics,%20Simulation,%20and%20Control.pdf)
                 	- [another_reference_paper](https://www.scitepress.org/Papers/2024/122963/122963.pdf) 
             	- thrust force (upward always) torque rotation based on motor force cancellation for quadcoptors
              	- (Note to self): use graphing calculators while designing dense reward models for RL tasks
              	- First training: somewhat successfull at 1000th episode (made it train for even longer after that its on the ground basically)
              	- 2nd training: takeoff and stable flight but not holding
              	- 3rd training: same as 2nd training (need to put reward func for holding and pos holding)
              	- 4th training: improved the reward structuring and weights (still not good enough drone just stubborn to takeoff now)
              	- 5th training: added the "floor is lava" penalty to force drone off the ground (haven't added orientation stabilization reward yet)
              	  
              	- 6th training: toppiling termination and its penalty for angling to unsafe orientations
              		- Reward's {alt_rew, pos_tracking, floor_is_lava(clip value change)} --> weight changes 
					- (PPO algo) horizon len, minibatch_size, num_envs {24,  24576, 1024}
              		- actions clampped to [-1.0, 1.0] with scale 1.0 (**note to self**: for scale > 1.0 remove clamp)
              	- 7th training
                 	- Reward's {alt_rew, pos_tracking, floor_is_lava(clip value change)} --> weight changes 
					- (PPO algo) horizon len, minibatch_size, num_envs {24,  24576, 1024}
              		- actions scaled by 5.0
              	 	- almost every iterations learned how to takeoff without need extra stabilization penalty but the drones aren't hovering (on to the next) within 500 episodes
              	    - results recorded
				- 8th training
              		- same condtions as 7th training but actions scale down by factor of 3.0

<!-- ## References & Attribution

This project builds upon:
- [PX4 Avoidance](https://github.com/PX4/PX4-Avoidance) - 3DVFH+* implementation (hardware tested local planner only)
- [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) - Trajectory optimization (only simulation as of now)
-->

**Last Updated**: Jun 2026 
**Repository**: [Labeeb1234/px4_drone](https://github.com/Labeeb1234/px4_drone) branch <feat/rl_quadcoptor_controller>
**Status**: Active Development








