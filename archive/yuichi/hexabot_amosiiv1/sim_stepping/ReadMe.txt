Simulation of amosiiv1 stepping in place with phase reset and inhibition for journal paper of Yuichi, 2014

Original from Yuichi, 
Yuichi_gorobots_fork >> ashigaru_LpzKOH >> projects/hexabot/hexabot_simulator9_amosv1_preExp1


///
This is the explanation about the programs

There are many classes for simulation in this file, but please just see main.cpp 

****************************************************************************** 
In main.cpp 

*Functions
  1,Make the robot and controller. 
  2,Select the simulation mode. 
  3,Set the initial values for simulation. 
  4,Manage the simulation procedure

*Maybe Change points??
 In main.cpp  
 #comment 0 (line 94)
  you can set the folder which you want to save the files

 #comment 1 (line 103)
  you can change the simulation speed
  If it is false, the movie is available and simulation is realtime

 #comment 2 (line 178)
  Initial value for the simulation (please read below)

 #comment 3 (line 332)
  The robot is declared here

*About Compile
 Notice that this programs depend on "AmosII.h, -.cpp, -sensormotordefinition.h".


*******************************************************************************
Preliminary experiments procedure

If it is possible, can you kindly also send the video of the movement also (Just part of the experiments, for example, 1 [min] of each gait is enough for me) ? 

+++++++++++++++++++++++++++++++++
1, For Backward wave motion

1-a, In main.cpp, search #comment 0 (line 94) and choose following 

 #define FILE_NAME "Backward_leg_sw000_ST10_fixedPs" // For Backward

1-b, In main.cpp, search #comment 2 (line 178) and choose following

 rlConf.initialState = osg::Vec2d(4.43, 4.47); // For Backward

1-c, Compile and Run simulation on flat surface, it takes almost 25 [min]

 The robot will never walk, it will just step on the same place.
 I think you do not need to touch the robot for 25[min]. 
 Current time is represented on console. 
 First 400[s], robot just steps at same duty rate to converge, 
 Next 1000[s], robot steps by changing duty rate gradually. 
 
1-d, After that, 3 files are available at the folder you set
 "1_runLog_Backward_leg_sw000_ST10_fixedPs_timed.dat"
 "1_runLog_Backward_leg_sw000_ST10_fixedPs_mapped.dat" 
 "Configs_Backward_leg_sw000_ST10_fixedPs.dat"

+++++++++++++++++++++++++++++++++++
2, For Backward wave motion

2-a, In main.cpp, search #comment 0 (line 94) and choose following 

 #define FILE_NAME "Travelling_leg_sw000_ST10_fixedPs" // For Travelling

2-b, In main.cpp, search #comment 2 (line 178) and choose following

 rlConf.initialState = osg::Vec2d(1.8, 1.84); // For Travelling

2-c, Compile and Run simulation for almost 25 [min] on flat surface

 The robot will never walk, it will just step on the same place. 
 I think you do not need to touch the robot for 25[min].
 Current time is represented on console. 
 First 400[s], robot just steps at same duty rate to converge, 
 Next 1000[s], robot steps by changing duty rate gradually. 
 
2-d, 3 files are available at the folder you set
 "1_runLog_Travelling_leg_sw000_ST10_fixedPs_timed.dat"
 "1_runLog_Travelling_leg_sw000_ST10_fixedPs_mapped.dat" 
 "Configs_Travelling_leg_sw000_ST10_fixedPs.dat"

*******************************************************************************

Thank you for helping me and for taking your time for this.
If you have any questions and problems, please ask me freely. 

Yuichi
