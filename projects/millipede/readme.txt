Guide for millipede robot simulation

The project has been divided in different parts, with different control systems implemented:

centralized_cpg : test of implemented robot, uses a single CPG to form a tripod like gait

independent_cpg : fixed local sensory feedback

adaptiveSF : adaptive local sensory feedback strength

gait_learning: same controller as adaptiveSF including a learning system to learn interconnections between CPGs (not finished)

navigation: tests for navigation control of millipede (not finished)

--------------------------------------------------------

All controllers are included in controllers/millipede

The localLegController class includes the CPGs and processes used in each local controller

In dataCollection a function to save data in a txt file was included.

--------------------------------------------------------

The number of segments of the millipede can be set up when getting the configuration of the robot with:
lpzrobots::MillipedeConf myMillipedeConf = lpzrobots::Millipede::getDefaultConf(1.0 /*_scale*/, 4 /*_legspersegment*/, 3/*_nofsegments!!*/, 1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/);

--------------------------------------------------------

All simulations are ready to be reset updating the parameters of the controllers a fixed number of times.
By modifying the parameters to be updated, their values and modification after each reset.
