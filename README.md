# C3-Robot-Simulation
Simulation model for the C3 robots 

Update v3.0
2015-11-10
* Added method startSimulation() in class World which draws the simulation. For more details, see main.m and method start Simulation in class World.
* Robot controller is put inside startSimulation() for easy accessing
* Class World will now only save the references of the robot, allow one to
work with a C3RobotArm object outside of the class World.
* Removed old comments

Update v2.0
2015-10-29
Major updates and cleaned up most of the old code and changed the classes' help messages.
* Forward,- and inversekinematics implemented.
* End point orientation (x,y,z). 
* Error checking. Invalid input parameters to the robot methods will produce error messages.
* Removed functionality to change robot's base point (unnecessary feature)
* Possible to create a left or a right robot arm (default is a right robot arm)

Update v1.0
2015-10-26
I copied a previous project to get the OOP project to work. 
Robot arm is as of now drawn as lines for the links and circles for the
joint.
Following functionalities now exists:
* Creating robot arm with an off-set center point
* Drawing capabilities now exist
* Saving and using previous saved plot view 

