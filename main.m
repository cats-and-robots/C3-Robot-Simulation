%% testing C3 model
clear all, close all, clc;


% How to create a robot arm
%robot = C3RobotArm();
robot = C3RobotArm(C3RobotArm.Right);
%robot = C3RobotArm(C3RobotArm.Left);

% How to set a target
target = [  -150; 450; -600]; % target in millimeter
%target = [  -abs(rand()); abs(rand()); -abs(rand())]*1000; 
target_Theta = [deg2rad(-90); 0; 0];

% How to create a simulation with a robot.
simulation = World(); % creates automatically a right robot arm in sim.
%simulation = World(robot); % you can also set the robot from the start
simulation.setRobot(robot); % replace any set robot with this one

% put in target in simulation (as of now, represented as a red cross)
simulation.setTarget(target);
% how to explicit set the robot's endpoint angle at the target  
%simulation.setTargetAngle(target_Theta); 

% Start the whole simulation
simulation.startSimulation();

% check current joint angles after simulation 
robot_angles_after_sim = robot.getJointAngles()
