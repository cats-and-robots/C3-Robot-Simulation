%% testing C3 model
clear all, close all, clc;


% How to create a robot arm
%robot = C3RobotArm();
robot = C3RobotArm(C3RobotArm.Right);
%robot = C3RobotArm(C3RobotArm.Left);

% How to set a target
%target = [  -150; 450; -600]; % target in millimeter
target = [  -abs(rand()); abs(rand()); -abs(rand())]*1000; 
target_Theta = [deg2rad(-90); 0; 0];

% How to get position of the robot arm (P and Theta).
% If 2nd parameter is true (now it is default as false), then it will
% update the position of the robot arm
[P, Theta] = robot.forwardKinematic(robot.getJointAngles());

% How to move the robot arm to a selected target+target angle Theta.
% Last parameter is "true" which means that we will update the robot model.
[picked_joint_angle_solution, joint_angle_solutions, flag_joint_angle_solutions] = ...
     robot.inverseKinematic(target, target_Theta, robot.getJointAngles(), true);


% How to create a simulation with a robot.
% simulation = World(); % creates automatically a right robot arm in sim.
simulation = World(robot);

% put in target in simulation (as of now, represented as a red cross)
simulation.setTarget(target);

% Draw the simulation
simulation.draw();




%% Test animation
% close all, clc
% 
% %filename = 'testgif.gif';
% dt=0.1;      %Animation timestep (10 frames per second)
% a=[1:100];
% 
% figure;
% hold on
% h1=plot(1,a(1));
% h2=plot(1,log(a(1)),'r');
% axis([ 0, 100, a(1), a(end)]);
% for i=2:100       
%     
%     t_loopstart=tic();  %Declaring time counter 
%     pause(0.01)
%     set(h1,'XData',(1:i),'YData',a(1:i))
%     set(h2,'XData',(1:i),'YData',log(a(1:i)))
%     title(sprintf('time: %.1f',i/10))
%     %refreshdata % old and slow. use set to update
%     
%     %Pausing animation    
%     el_time=toc(t_loopstart); 
%     pause(dt-el_time);
%     %drawnow % pause already calls drawnow automatically 
%     
% end



