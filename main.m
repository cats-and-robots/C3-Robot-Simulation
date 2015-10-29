%% testing C3 model
clear all, close all, clc;


%robot = C3RobotArm();
robot = C3RobotArm(C3RobotArm.Right);
%robot = C3RobotArm(C3RobotArm.Left);

%target = [ -0.250 ; 0.150; -0.650] * 1000; 
target = [  -150; 450; -600]; % target in millimeter
target_Theta = [deg2rad(-135); 0; 0];

[P, Theta] = robot.forwardKinematic(robot.getJointAngles());

[picked_joint_angle_solution, joint_angle_solutions, flag_joint_angle_solutions] = ...
    robot.inverseKinematic(target, target_Theta, robot.getJointAngles(), true);


simulation = World(robot);
simulation.setTarget(target);
simulation.draw();



%simulation.start();


%% Test animation
close all, clc

%filename = 'testgif.gif';
dt=0.1;      %Animation timestep (10 frames per second)
a=[1:100];

figure;
hold on
h1=plot(1,a(1));
h2=plot(1,log(a(1)),'r');
axis([ 0, 100, a(1), a(end)]);
for i=2:100       
    
    t_loopstart=tic();  %Declaring time counter 
    pause(0.01)
    set(h1,'XData',(1:i),'YData',a(1:i))
    set(h2,'XData',(1:i),'YData',log(a(1:i)))
    title(sprintf('time: %.1f',i/10))
    %refreshdata % old and slow. use set to update
    
    %Pausing animation    
    el_time=toc(t_loopstart); 
    pause(dt-el_time);
    %drawnow % pause already calls drawnow automatically 
    
end


