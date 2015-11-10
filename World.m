classdef World < AbstractClassDraw
    %WORLD Class that represent the whole simulation world. It contains a
    %human and a robot, and it is from here that the logical interaction
    %between then takes place.
    
    properties (Access = protected)
        target
        target_Theta = [deg2rad(-90); 0; 0]; % default target angle
        figure_handle
        axes_handle
        robot
        
    end
    properties (Access = public)

    end % end of properties
    
    methods
        function obj = World(varargin)
            if nargin == 0
                obj.robot = C3RobotArm(); % created a default right-arm robot
            elseif nargin == 1
                if ~isa(varargin{1}, 'C3RobotArm')
                    error('Input argument is not a C3 robot arm!');
                end
                %obj.robot = copy(varargin{1}); %deep-copy
                obj.robot = varargin{1}; % reference copy
            else
                error('Too many input paramters to constructor!');
            end
            
        
        end % end of constructor
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function draw(obj)
            %DRAW Draws the first initial frame for all object in a figure
            %set by figure_handle.
            
            % setup the figure for the simulation
            obj.figure_handle = figure;
            set(obj.figure_handle, ...
                'Name','C3 Robot Simulation', ...
                'NumberTitle','off'...
                ); 
            
            % setup the figure axes (view of the plot) for the simulation
            % the view setting is stored in a mat file 
            obj.axes_handle = gca;
            if exist('plot_view.mat', 'file')
                load('plot_view');
                set(obj.axes_handle, 'PlotBoxAspectRatio',pba);
                set(obj.axes_handle, 'DataAspectRatio',dar);
                set(obj.axes_handle, 'CameraViewAngle',cva);
                set(obj.axes_handle, 'CameraUpVector',cuv);
                set(obj.axes_handle, 'CameraTarget',ct);
                set(obj.axes_handle, 'CameraPosition',cp);
            end
            
            %initial setup of the plot window
            hold on
            grid on
            axis equal
            xlabel('x [mm]'), ylabel('y [mm]'), zlabel('z [mm]') 
            axis([-1000 1000 -1000 1000 -1000 200 ])
            
            % draw robot
            obj.robot.draw();
            
            % draw target
            if ~isempty(obj.target)
                plot3(obj.target(1),obj.target(2),obj.target(3), ...
                    'x','Color', 'red', 'MarkerSize',15, 'LineWidth', 3);
            end
            
        end % end of draw
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function drawUpdate(obj)
            %DRAW Updates the animation frame for all objects
            obj.robot.drawUpdate();
        end % end of draw
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function savePlotView(obj)
            if isempty(obj.axes_handle)
                error( sprintf(['Cannot save the simulation view!' ...
                    '\nPlease call method draw() to create a simulation view'])); 
            end
            
            pba = get(obj.axes_handle, 'PlotBoxAspectRatio');
            dar = get(obj.axes_handle, 'DataAspectRatio');
            cva = get(obj.axes_handle, 'CameraViewAngle');
            cuv = get(obj.axes_handle, 'CameraUpVector');
            ct = get(obj.axes_handle, 'CameraTarget');
            cp = get(obj.axes_handle, 'CameraPosition');
            
            save('plot_view.mat', 'pba', 'dar', 'cva', 'cuv', 'ct', 'cp');
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setRobot(obj, robot)
            if ~isa(robot, 'C3RobotArm')
                error('Input argument is not a C3 robot arm!');
            end
            obj.robot = robot;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setTarget(obj, target)
            obj.check3ColumnVector(target);
            obj.target = target;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function target = getTarget(obj)
            target = obj.target;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setTargetAngle(obj,target_Theta)
            obj.check3ColumnVector(target_Theta);
            obj.target_Theta = target_Theta;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function target_Theta = getTargetAngle(obj)
            target_Theta = obj.target_Theta;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function startSimulation(obj)
            
            % Check to see that the target and target angles are set to
            % something. If setTargetAngle have not been called, then the
            % default target angle is used (see constructor)
            obj.check3ColumnVector(obj.target);
            obj.check3ColumnVector(obj.target_Theta);
            
            % get starting joint angles
            current_joint_angles = obj.robot.getJointAngles();
            
            % get target joint angles using the inverse kinematic
            [target_joint_angles, joint_angle_solutions, flag_joint_angle_solutions] = ...
                 obj.robot.inverseKinematic(obj.target, obj.target_Theta, current_joint_angles);            
    
            dt = 1/10; % update world every 1/10 of a second = 10 fps 
            t = 0; % time parameter
             
            % This method is pretty naive and can be improved 
            % I am now dividing the angle difference in 100 and will 
            % update every angle with this difference for each iteration
            n = 100;  
            d_j = (target_joint_angles - current_joint_angles) / n;
            
            obj.draw(); % draw the world
            
            do_continue = true; 
            while (do_continue)
                t_loopstart=tic();  %Declaring time counter 
                
                % This method is pretty naive and can be improved 
                % I am now dividing the angle difference in 100 and will 
                % update every angle with this difference for each iteration
                current_joint_angles = current_joint_angles + d_j;
                
                % update position of robot arm using forward kinematics
                obj.robot.forwardKinematic(current_joint_angles, true);
                
                % update the draw plot
                obj.drawUpdate(); 
                
                % This method is pretty naive and can be improved
                % if the difference between the target and current joint
                % angles are small enough we have reached the target and
                % will change the while loop parameter to false
                if sum(abs(target_joint_angles - current_joint_angles)) < 1e-5
                    do_continue = false;
                end
                
                t = t + dt; % increment time counter
                title(sprintf('time: %.1f',t )) % update plot title
                elapsed_time=toc(t_loopstart); 

                % pause the program long enough in order to get the frame
                % rate indicated by dt
                pause( (dt - elapsed_time) ); 
                
            end
            
            fprintf('TARGET REACHED!\n')
            
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end % end of methods
    
    
    
end

