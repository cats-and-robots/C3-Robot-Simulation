classdef World < AbstractClassDraw
    %WORLD Class that represent the whole simulation world. It contains a
    %human and a robot, and it is from here that the logical interaction
    %between then takes place.
    
    properties (Access = protected)
        target
        figure_handle
        axes_handle
        human
        robot
        
        objects
        targets
    end
    properties (Access = public)
        
%         human_future
%         human_mean
%         
%         robot_prediction
%         
%         dt = 0.1 %Animation timestep (10 frames per second)
%         %dt = 0.5;
%         
%         all_l_star
%         all_s_hat
%         all_g
%         all_mix_gaussian_mean
%         all_mix_gaussian_Sigma
%         elapsed_time_array
    end % end of properties
    
    methods
        function obj = World(varargin)
            if nargin == 0
                obj.robot = C3RobotArm();
            
            %deep-copy of the objects
            elseif nargin == 1
                obj.robot = copy(varargin{1});
            
            elseif nargin == 2
                error('Too many input paramters to constructor!');
%                 obj.human = copy(varargin{1});
%                 obj.robot = copy(varargin{2});
%                 obj.robot_prediction = RobotArm(obj.robot.Origin(1),obj.robot.Origin(2));
%                 obj.robot_prediction.Controller_goal_position = obj.robot.Controller_goal_position;
%                 obj.robot_prediction.Controller_goal_speed = obj.robot.Controller_goal_speed;
% 
%                 obj.human_future = copy(varargin{1});
%                 obj.human_mean = copy(varargin{1});
            
            else
                error('Too many input paramters to constructor!');
%                 obj.human = Human;
%                 obj.robot = RobotArm;
%                 obj.robot_prediction = RobotArm(obj.robot.Origin(1),obj.robot.Origin(2));
%                 obj.robot_prediction.Controller_goal_position = obj.robot.Controller_goal_position;
%                 obj.robot_prediction.Controller_goal_speed = obj.robot.Controller_goal_speed;
%                 obj.robot_prediction = RobotArm;
%                 obj.human_future = copy(obj.human);
%                 obj.human_mean = copy(obj.human);
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
        function setTarget(obj, target)
            obj.check3ColumnVector(target);
            obj.target = target;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function target = getTarget(obj)
            target = obj.target;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         function start(obj)
%             %START Start and run the simulation
%             nr_of_future_prediction_steps = 10;
%             
%             [~,all_timesteps] = obj.human.getAllMeasuredPositionData();
%             nr_of_human_dimension = numel(obj.human.Center);
%             nr_of_tables = numel(obj.human.Data_tables);
%             nr_of_timesteps = numel(all_timesteps);
%             obj.elapsed_time_array = zeros(nr_of_timesteps ,1);
% 
% 
%             obj.all_g = zeros(nr_of_timesteps,nr_of_future_prediction_steps+1);
%             obj.all_mix_gaussian_mean = zeros(nr_of_human_dimension, nr_of_timesteps,nr_of_future_prediction_steps+1);
%             obj.all_mix_gaussian_Sigma = zeros(nr_of_human_dimension, nr_of_human_dimension, nr_of_timesteps,nr_of_future_prediction_steps+1);
%             obj.all_l_star = zeros(nr_of_tables ,nr_of_timesteps, nr_of_future_prediction_steps+1);
%             obj.all_s_hat = zeros(nr_of_human_dimension ,nr_of_tables ,nr_of_timesteps, nr_of_future_prediction_steps+1);           
% 
% 
%             t_bar = 1;
%             do_continue = true;
%             robot_prediction_has_reached_its_goal = false;
%             
%             obj.draw();
%             while (do_continue)
%                 t_loopstart=tic();  %Declaring time counter 
%                 [position, time] = obj.human.getCurrentMeasuredPositionData();
% 
%                 [obj.all_g(t_bar,:), obj.all_mix_gaussian_mean(:,t_bar,:), obj.all_mix_gaussian_Sigma(:,:,t_bar,:), ...
%                     obj.all_l_star(:,t_bar,:), obj.all_s_hat(:,:,t_bar,:)] = ...
%                     obj.human.prediction_function( position, time, nr_of_future_prediction_steps);
%                 
%                 human_update_was_possible = obj.human.update();
%                 
%                 [robot_has_reached_its_goal, ~, distance_error] = obj.robot.controllerUpdateOneTimestep();
%                 fprintf('Distance error: %f \n', distance_error);
%                 
%                 
%                 [~, ~, ~] = obj.robot_prediction.controllerUpdateOneTimestep(); % get to current position
%                 p_collision_complement = 1;
%                 
%                 
%                 for i = 1:nr_of_future_prediction_steps+1
%                     estimate_human_mean = obj.all_mix_gaussian_mean(:,t_bar,i)';
%                     estimate_human_Sigma = obj.all_mix_gaussian_Sigma(:,:,t_bar,i);
%                     
%                     pics_probability =  obj.robot_prediction.pics_function(estimate_human_mean, estimate_human_Sigma);
%                     p_collision_complement = p_collision_complement * (1 -  pics_probability);
%                     if (~robot_prediction_has_reached_its_goal)
%                         [robot_prediction_has_reached_its_goal, ~, ~] = obj.robot_prediction.controllerUpdateOneTimestep();
% 
%                     end
%                 end
%                 
%                 % Calculate probability of collison with human.
%                 p_collision = 1 - p_collision_complement;
%                 
%                 
%                 do_continue = human_update_was_possible || ~robot_has_reached_its_goal;
%                 
%                 
%                 obj.human_future.Center = obj.all_mix_gaussian_mean(:,t_bar,end)';
%                 obj.human_mean.Center = obj.all_mix_gaussian_mean(:,t_bar,1)';
%                 
%                 obj.drawUpdate();
%                
%                 if (~robot_prediction_has_reached_its_goal)
%                     obj.robot_prediction.Joint_angles = obj.robot.Joint_angles;
%                     obj.robot_prediction.Joint_angles_speed = obj.robot.Joint_angles_speed;
%                     obj.robot_prediction.Endpoint_speed = obj.robot.Endpoint_speed;
%                     obj.robot_prediction.Controller_err_sum = obj.robot.Controller_err_sum;
%                     obj.robot_prediction.updateRobotPartsPosition();
%                 else
%                     obj.robot_prediction.updateRobotPartsPosition();
%                     disp('Predction DONE!');
% 					disp('Hej');
%                 end
%                 
%                 title(sprintf('time: %.1f',t_bar/10))
%                 el_time=toc(t_loopstart); 
%                 
%                 obj.elapsed_time_array(t_bar) = el_time;
%                 t_bar = t_bar + 1;
%                 
%                 pause(obj.dt-el_time);
%             end
%         end % end of start
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end % end of methods
    
    
    
end

