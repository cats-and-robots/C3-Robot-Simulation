classdef Human < AbstractClassDraw  
    %HUMAN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        Center % current 2D position of the human.
        %{
        time = []; % 1xN vector with all timesteps
        Measured_position = []; % 2xN contains all measured position
        True_position = []; % 2xN contains true 2D position (for validating data)
        %}
        nr_of_human_dimension
        nr_of_tables
        current_time_index
        Measured_position_table = struct([]); % 2xN contains all measured position
        Data_tables = struct([]); % 1xTables stuct. Each struct contains 'time' (1xN) and 'position' (2xN)
          
        deltaA = 2;
        deltaB = 1;
        prev_l_star %nr_of_tables x 1
        %prev_l_star_for_future_prediction %nr_of_tables x future_prediction
        table_time_stepsize
        
        Sigma % covariance for prediction funtion
        %std = 0.5; % standard deviation value used to set the covariance (OBS! variance = std^2)
        std = sqrt(0.5);
    end
    
    methods     
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Human(varargin)
            if nargin == 2
                obj.Center = [varargin{1} ; varargin{2}];
            else
                obj.Center = [ 0 ; 0 ];
            end
            obj.nr_of_human_dimension = numel(obj.Center);
            obj.Sigma = eye(numel(obj.Center))*obj.std^2; 
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setDataTables(obj, tables)
            % Set the tables that contain information of human position at different timesteps 
            obj.Data_tables = tables;
            obj.nr_of_tables = numel(obj.Data_tables);
            % allocate the size of prev_l_star that is used by l_star_function
            obj.prev_l_star = zeros(numel(tables),1);
            % set the time stepsize for the l_star_function. Assumes that
            % the first table time stepsize is the same for all tables.
            obj.table_time_stepsize = (tables(1).time(end)-tables(1).time(1))/(numel(tables(1).time)-1);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setMeasuredPositionDataTable(obj, measured_table_position)
            obj.Measured_position_table = measured_table_position;
            obj.current_time_index = 1;
            obj.Center = obj.Measured_position_table.position(:,1);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [position, time] = getCurrentMeasuredPositionData(obj)
            %getCurrentMeasuredPositionData Returns [ position(nx1), time ]
            if (isempty(obj.Measured_position_table))
                error('Error in getCurrentMeasuredPositionData. \nPlease set data table for measured human position (type %s).',class(obj.Data_tables));
            end
            position = obj.Center;
            time = obj.Measured_position_table.time(obj.current_time_index);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [all_positions, all_timesteps] = getAllMeasuredPositionData(obj)
            %getAllMeasuredPositionData Returns [ position(nxN), time(1xN) ]
            if (isempty(obj.Measured_position_table))
                error('Error in getAllMeasuredPositionData. \nPlease set data table for measured human position (type %s).',class(obj.Data_tables));
            end
            all_positions = obj.Measured_position_table.position;
            all_timesteps = obj.Measured_position_table.time;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function is_updated = update(obj)
            %UPDATE Update the position and increase the time. Everytime  
            %this method is called,the position and time is increased with
            %one time unit.It returns true if it could increase to the next 
            %step, else it returns false.
            %To get the position and time, call getCurrentMeasuredPositionData
            if (isempty(obj.Measured_position_table))
                error('Error in getCurrentMeasuredPositionData. \nPlease set data table for measured human position (type %s).',class(obj.Measured_position_table));
            end
            % if we have reached the end of our measured data points
            if (obj.current_time_index >= numel(obj.Measured_position_table.time))
                is_updated = false;
            else
                obj.current_time_index = obj.current_time_index + 1;
                obj.Center = obj.Measured_position_table.position(:,obj.current_time_index);
                is_updated = true;
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function draw(obj)
            obj.draw_handle = plot(obj.Center(1), obj.Center(2),'c.', 'MarkerSize',20');
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function drawUpdate(obj)
            set(obj.draw_handle,'XData',obj.Center(1),'YData',obj.Center(2))
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [g,mix_gaussian_mean, mix_gaussian_Sigma, l_star, s_hat] = prediction_function(obj, measured_position, t, nr_of_future_prediction_steps )

            if (isempty(obj.Data_tables))
                error('Error in prediction_function. \nPlease set data tables (type %s).',class(obj.Data_tables));
            end  
            
            if ~exist('nr_of_future_prediction_steps','var') || nr_of_future_prediction_steps == 0
                % default parameter. How many steps into the future we 
                % would like to predict
                %nr_of_future_prediction_steps = 0;
                % get current estimate of the position
                [g,mix_gaussian_mean, mix_gaussian_Sigma, l_star, s_hat] = obj.estimation_of_current_position(measured_position, t);
            else
                %obj.prev_l_star_for_future_prediction = zeros(obj.nr_of_tables,nr_of_future_prediction_steps+1);

                g = zeros(1,nr_of_future_prediction_steps+1);
                mix_gaussian_mean = zeros(obj.nr_of_human_dimension, nr_of_future_prediction_steps+1);
                mix_gaussian_Sigma = zeros(obj.nr_of_human_dimension, obj.nr_of_human_dimension, nr_of_future_prediction_steps+1);
                l_star = zeros(obj.nr_of_tables , nr_of_future_prediction_steps+1);
                s_hat = zeros(obj.nr_of_human_dimension, obj.nr_of_tables, nr_of_future_prediction_steps+1); 
                
                %prev_l_star_copy = obj.prev_l_star; % save a copy of the previous best time values from each table  
                [g(:,1),mix_gaussian_mean(:,1), mix_gaussian_Sigma(:,:,1), l_star(:,1), s_hat(:,:,1)] = ...
                    obj.estimation_of_current_position(measured_position, t);
                
                % get future predictions of the estimated positions
                for i = 1:nr_of_future_prediction_steps
                    measured_position = mix_gaussian_mean(:,i); 
                    [g(:,i+1),mix_gaussian_mean(:,i+1), mix_gaussian_Sigma(:,:,i+1), l_star(:,i+1), s_hat(:,:,i+1)] = ...
                        obj.estimation_of_current_position(measured_position, t+i*obj.table_time_stepsize);
                end
                obj.prev_l_star = l_star(:,1); % retrieve the copy of previous best time values from each table  
            end

        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [g,mix_gaussian_mean, mix_gaussian_Sigma, l_star, s_hat] = estimation_of_current_position(obj, measured_position, t)

            if (isempty(obj.Data_tables))
                error('Error in prediction_function. \nPlease set data tables (type %s).',class(obj.Data_tables));
            end
            
            w_bar = zeros(numel(obj.Data_tables), 1); 
            s_hat = zeros(numel(measured_position), numel(obj.Data_tables));

            % calculate l_star for all tables for 1 timestep and 1 measured position
            l_star  = obj.l_star_function(measured_position, t);
            
            for i = 1:numel(obj.Data_tables)
                % compute state prediction function
                index_position = find(obj.Data_tables(i).time == l_star(i), 1, 'first');
                s_hat(:,i) = obj.Data_tables(i).position(:,index_position);

                % calculate the weights for 1 timestep and 1 measured position
                distance = obj.distance_function(measured_position,s_hat(:,i));
                if (distance < 1e-7) % set distance to a small value to avoid divide by zero error
                    distance = 1e-7;
                end
                w_bar(i) = 1 / distance; 
            end
            
            % normalize the weights so that the sum of each tables weight is 1
            w = w_bar ./ sum(w_bar);
            
            % calculate the probability g of how similar the measured
            % position is to the one predicted from the data tables
            % The sum of weighted gaussian distributions will have a
            % weighted (w) mean = w(i)*mean(i) and weighted variance =
            % w(i)^2*variance(i) 
            g = 0;
            mix_gaussian_mean = zeros(numel(measured_position),1);
            %mix_gaussian_2nd_moment = zeros(numel(measured_position),numel(measured_position));
            %mix_gaussian_Sigma_2 = zeros(numel(measured_position),numel(measured_position));
            mix_gaussian_Sigma = zeros(size(obj.Sigma));
            for i = 1:numel(obj.Data_tables)
                % set the current table's closest position (s_hat) to the
                % mean of the gaussian function
                mu = s_hat(:,i); 
                g = g + w(i)*mvnpdf(measured_position,mu, obj.Sigma);
                mix_gaussian_mean = mix_gaussian_mean + w(i)*mu;
                %mix_gaussian_2nd_moment = mix_gaussian_2nd_moment +
                %w(i)*(diag(mu).^2 + obj.Sigma.^2); 
                %mix_gaussian_2nd_moment = mix_gaussian_2nd_moment +  w(i)*(diag(mu).^2 + obj.Sigma);

                mix_gaussian_Sigma = mix_gaussian_Sigma + w(i)^2*obj.Sigma;
            end

            %mix_gaussian_Sigma = mix_gaussian_2nd_moment - diag(mix_gaussian_mean).^2

            % increase the covariance (obj.Sigma)
            obj.Sigma_increase();
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end % end of public methods
    
    methods (Access = private)
        function Sigma_increase(obj)
            %SIGMA_INCREASE Increase covariance Sigma after each prediction
            %calculation
            
            %linear increase of Sigma with squared std_h for each time prediction_function is called
            std_h = 0; 
            obj.Sigma = obj.Sigma + eye(numel(obj.Center))*std_h^2;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function d = distance_function(obj,  y, s )
            %DISTANT_FUNCTION Distant between measurement and table data entry
            %   Detailed explanation goes here
            m = y - s;
            if isvector(m)
                d = norm(m);
            else 
                % every column is a vector, return the norm of each column
                d=sqrt(sum(m.^2,1));
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function l_star  = l_star_function(obj, measured_position, t)
        %UNTITLED3 Summary of this function goes here
        %   Produces l_star for one timestep for all the tables

            l_star = zeros(numel(obj.Data_tables),1);

            tA_start = t - obj.deltaA*obj.table_time_stepsize;
            tA_end = t + obj.deltaA*obj.table_time_stepsize;
            tA = tA_start:obj.table_time_stepsize:tA_end;
            % there are some extra numerical numbers left in all members in tA 
            % tB. These have to be removed in order for union to see them as
            % the same member. To remove them, convert all numbers to strings
            % and then convert them back to numbers.
            tA = str2num(num2str(tA));

            % create tB
            tB_start_vector = obj.prev_l_star - obj.deltaB*obj.table_time_stepsize;
            tB_end_vector = obj.prev_l_star + obj.deltaB*obj.table_time_stepsize;

            tB_matrix = zeros(numel(obj.Data_tables), numel(tB_start_vector(1):obj.table_time_stepsize:tB_end_vector(1)));
            for i = 1:numel(obj.Data_tables)
                tB_matrix(i,:) = tB_start_vector(i):obj.table_time_stepsize:tB_end_vector(i);
            end
            tB_matrix = str2num(num2str(tB_matrix));


            for i = 1:numel(obj.Data_tables)
                tB = tB_matrix(i,:);
                intersect_elements = intersect(tA, tB);
                % create the l-interval using tA and tB 
                % if they overlap, the l-interval is the union

                if ~isempty(intersect_elements)
                    % the intervals tA and tB are intersecting
                    l_interval = union(tA, tB);
                    
                    [ index_start, index_end ] = obj.findValidTimeIntervalIndices(l_interval, i);
                    [~,l_star_value] = obj.findNeareastDataTablePositionWithinTimeInterval(measured_position, i, index_start, index_end);

                    l_star(i) = l_star_value;
                else
                    % check each interval tA and tB and compare the best match inside
                    % each of the two time intervals
                    [ index_start_A, index_end_A ] = obj.findValidTimeIntervalIndices(tA, i);
                    [min_distance_A,l_star_value_A] = obj.findNeareastDataTablePositionWithinTimeInterval(measured_position, i, index_start_A, index_end_A);
                    [ index_start_B, index_end_B ] = obj.findValidTimeIntervalIndices(tB, i);
                    [min_distance_B,l_star_value_B] = obj.findNeareastDataTablePositionWithinTimeInterval(measured_position, i, index_start_B, index_end_B);

                    if min_distance_A <= min_distance_B
                        l_star(i) = l_star_value_A;
                    else
                        l_star(i) = l_star_value_B;
                    end
                end    
            end
            
            % remember l_star til next time
            obj.prev_l_star = l_star;

        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [ index_start, index_end ] = findValidTimeIntervalIndices(obj, l_interval, table_index)
            t_full = obj.Data_tables(table_index).time;
            index_start = []; index_end = [];
            for index = 1:numel(l_interval)
                if isempty(index_start)
                    index_start = find(t_full == l_interval(index),1,'first');
                else
                    break;
                end
            end
            for index = numel(l_interval):-1:1
                if isempty(index_end)
                    index_end = find(t_full == l_interval(index),1,'last');
                else
                    break;
                end
            end
            
            
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [min_distance,l_star_value] = findNeareastDataTablePositionWithinTimeInterval(obj, measured_position, table_index, index_start, index_end)

            s = obj.Data_tables(table_index).position(:,index_start:index_end);
            s_time = obj.Data_tables(table_index).time(index_start:index_end);
            y_multiple = repmat(measured_position,[1,numel(index_start:index_end)]);
            d = obj.distance_function(y_multiple, s);
            [min_distance,I] = min(d);
            l_star_value = s_time(I);

        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end % end of private methods
    
end

