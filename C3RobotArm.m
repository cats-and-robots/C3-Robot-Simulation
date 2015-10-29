classdef C3RobotArm < AbstractClassDraw
    %ROBOTARM Model of a 4 link planar robot arm in 2D. 
    %   Is constructed of 4 Link and 5 Joint objects.
    %   It inherits from abstract class AbstractClassDraw.
    %
    %   properties (public, constant, hidden)
    %       Left: 0
    %       Right: 1
    %
    %   properties (public)
    %       type: either C3RobotArm.Left or C3RobotArm.Right
    %       joint_angles: vector of all 6 joint angles [rad] for the robot
    %       parts: storage for all robot's parts
    %
    %   properties (private, constant)
    %       [link lenght constants [meter] (based on DH parameters)]
    %       a1 = 0.10, a2 = 0.25, d1 = 0.32, d4 = 0.25, d6 = 0.065
    %       
    %       [vectors storing joint angle max velocity [rad/sec]]
    %       joint_angles_max_speed 
    % 
    %       [vectors storing joint angle limitations [rad]]
    %       joint_angles_min_angle, joint_angles_max_angle
    %
    %
    %   methods (public)
    %       obj = C3RobotArm(varargin): constructor
    %       draw(): plot the robot arm in the plot window 
    %       drawUpdate(): update the robot arm's drawing animation
    %       setJointAngles(joint_angles): takes a 6x1 vector [rad]
    %       joint_angles = getJointAngles(): returns a 6x1 vector [rad]
    %
    %       [P, Theta] = forwardKinematic(obj, joint_angles,
    %       flag_update_robot=false): 
    %           P: 3x5 matrix, each column is the joint position [mm]
    %           Theta: 3x1 vector, end point rotation in x,y,z [rad]
    %           joint_angles: target joint angles [rad]
    %           flag_update_robot: if true, updates robot arm position
    %
    %       [picked_joint_angle_solution, joint_angle_solutions,
    %       flag_joint_angle_solutions] = inverseKinematic(obj,
    %       target_position, target_Theta, current_joint_angles,
    %       flag_update_robot=false):
    %           picked_joint_angle_solution: 6x1 vector, best solution for
    %           the joint angles. [rad]
    %           joint_angle_solutions: 6x8 matrix, all found solutions[rad]
    %           flag_joint_angle_solutions: 1x8 vector. Indicates if
    %           solution is valid.
    %           target_position: 3x1 vector [mm]
    %           target_Theta: 3x1 vector, target rotation in x,y,z [rad]
    %           current_joint_angles: 6x1 vector [rad]
    %           flag_update_robot: if true, updates robot arm position

    
    % This class is copyable and can thus not have enumerators, so these
    % two class attributes will work as enumerators for indicating if it is
    % a left or right robot arm. The same value notation is used with the
    % real C3 robot arms. The left arm (0) is the arm that would be your
    % left real arm from the robot's perspective when the arms is in their
    % initial position (all angles being set to 0)
    % Right is set as the default robot type.
    properties (Access = public, Constant = true, Hidden = true )
        Left = 0;
        Right = 1; 
    end
    
    properties (Access = public)
        type; % C3RobotArm.Left or C3RobotArm.Right
        joint_angles; % vector of all 6 joint angles [rad] for the robot
        parts; % storage for all robot's parts. First half are the links
    end
    
    properties (Access = private, Constant = true)
        % link lenght constants [meter] (based on DH parameters)
        a1 = 0.10;
        a2 = 0.25;
        d1 = 0.32;
        d4 = 0.25;
        d6 = 0.065;
        
        % limitations set for the robot arm spedd [rad/sec]
        joint_angles_max_speed = ...
            [ ...
            deg2rad(225); deg2rad(225);...
            deg2rad(257); deg2rad(279); ...
            deg2rad(275); deg2rad(360) ...
            ];
        % limitations of allowed joint angles [rad]
        joint_angles_min_angle = ...
            [ ...
            deg2rad(-170); deg2rad(-160); ...
            deg2rad(-51); deg2rad(-200); ...
            deg2rad(-135); deg2rad(-360) ...
            ];
		joint_angles_max_angle = ...
            [ ...
            deg2rad(170); deg2rad(65); ...
            deg2rad(225); deg2rad(200); ...
            deg2rad(135); deg2rad(360) ...
            ];
    end
    
    methods (Access = public)
        % Constructor      
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = C3RobotArm(varargin)
            if nargin == 0
                % set right as the default robot type
                obj.type = C3RobotArm.Right;
                
            elseif nargin == 1 % robot type is given
                if ~isscalar(varargin{1})
                    error(sprintf(['Input robot type must be a scalar!\n'...
                    'Input robot type is either C3RobotArm.Left (0) or C3RobotArm.Right (1)']));
                end
                if varargin{1} == C3RobotArm.Left || ...
                        varargin{1} == C3RobotArm.Right
                    obj.type = varargin{1};
                else
                    error(sprintf(['Input robot type is wrong!\n'...
                    'Input robot type is either C3RobotArm.Left (0) or C3RobotArm.Right (1)']));
                end    
                
            else
                error(sprintf(['Too many input parameters!'...
                '\nC3RobotArm constructor takes 0 or 1 input parameters.' ...
                '\n\n0 param: C3 robot is set to a right arm per default.'...
                '\n1 param: C3RobotArm.Left (0) or C3RobotArm.Right (1)']));
            end
            
            % create 5 joints and 4 links
            nrOfParts = 9;
            for i = 1:nrOfParts
                if mod(i,2) == 1
                    obj.parts = [obj.parts Joint([0;0;0])];
                else
                    obj.parts = [obj.parts Link([0;0;0],[0;0;0])];
                end
            end % end of for-loop
            
            % set the initial robot joint angles to all be 0
            init_joint_angles = [0; 0; 0; 0; 0; 0];
            obj.checkJointAngles(init_joint_angles); % I'm paranoid <_< 
            obj.setJointAngles(init_joint_angles);
            
            % use the forward kinematic to update the robot parts 
            obj.forwardKinematic(obj.getJointAngles(), true);
             
        end % end of constructor
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function draw( obj )
        %DRAW Draws the robot arm.
        %   To draw the robot arm, it goes through every object inside RobotParts 
        %   and calls every object's own draw-method. The objects are subclasses
        %   created from AbstractClassRobotPart and AbstractClassDraw and must have a draw method
        %   implemented.
        %   It colors the origin black, endpoint red and the rest of the
        %   robot blue.
        
            for i = 1:numel(obj.parts)
                obj.parts(i).draw();
            end

            set(obj.parts(1).draw_handle, 'MarkerFaceColor', 'black');
            set(obj.parts(end).draw_handle, 'MarkerFaceColor', 'red');
            
        end % end of draw
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function drawUpdate(obj)
            for i = 1:numel(obj.parts)
                obj.parts(i).drawUpdate();
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setJointAngles(obj, joint_angles)
            obj.checkJointAngles(joint_angles);
            obj.joint_angles = joint_angles;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function joint_angles = getJointAngles(obj)
            joint_angles = obj.joint_angles;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [P, Theta] = forwardKinematic(obj, joint_angles, flag_update_robot) %#ok<INUSD>
            
            obj.checkJointAngles(joint_angles);
            
            if ~exist('flag_update_robot','var')
                flag_update_robot = false; 
            else
                if ~isa(flag_update_robot,'logical') % in Matlab2015 and higher, you can use isboolean instead
                    error(sprintf(['Input flag update-robot is not a boolean!'...
                    '\nInput flag update-robot need to be either false or true']));
                end
            end
            
            % if flag_return_link_positions is true then we store and return
            % the link positions in this matrix.
            % Else, we directly updates the link positions for this object's
            % robot parts.
            
            % help parameters
            s1 = sin(joint_angles(1));
            c1 = cos(joint_angles(1));
            s2 = sin(joint_angles(2));
            c2 = cos(joint_angles(2));
            s4 = sin(joint_angles(4));
            c4 = cos(joint_angles(4));
            s5 = sin(joint_angles(5));
            c5 = cos(joint_angles(5));
            s6 = sin(joint_angles(6));
            c6 = cos(joint_angles(6));
            s23 = sin( joint_angles(2) + joint_angles(3) );
            c23 = cos( joint_angles(2) + joint_angles(3) );
            
            % help parameters used to calculate the end position in
            % kinematic coordinate space. The names and expression are
            % taken from forward kinematic formula used by the real time
            % applications used by iMOTION
            % p0x, p0y and p1z are used to find the position of the end
            % effector for a robot arm.
            % The other are used to calculate the orientation (n,t,b) of
            % the end effector
            p0x = c1*s23*c4 + s1*s4;
            p1x = s1*c4 - c1*s23*s4;
            p0y = s1*s23*c4 - c1*s4;
            p1y = s1*s23*s4 + c1*c4;
            p0z = s23*s5 - c23*c4*c5;
            p1z = c23*c4*s5 + s23*c5;
            
            % storing robots positions in kinematics workin space
            p = zeros(3,5);
            % Joint 1 position is in origo
            % Joint 2 position
            p(1,2) = obj.a1*c1;
            p(2,2) = obj.a1*s1;
            p(3,2) = obj.d1;

            % Joint 3 position
            p(1,3) = (obj.a1 - obj.a2*s2)*c1;
            p(2,3) = (obj.a1 - obj.a2*s2)*s1;
            p(3,3) = obj.a2*c2 + obj.d1;

            % Joint 4 is just rotation
            % Joint 5 position
            p(1,4) = (obj.a1 - obj.a2*s2)*c1 - (-obj.d4)*c1*c23;
            p(2,4) = (obj.a1 - obj.a2*s2)*s1 - (-obj.d4)*s1*c23;
            p(3,4) = obj.a2*c2 + obj.d1 - (-obj.d4)*s23;

            % Joint 6, end-effector position
            p(1,5) = (p0x*s5 - c1*c23*c5)*(-obj.d6) + (obj.a1 - obj.a2*s2)*c1 - (-obj.d4)*c1*c23;
            p(2,5) = (p0y*s5 - s1*c23*c4)*(-obj.d6) + (obj.a1 - obj.a2*s2)*s1 - (-obj.d4)*s1*c23;
            p(3,5) = -p1z*(-obj.d6) + obj.a2*c2 + obj.d1 - (-obj.d4)*s23;
            
            % calculate end effector orientation vectors n,t,b
            n = zeros(3,1);
            t = zeros(3,1);
            b = zeros(3,1);
            n(1) = (p0x*c5 + c1*c23*s5)*c6 + p1x*s6;
            t(1) = -(p0x*c5 + c1*c23*s5)*s6 + p1x*c6;
            b(1) = p0x*s5 - c1*c23*c5;
            
            n(2) = (p0y*c5 + s1*c23*s5)*c6 - p1y*s6;
            t(2) = -(p0y*c5 + s1*c23*s5)*s6 - p1y*c6;
            b(2) = p0y*s5 - s1*c23*c5;
            
            n(3) = p0z*c6 + c23*s4*s6;
            t(3) = -p0z*s6 + c23*s4*c6;
            b(3) = -p1z;
            
            if obj.type == C3RobotArm.Left
                [P, Theta] = obj.transformKinematicCoordinate2LeftArmCoordinate(n,t,b,p);
            elseif obj.type == C3RobotArm.Right
                %disp('Right arm kinematics is not yet completed');
                [P, Theta] = obj.transformKinematicCoordinate2RightArmCoordinate(n,t,b,p);
            else
                error(sprintf(['Robot arm is of unknown type!'...
                    '\nA robot arm is either of type Left or Right']));
            end
            
            if flag_update_robot
                % update position of joint angles and robot parts
                obj.setJointAngles(joint_angles);
                obj.updateRobotParts(P);
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [picked_joint_angle_solution, joint_angle_solutions, flag_joint_angle_solutions] = inverseKinematic(obj, target_position, target_Theta, current_joint_angles, flag_update_robot) %#ok<INUSD>
            obj.check3ColumnVector(target_position);
            obj.check3ColumnVector(target_Theta);
            obj.checkJointAngles(current_joint_angles);
            
            if ~exist('flag_update_robot','var')
                flag_update_robot = false; 
            else
                if ~isa(flag_update_robot,'logical') % in Matlab2015 and higher, you can use isboolean instead
                    error(sprintf(['Input flag update-robot is not a boolean!'...
                    '\nInput flag update-robot need to be either false or true']));
                end
            end
            
            
            % convert the target position and orientaion in robot workspace
            % coordinates into the kinematic workspace coordinates
            if obj.type == C3RobotArm.Left
                [n,t,b,p] = obj.transformLeftArmCoordinate2KinematicCoordinate(target_position, target_Theta);
            elseif obj.type == C3RobotArm.Right
                [n,t,b,p] = obj.transformRightArmCoordinate2KinematicCoordinate(target_position, target_Theta);
            else
                error(sprintf(['Robot arm is of unknown type!'...
                    '\nA robot arm is either of type Left or Right']));
            end

            % there will exist multiple solutions for the inverse
            % kinematics. We will store 8 of them in a 6x8 matrix
            % (in the old code, this matrix is called 'jangle'
            nrOfSolutions = 8;
            joint_angle_solutions = zeros(6,nrOfSolutions);
            
            
            % flag parameter to store if the n:th solution exist or not
            flag_joint_angle_solutions = ones(1,nrOfSolutions);
            
            ntb = [n t b];
            % use no extension [mm]
            extension = [0;0;0]; 
            
            p = p - ntb*extension;

% 			/***** J1‚ÌŠp“x‚ðŽZ?o *****/
            % Setting up joint 1
            
            w = obj.d6*b + p;

% for loop logic: the if statement is true for 0<=i<=3 and false for
% 4<=i<=7
            for i = 1:nrOfSolutions
                if i<=4 % 4 first solutions for the first angle
                    if (w(1) == 0 && w(2) == 0) 
                        joint_angle_solutions(1,i) = current_joint_angles(1);
                    else
                        joint_angle_solutions(1,i) = atan2(w(2), w(1));
                    end
                else % remaining solutions for the first angle
                    if (w(1) == 0 && w(2) == 0)
                        joint_angle_solutions(1,i) = current_joint_angles(1) + pi;
                    else
                        joint_angle_solutions(1,i) = atan2(w(2), w(1)) + pi;
                    end
                        
                end
            end
            
            % Modify solutions so that joint 1 is between [-pi, pi]
            for i = 1:nrOfSolutions
                if joint_angle_solutions(1,i) < -pi 
                    joint_angle_solutions(1,i) = 2*pi + joint_angle_solutions(1,i);
                elseif joint_angle_solutions(1,i) > pi 
                    joint_angle_solutions(1,i) = joint_angle_solutions(1,i) - 2*pi;
                end
            end

% 			/***** J2‹y‚ÑJ3‚ÌŠp“x‚ðŽZ?o *****/
            % Setting up joint 2 to joint 3
            % FIX THIS AWFUL LOOP WHEN WE KNOW IT WORKS!
            for i = 1:(nrOfSolutions/2):nrOfSolutions
                % use solution angles instead of current angles
                s1 = sin(joint_angle_solutions(1,i));
				c1 = cos(joint_angle_solutions(1,i));
                % rotate along z-axis, adjust length in x and z direction
                wp = [ ... 
                    w(1)*c1 + w(2)*s1 - obj.a1; ...
                    -w(1)*s1 + w(2)*c1 ; ...
                    w(3) - obj.d1 ...
                    ];
                % length of vector projected on the x and z plane
                p1 = sqrt(wp(1)*wp(1) + wp(3)*wp(3));
                reach_limit = 2*obj.a2;
                
                if (p1 > reach_limit) % link cannot reach, solution do not exist
                    joint_angle_solutions(2,i) = nan;
					joint_angle_solutions(3,i) = nan;

					joint_angle_solutions(2, i + 1) = nan;
					joint_angle_solutions(3, i + 1) = nan;

					joint_angle_solutions(2, i + 2) = nan;
					joint_angle_solutions(3, i + 2) = nan;

					joint_angle_solutions(2, i + 3) = nan;
					joint_angle_solutions(3, i + 3) = nan;
                    
                    flag_joint_angle_solutions(i) = 0;
                    flag_joint_angle_solutions(i+1) = 0;
                    flag_joint_angle_solutions(i+2) = 0;
                    flag_joint_angle_solutions(i+3) = 0;
                    
                else                    
                    p2 = atan2(wp(3), wp(1));
					p3 = acos( (p1 / 2) / obj.a2);

                    joint_angle_solutions(2,i) = p2 + p3 - pi / 2;
					joint_angle_solutions(3,i) = pi / 2 - 2 * p3;
					
                    % Modify solutions so that joint 2 is between [-pi, pi]
                    if joint_angle_solutions(2,i) < -pi
                        joint_angle_solutions(2,i) = 2 * pi + joint_angle_solutions(2,i);
                    elseif joint_angle_solutions(2,i) > pi
                        joint_angle_solutions(2,i) = joint_angle_solutions(2,i) - 2 * pi;
                    end
   
					joint_angle_solutions(2, i + 1) = joint_angle_solutions(2,i);
					joint_angle_solutions(3, i + 1) = joint_angle_solutions(3,i);

					joint_angle_solutions(2, i + 2) = p2 - p3 - pi / 2;
					joint_angle_solutions(3, i + 2) = pi / 2 + 2 * p3;
                    
                    % Modify solutions so that joint 2 is between [-pi, pi]
                    if joint_angle_solutions(2, i + 2) < -pi
                        joint_angle_solutions(2, i + 2) = 2 * pi + joint_angle_solutions(2, i + 2);
                    elseif joint_angle_solutions(2, i + 2) > pi
                        joint_angle_solutions(2, i + 2) = joint_angle_solutions(2, i + 2) - 2 * pi;
                    end
                    
					joint_angle_solutions(2, i + 3) = joint_angle_solutions(2, i + 2);
                    joint_angle_solutions(3, i + 3) = joint_angle_solutions(3, i + 2);
                end
            end

            %/***** J4‹y‚ÑJ5‚ÌŠp“x‚ðŽZ?o *****/
            % Setting up joint 4 to joint 5
            % loop over every 2nd joint solution
            for i = 1:2:nrOfSolutions
                % use solution angles instead of current angles
                s1 = sin(joint_angle_solutions(1,i));
				c1 = cos(joint_angle_solutions(1,i));

				s2 = sin(joint_angle_solutions(2,i));
				c2 = cos(joint_angle_solutions(2,i));

				s3 = sin(joint_angle_solutions(3,i));
				c3 = cos(joint_angle_solutions(3,i));

                % point of the end effector (I think)
                pp = [ ...
                    c3*((p(1)*c1 + p(2)*s1 - obj.a1)*c2 + (p(3) - obj.d1)*s2) ...
					+ s3*(-(p(1)*c1 + p(2)*s1 - obj.a1)*s2 + (p(3) - obj.d1)*c2 - obj.a2) - obj.d4;...
                    
                    -p(1)*s1 + p(2)*c1;...
                    
                     -s3*((p(1)*c1 + p(2)*s1 - obj.a1)*c2 + (p(3) - obj.d1)*s2) ...
					+ c3*(-(p(1)*c1 + p(2)*s1 - obj.a1)*s2 + (p(3) - obj.d1)*c2 - obj.a2);...
                    ];
                
                joint_angle_solutions(5,i) = acos(pp(1) / norm(pp));
                if joint_angle_solutions(5,i) == 0
                    joint_angle_solutions(4,i) = current_joint_angles(4);
                else
                    joint_angle_solutions(4,i) = atan2( pp(2), pp(3) );
                end
                
                %//’è‹`ˆæ‚ð-ƒÎ?`ƒÎ‚É•ÏŠ
                % Modify solutions so that joint 4 is between [-pi, pi]
                if joint_angle_solutions(4,i) < -pi
                    joint_angle_solutions(4,i) = 2 * pi + joint_angle_solutions(4,i);
                elseif joint_angle_solutions(4,i) > pi
                    joint_angle_solutions(4,i) = joint_angle_solutions(4,i) - 2 * pi;
                end

                joint_angle_solutions(5, i + 1) = -joint_angle_solutions(5, i);
                if joint_angle_solutions(5, i + 1) == 0
                    joint_angle_solutions(4, i + 1) = current_joint_angles(4);
                else
                    joint_angle_solutions(4, i + 1) = joint_angle_solutions(4,i) - pi;
                end

                %//’è‹`ˆæ‚ð-ƒÎ?`ƒÎ‚É•ÏŠ
                % Modify solutions so that joint 4 is between [-pi, pi]
                if joint_angle_solutions(4, i + 1) < -pi
                    joint_angle_solutions(4, i + 1) = 2 * pi + joint_angle_solutions(4, i + 1);
                elseif joint_angle_solutions(4, i + 1) > pi
                    joint_angle_solutions(4, i + 1) = joint_angle_solutions(4, i + 1) - 2 * pi;
                end
                
            end

            %/***** J6‚ÌŠp“x‚ðŽZ?o *****/
            % Setting up joint 6 
            % loop over every 2nd joint solution
            for i = 1:2:nrOfSolutions
                % use solution angles instead of current angles
				s1 = sin( joint_angle_solutions(1,i) );
				c1 = cos( joint_angle_solutions(1,i) );

				s23 = sin( joint_angle_solutions(2,i) + joint_angle_solutions(3,i) );
				c23 = cos( joint_angle_solutions(2,i) + joint_angle_solutions(3,i) );

				s4 = sin( joint_angle_solutions(4,i) );
				c4 = cos( joint_angle_solutions(4,i) );

				s5 = sin( joint_angle_solutions(5, i) );
				c5 = cos( joint_angle_solutions(5, i) );

                wpp = [ ...
                    (c1*s23*c4 + s1*s4)*c5 + c1*c23*s5;...
                    (s1*s23*c4 - c1*s4)*c5 + s1*c23*s5;...
                    s23*s5 - c23*c4*c5;...
                    ];

                % dot product between wpp and n vector
                %p0 = wpp(1)*n(1) + wpp(2)*n(2) + wpp(3)*n(3);
                p0 = wpp'*n; 
                if p0 > 1
                    p0 = 1;
                elseif p0 < -1
                    p0 = -1;
                end
				
                % cross-product between wpp and n
%                 wp = [ ...
%                     wpp(2)*n(3) - wpp(3)*n(2);...
%                     wpp(3)*n(1) - wpp(1)*n(3);...
%                     wpp(1)*n(2) - wpp(2)*n(1);...
%                     ];
                wp = cross(wpp, n);
                % dot product between wp and b vector
                %p1 = wp(1)bn(1) + wp(2)*b(2) + wp(3)*b(3);
                p1 = wp'*b; 
                if p1 > 0
                    joint_angle_solutions(6,i) = acos(p0);
                else
                    joint_angle_solutions(6,i) = -acos(p0);
                end
                
                joint_angle_solutions(6, i + 1) = joint_angle_solutions(6, i) - pi;
                
                %//’è‹`ˆæ‚ð-ƒÎ?`ƒÎ‚É•ÏŠ·
                % Modify solutions so that joint 4 is between [-pi, pi]
                if joint_angle_solutions(6, i + 1) < -pi
                    joint_angle_solutions(6, i + 1) = 2 * pi + joint_angle_solutions(6, i + 1);
                elseif joint_angle_solutions(6, i + 1) > pi
                    joint_angle_solutions(6, i + 1) = joint_angle_solutions(6, i + 1) - 2 * pi;
                end
            end

            % If the solution's joint angles are not within valid angles,
            % mark that solution as invalid
            for i = 1:nrOfSolutions
                if ~( prod( obj.joint_angles_min_angle < joint_angle_solutions(:,i) )&& ...
                        prod( joint_angle_solutions(:,i) < obj.joint_angles_max_angle ) )
                    flag_joint_angle_solutions(i) = 0;
                end
            end

            % use the found solutions angles that is closest to the current
            % joint angles
            solution_indices = find(flag_joint_angle_solutions == 1);
            if isempty(solution_indices)
                error('No inverse kinematic solution exist for the given target!');
            end
            
            % from the found solution angles, choose the one whose angles
            % are closest to the current joint angles
            picked_joint_angle_solution = zeros(6,1);
            smallest_diff = Inf;
            for i=solution_indices
                diff = sum(abs(current_joint_angles - joint_angle_solutions(:,i) ));
                if diff < smallest_diff
                    smallest_diff = diff;
                    picked_joint_angle_solution = joint_angle_solutions(:,i); 
                end
            end

            % update position of joint angles and robot parts
            if flag_update_robot
                forwardKinematic(obj, picked_joint_angle_solution, flag_update_robot);
            end
        end 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end % end of public methods
    
    methods (Access = private)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function rot_mat = calculateRotationMatrix(obj,rot_vec)
            %CALCULATEROTATIONMATRIX Creates a 3x3 rotation matrix from a
            %3x1 vector. This code was taken from the iMOTION project where
            %this function is used to calculate the n,t,b vectors for the
            %end-point rotation.
            %The input 3x1 vector (Theta in iMOTION model) shall be given in radiance
            obj.check3ColumnVector(rot_vec);
            
            s = sin(rot_vec);
            c = cos(rot_vec);
            rot_mat = [ ...
                c(2)*c(3),                  -c(2)*s(3),                 s(2); ...
                s(1)*s(2)*c(3)+c(1)*s(3),   -s(1)*s(2)*s(3)+c(1)*c(3),  -s(1)*c(2);...
                -c(1)*s(2)*c(3)+s(1)*s(3),  c(1)*s(2)*s(3)+s(1)*c(3),   c(1)*c(2); ...
                ];
            
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [n,t,b,p] = transformLeftArmCoordinate2KinematicCoordinate(obj,P,Theta)
            % Remember to give Theta in radiance
            obj.checkPositionMatrix(P);
            obj.check3ColumnVector(Theta);

            % change kinematic working space to right arm working space
            % set the base of the rotation matrix
            m0 = [ ...
                 0,  1,  0; ...
                -1,  0,  0; ...
                 0,  0, -1  ...
                ];
            
            %rotate, and scale down the end point (from millimeter to meter)
            p = m0*P ./ 1000;             
            % transform the rotation vector to a matrix
            m1 = obj.calculateRotationMatrix(Theta);
            
            m1_temp = m0*m1;
            m0_temp = [ ...
                 0,  1,  0; ...
                -1,  0,  0; ...
                 0,  0, -1  ...
                ];
            
            ntb = m1_temp*m0_temp;
            n = ntb(:,1);
            t = ntb(:,2);
            b = ntb(:,3); 	
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [n,t,b,p] = transformRightArmCoordinate2KinematicCoordinate(obj,P,Theta)
            % Remember to give Theta in radiance
            obj.checkPositionMatrix(P);
            obj.check3ColumnVector(Theta);
            
            % change kinematic working space to right arm working space
            % set the base of the rotation matrix
            m0 = [ ...
                0,  1,  0; ...
                1,  0,  0; ...
                0,  0, -1  ...
                ];
            
            %rotate, and scale down the end point (from millimeter to meter)
            p = m0*P ./ 1000;            
            % transform the rotation vector to a matrix
            m1 = obj.calculateRotationMatrix(Theta);
            
            m1_temp = m0*m1;
            m0_temp = [ ...
                 0, -1,  0; ...
                -1,  0,  0; ...
                 0,  0, -1  ...
                ];
            
            ntb = m1_temp*m0_temp;
            n = ntb(:,1);
            t = ntb(:,2);
            b = ntb(:,3);      
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [P,Theta] = transformKinematicCoordinate2LeftArmCoordinate(obj,n,t,b,p)
            obj.check3ColumnVector(n);
            obj.check3ColumnVector(t);
            obj.check3ColumnVector(b);
            obj.checkPositionMatrix(p);
            
            m0 = [ ...
                0,  -1, 0; ...
                1,  0,  0; ...
                0,  0, -1  ...
                ];
            m1 = [n t b];
            
            %rotate every column vector in p, and scale everything up with 1000 (from meter to millimeter)
            P = m0*p * 1000;
            
            m1_temp = m0*m1;
            
            m0_temp = [ ...
                 0,  -1,  0; ...
                 1,   0,  0; ...
                 0,   0, -1  ...
                ];
            Theta_mat = m1_temp*m0_temp;
            Theta = [...
                atan2( -Theta_mat(2,3), Theta_mat(3,3)); ...
                asin( Theta_mat(1,3) ); ...
                atan2(-Theta_mat(1,2), Theta_mat(1,1)) ...
                ]; 
            
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [P,Theta] = transformKinematicCoordinate2RightArmCoordinate(obj,n,t,b,p)
            obj.check3ColumnVector(n);
            obj.check3ColumnVector(t);
            obj.check3ColumnVector(b);
            obj.checkPositionMatrix(p);

            m0 = [ ...
                 0,  1,  0; ...
                 1,  0,  0; ...
                 0,  0, -1  ...
                ];
            m1 = [n t b];
            
            %rotate every column vector in p, and scale everything up with 1000 (from meter to millimeter)
            P = m0*p * 1000;
            
            m1_temp = m0*m1;
            
            m0_temp = [ ...
                 0,  -1,  0; ...
                 -1,  0,  0; ...
                 0,  0, -1  ...
                ];
            Theta_mat = m1_temp*m0_temp;
            Theta = [...
                atan2( -Theta_mat(2,3), Theta_mat(3,3)); ...
                asin( Theta_mat(1,3) ); ...
                atan2(-Theta_mat(1,2), Theta_mat(1,1)) ...
                ];
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function updateRobotParts(obj,position_matrix)
            if ~isequal([3 5], size(position_matrix))
                error(sprintf(['Input position matrix is not in correct dimensions!'...
                    '\nInput position matrix need be a 3x5 matrix']));
            end
            
            column_index = 1;
            for i = 1:numel(obj.parts)
                if isa(obj.parts(i), 'Joint')
                    center_point = position_matrix(:,column_index);
                    obj.parts(i).updatePosition( center_point );
                    column_index = column_index + 1;
                elseif isa(obj.parts(i), 'Link')
                    start_point = position_matrix(:,column_index - 1 );
                    end_point = position_matrix(:,column_index);
                    obj.parts(i).updatePosition( start_point, end_point ); 
                else
                    error('Trying to update a robot part of unknown type');
                end
            end % end of for-loop
        end % end of method
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function checkPositionMatrix(obj, p)
            if ~isequal(3, size(p,1))
                error(sprintf(['Input position matrix is not in correct dimensions!'...
                    '\nInput position matrix needs to be a 3xN matrix']));
            end
            if ~isnumeric(p)
                error(sprintf(['Input position matrix contains non-numeric values!'...
                    '\nInput position matrix can only contain numeric values']));
            end
            if ~isreal(p)
                error(sprintf(['Input position matrix contains complex values!'...
                    '\nInput position matrix can only contain real values']));
            end            
            if ~all(isfinite(p(:)))
                error(sprintf(['Input position matrix contains nan or infinity values!'...
                    '\nInput position matrix can only contain finite values.']));
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function checkJointAngles(obj, joint_angles)
            if ~isequal([6 1], size(joint_angles))
                error(sprintf(['Input joint angles are not in correct dimensions!'...
                    '\nInput joint angles need to be a 6x1 vector!']));
            end
            if ~isnumeric(joint_angles)
                error(sprintf(['Input joint angles contains non-numeric values!'...
                    '\nInput joint angles can only contain numeric values']));
            end
            if ~isreal(joint_angles)
                error(sprintf(['Input joint angles contains complex values!'...
                    '\nInput joint angles can only contain real values']));
            end            
            if ~all(isfinite(joint_angles(:)))
                error(sprintf(['Input joint angles contains nan or infinity values!'...
                    '\nInput joint angles can only contain finite values.']));
            end
            if ~( prod( obj.joint_angles_min_angle < joint_angles ) && ...
                    prod( joint_angles < obj.joint_angles_max_angle ) )

                error(sprintf(['Input joint angles are not within allowed '...
                  'joint angle range for the C3 robot arm!']));
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         function jac_M = jacMatrix(obj,joint_angles)
%             %% A 2x4 matrix, input is joint angles speed and output is a linear transformation matrix 
%             % that maps robot_position_speed = jac_M * joint_angle_speed  
%             % It also provides a relationship between joint torques and the 
%             % resultant force and torque applied by the end-effector:
%             % joint_torque_force = transpose(jac_M)*end_point_force
%             L1 = obj.Link_lengths(1);
%             L2 = obj.Link_lengths(2);
%             L3 = obj.Link_lengths(3);
%             L4 = obj.Link_lengths(4);
%             t1 = joint_angles(1);
%             t2 = joint_angles(2);
%             t3 = joint_angles(3);
%             t4 = joint_angles(4);
%             jac_M =[-L1*sin(t1)-L2*sin(t1+t2)-L3*sin(t1+t2+t3)-L4*sin(t1+t2+t3+t4), ...
%                     -L2*sin(t1+t2)-L3*sin(t1+t2+t3)-L4*sin(t1+t2+t3+t4), ...
%                     -L3*sin(t1+t2+t3)-L4*sin(t1+t2+t3+t4), ...
%                     -L4*sin(t1+t2+t3+t4) ;
%                     L1*cos(t1)+L2*cos(t1+t2)+L3*cos(t1+t2+t3)+L4*cos(t1+t2+t3+t4), ...
%                     L2*cos(t1+t2)+L3*cos(t1+t2+t3)+L4*cos(t1+t2+t3+t4), ...
%                     L3*cos(t1+t2+t3)+L4*cos(t1+t2+t3+t4), ...
%                     L4*cos(t1+t2+t3+t4)];
%         end % end of jacMatrix
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    
    
end

