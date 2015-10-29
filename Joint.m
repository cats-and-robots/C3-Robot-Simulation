classdef Joint < AbstractClassRobotPart & AbstractClassDraw
    %JOINT Represent a joint (circle connecting two links) for a robot arm.
    %   It inherits from abstract class AbstractClassRobotPart and 
    %   AbstractClassDraw
    %
    %   properties (protected)
    %       center_point: 3x1 vector containing its center coordinate
    %
    %   methods (public)
    %       obj = Link(center_p): constructor
    %       updatePosition(center_p): update the center point
    %       position = getPosition(): returns the center position
    %       draw(): plots the link in the plot window 
    %       drawUpdate(): updates the drawing animation
    
    properties (Access = protected)
        center_point;
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    methods
        function obj = Joint(center_p)
            obj.check3ColumnVector(center_p); 
            obj.center_point = center_p;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function updatePosition(obj, varargin)
            if nargin == 2 % obj plus 1 extra input paramter in varargin
                obj.check3ColumnVector(varargin{1});
                obj.center_point = varargin{1};
            else
                error(sprintf(['Wrong number of input parameters!'...
                '\nJoint updatePosition method takes 1 input parameters']));
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function position = getPosition(obj)
            position = obj.center_point;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function h = draw(obj)
            obj.draw_handle = plot3( ... 
                obj.center_point(1), ... 
                obj.center_point(2), ...
                obj.center_point(3), ...
                'o', 'MarkerSize',10, 'Color', 'blue', 'MarkerFaceColor', 'green' ...
            );
            h = obj.draw_handle;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function drawUpdate(obj)
            set(obj.draw_handle, ... 
                'XData',obj.center_point(1), ...
                'YData',obj.center_point(2), ...
                'ZData',obj.center_point(3) ...
            );
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    
end

