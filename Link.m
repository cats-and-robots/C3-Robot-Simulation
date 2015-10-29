classdef Link < AbstractClassRobotPart & AbstractClassDraw
    %LINK Represent a link (space between two joints) for a robot arm.
    %   It inherits from abstract class AbstractClassRobotPart and 
    %   AbstractClassDraw
    %
    %   properties (protected)
    %       start_point: 3x1 vector specifing start coordinate of link
    %       end_point: 3x1 vector specifing end coordinate of link
    %
    %   methods
    %       obj = Joint(start_p, end_p): constructor
    %       updatePosition(center_p): update the center point
    %       position = getPosition(): returns the center position
    %       draw(): plots the link in the plot window 
    %       drawUpdate(): updates the drawing animation

    properties (Access = protected)
        % 3-vectors storing the position of the link in 3D space
        start_point;
        end_point;
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    methods
        function obj = Link(start_p, end_p)
            obj.check3ColumnVector(start_p);
            obj.start_point = start_p;
            
            obj.check3ColumnVector(end_p);
            obj.end_point = end_p;              
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function updatePosition(obj, varargin)
            if nargin == 3 % obj plus 2 extra input paramter in varargin
                obj.check3ColumnVector(varargin{1});
                obj.start_point = varargin{1};
                
                obj.check3ColumnVector(varargin{2});
                obj.end_point = varargin{2};
                
            else
                error(sprintf(['Wrong number of input parameters!'...
                '\Link updatePosition method takes 2 input parameters']));
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function position = getPosition(obj)
            position = [obj.start_point obj.end_point];
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function h = draw(obj)
            obj.draw_handle = plot3( ...
                [obj.start_point(1) obj.end_point(1)], ...
                [obj.start_point(2) obj.end_point(2)], ...
                [obj.start_point(3) obj.end_point(3)], ...
                'Color', 'blue','LineWidth', 3 ...
            );
            h = obj.draw_handle;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function drawUpdate(obj)
            set(obj.draw_handle, ... 
                'XData',[obj.start_point(1) obj.end_point(1)], ...
                'YData',[obj.start_point(2) obj.end_point(2)], ...
                'ZData',[obj.start_point(3) obj.end_point(3)] ...
            );
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

