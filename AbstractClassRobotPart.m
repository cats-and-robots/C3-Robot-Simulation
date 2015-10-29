classdef AbstractClassRobotPart < matlab.mixin.Heterogeneous & matlab.mixin.Copyable
    % ABSTRACTCLASSROBOTPART Abstract class for creating a part of the 
    %   robot.
    %
    %   methods [ABSTRACT] (public)
    %       updatePosition(varargin): update the object's position
    %       position = getPosition(): returns the object's position
    
    methods (Abstract)
        updatePosition(obj,varargin)
        position = getPosition(obj)
    end
    
end