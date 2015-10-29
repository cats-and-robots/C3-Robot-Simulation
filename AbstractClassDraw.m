classdef AbstractClassDraw <  matlab.mixin.Copyable
    %ABSTRACTCLASSDRAW Abstract class describing the methods needed for all
    %   objects that draws something in the simulation.
    %   All object instances that draws something works in the 3 
    %   dimensional space, thus this abstract class also contains a 
    %   method to check if a 3 column vector is valid or not.
    %
    %   properties (protected)
    %       draw_handle: handle for the figure drawing of each object
    %
    %   methods [ABSTRACT] (public)
    %       position = getPosition(): returns the center position
    %       draw(): plot the object in the plot window 
    %       drawUpdate(): updates the drawing animation
    %
    %   methods (protected)
    %       check3ColumnVector(v): produces a error msg for invalid vectors
    
    properties (Access = protected)
        draw_handle
    end
    
    methods (Access = public)
        draw(obj)
        drawUpdate(obj)
    end
    
    methods (Access = protected)
       function check3ColumnVector(obj, v)
            if ~isequal([3 1], size(v))
                error(sprintf(['Input vector is not in correct dimensions!'...
                    '\nInput vector needs to be a 3x1 vector']));
            end
            if ~isnumeric(v)
                error(sprintf(['Input vector contains non-numeric values!'...
                    '\nInput vector can only contain numeric values']));
            end
            if ~isreal(v)
                error(sprintf(['Input vector contains complex values!'...
                    '\nInput vector can only contain real values']));
            end            
            if ~all(isfinite(v(:)))
                error(sprintf(['Input vector matrix contains nan or infinity values!'...
                    '\nInput vector can only contain finite values.']));
            end
        end 
    end
    
end

