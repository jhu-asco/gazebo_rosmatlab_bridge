classdef MatlabPqpManager<handle
    %MATLABPQPMANAGER Loads Pqp meshes and provides routines to compute
    %distances between two meshes. If the first mesh is inside second
    %provides negative distance implying collision.
    
    properties (Access = private)
        Mexdata;%Data of models stored by the mex interface
    end
    
    methods (Access = public)
        function h = MatlabPqpManager()
            %Starts pqp interface
            h.Mexdata = mex_pqp('new');%Create an interface
        end
        function Loadmesh(h,input,modelname)
            %input can be filename or just a double signifying a
            %sphere of that radius.
            %Load the mesh with given filename. The modelname is used to
            %access the mesh later for changings its position etc
           mex_pqp('loadmesh',h.Mexdata,modelname,input); 
        end
        function Setmeshstate(h,modelname,rigidbodystate)
            %Takes in the rigidbody state and sets the mesh to the state
            %Does not care about the twist since this is static object
            mex_pqp('setmeshstate', h.Mexdata, modelname,rigidbodystate.getpositionandorientation()); 
        end
        function [distance, points, triangle] = Computedistance(h,inputmodelname, targetmodelname, inputmodelstate, targetmodelstate)
            %Find the distance between the meshes. Will give negative
            %distance if inputmodel is inside the outputmodel. This allows
            %to know if one mesh is colliding with other
            % state1 and state2 are optional. If not provided uses the
            % current state. 
            % 
            if nargin == 5
                h.Setmeshstate(inputmodelname, inputmodelstate);
                h.Setmeshstate(targetmodelname, targetmodelstate);
            end
            if nargout == 1
                distance = mex_pqp('computedistance', h.Mexdata, inputmodelname, targetmodelname);
            elseif nargout == 2
                [distance, points] = mex_pqp('computedistance', h.Mexdata, inputmodelname, targetmodelname);
            elseif nargout == 3
                [distance, points, triangle] = mex_pqp('computedistance', h.Mexdata, inputmodelname, targetmodelname);
            end
        end
    end
    
end

