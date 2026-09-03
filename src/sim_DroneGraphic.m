classdef sim_DroneGraphic < handle
    % sim_DroneGraphic: The 3D graphic for one drone.
    %
    % Separate from sim_QuadrotorPlant so the plant can be simulated and tested
    % with no figure open, and so N drones share one copy of the mesh.
    %
    % Two costs were removed relative to Project_2/3.
    %
    %   1. STL loading. Every robotDrone read and decimated both STL
    %      files in its constructor - 18 times for a 9-drone swarm,
    %      re-parsing 51k triangles each time. loadMeshes caches the
    %      reduced patch data in a persistent, so the files are read once
    %      per MATLAB session.
    %
    %   2. Per-frame transform rebuilding. updateAnimation recomputed the
    %      three CONSTANT sub-transforms (body, propeller and frame
    %      offsets) every frame and called makehgtform four times. The
    %      constant ones are set once here, and the pose matrix is
    %      written directly.

    properties (Access = private)
        hDrone      % hgtransform for the whole drone (pose)
    end

    properties (Constant, Access = private)
        % Mesh alignment, carried over unchanged from Project_2/3 so the
        % drone looks exactly as it did.
        STL_BODY      = [-0.25, -0.15, -0.30];
        STL_PROPELLER = [-0.345, 0.125, -0.365];
        STL_SCALE     = 0.00125;
        AXIS_LENGTH   = 0.5;
    end

    methods
        %% Construction
        function obj = sim_DroneGraphic(ax, bodyColor)
            % Create the drone graphic in a 3D axes.
            %
            % The x-axis is drawn in red, y-axis in green and z-axis in
            % blue, at the centre of mass of the drone.
            %
            % Inputs:
            %          ax - 3D axes handle
            %   bodyColor - 1x3 RGB for the airframe, so drones in a
            %               formation can be told apart

            arguments (Input)
                ax
                bodyColor (1,3) double = [0.85 0.87 0.90]
            end

            % One matrix write on hDrone moves the whole aircraft.
            obj.hDrone = hgtransform('Parent', ax);
            hBody      = hgtransform('Parent', obj.hDrone);
            hProp      = hgtransform('Parent', obj.hDrone);
            hFrame     = hgtransform('Parent', obj.hDrone);

            [bodyMesh, propMesh] = sim_DroneGraphic.loadMeshes();

            patch('Faces', bodyMesh.f, 'Vertices', bodyMesh.v, ...
                'FaceColor', bodyColor, 'EdgeColor', 'none', ...
                'Parent', hBody);

            patch('Faces', propMesh.f, 'Vertices', propMesh.v, ...
                'FaceColor', [0.45 0.45 0.48], 'EdgeColor', 'none', ...
                'Parent', hProp);

            %% Body coordinate frame
            L = sim_DroneGraphic.AXIS_LENGTH;
            line([0 L], [0 0], [0 0], 'Color', 'r', ...
                'LineWidth', 2, 'Parent', hFrame);
            line([0 0], [0 L], [0 0], 'Color', 'g', ...
                'LineWidth', 2, 'Parent', hFrame);
            line([0 0], [0 0], [0 L], 'Color', 'b', ...
                'LineWidth', 2, 'Parent', hFrame);

            %% Constant sub-transforms, set once
            hBody.Matrix = makehgtform( ...
                'xrotate', pi/2, 'yrotate', pi/2, ...
                'translate', sim_DroneGraphic.STL_BODY, ...
                'scale', sim_DroneGraphic.STL_SCALE);

            hProp.Matrix = makehgtform( ...
                'xrotate', pi/2, 'yrotate', pi/2, ...
                'translate', sim_DroneGraphic.STL_PROPELLER, ...
                'scale', sim_DroneGraphic.STL_SCALE);
        end

        %% Pose update
        function update(obj, x)
            % update: Move the graphic to the state x.
            %
            % The pose is a translation followed by the Z-Y-X Euler
            % rotation, H = T(x,y,z)*Rz(psi)*Ry(theta)*Rx(phi), whose
            % third column is
            %   [cPsi*sThe*cPhi + sPsi*sPhi;
            %    sPsi*sThe*cPhi - cPsi*sPhi;
            %    cThe*cPhi]
            % i.e. exactly the thrust direction of Eq. (10). The picture
            % and the dynamics therefore share one convention - worth
            % stating, because a visualisation that silently disagrees
            % with the model is the hardest kind of bug to see.
            %
            % Written out rather than composed with makehgtform, which
            % would validate arguments and multiply four 4x4 matrices per
            % frame per drone.
            %
            % Input:
            %   x - Current 12x1 state vector

            sPsi = sin(x(7)); cPsi = cos(x(7));
            sThe = sin(x(8)); cThe = cos(x(8));
            sPhi = sin(x(9)); cPhi = cos(x(9));

            obj.hDrone.Matrix = [ ...
                cPsi*cThe, cPsi*sThe*sPhi - sPsi*cPhi, ...
                           cPsi*sThe*cPhi + sPsi*sPhi, x(1);
                sPsi*cThe, sPsi*sThe*sPhi + cPsi*cPhi, ...
                           sPsi*sThe*cPhi - cPsi*sPhi, x(2);
                    -sThe,                  cThe*sPhi, ...
                                            cThe*cPhi, x(3);
                        0,                          0, 0, 1];
        end
    end

    methods (Static)
        %% Shared mesh cache
        function [bodyMesh, propMesh] = loadMeshes()
            % loadMeshes: Read and decimate the STL models once per
            % MATLAB session, then hand out the cached result.
            %
            % Outputs:
            %   bodyMesh, propMesh - Structs with fields f and v

            persistent cachedBody cachedProp

            if isempty(cachedBody)
                cachedBody = sim_DroneGraphic.readSTL('DroneBody.STL');
                cachedProp = sim_DroneGraphic.readSTL('DronePropeller.STL');
            end

            bodyMesh = cachedBody;
            propMesh = cachedProp;
        end
    end

    methods (Static, Access = private)
        %% STL reader with decimation
        function mesh = readSTL(fileName)
            % readSTL: Load an STL and reduce its triangle count to 10%.
            %
            % Resolves the file relative to this class rather than to the
            % working directory. Project_2/3 hard-coded
            % 'media/DroneBody.STL', so the simulation only started if
            % MATLAB happened to be cd'ed into the project folder.

            here = fileparts(fileparts(mfilename('fullpath')));
            fullName = fullfile(here, 'media', fileName);

            if ~isfile(fullName)
                error('sim_DroneGraphic:missingSTL', ...
                    'Could not find "%s".', fullName);
            end

            fv = stlread(fullName);

            % stlread returns a triangulation on R2018b and later, a
            % struct on older releases.
            if isa(fv, 'triangulation')
                f = fv.ConnectivityList;
                v = fv.Points;
            else
                f = fv.faces;
                v = fv.vertices;
            end

            [mesh.f, mesh.v] = reducepatch(f, v, 0.1);
        end
    end
end
