classdef sim_FormationPlanner < handle
    % sim_FormationPlanner: Consensus formation keeping with collision avoidance.
    %
    % Keeps the idea of Project_3 - each drone steers toward a weighted
    % agreement with the neighbours it can see - and fixes three defects.
    %
    % 1. THE SHAPE VECTOR WAS BACKWARDS. Project_3 built
    %        s_ij  = scale*(xs_j - xs_i)
    %        x_des = sum_j c_ij * (x_j - x_i + s_ij)
    %    At the correct formation, x_j - x_i equals xs_j - xs_i, so every
    %    term became 2*(xs_j - xs_i) instead of zero: the formation was
    %    not a fixed point of its own control law. The result was then
    %    added to formationCenter rather than to the drone's own
    %    position, commanding every drone to the same centre point
    %    regardless of its assigned slot.
    %
    %    The law used here,
    %        x_i_des = sum_j c_ij * (x_j + s_i - s_j),  sum_j c_ij = 1
    %    has the formation as an exact fixed point, because at
    %    x_j = x_c + s_j every term is x_c + s_i.
    %
    % 2. IT RECOMPUTED EVERYTHING PER DRONE. calculateNeighbors and
    %    computeConfidenceMatrix were called inside the per-drone loop,
    %    so the whole N-by-N neighbour search ran N times per step for
    %    identical results - O(N^3) where O(N^2) suffices.
    %
    % 3. THE LEADER WEIGHT LEAKED. Project_3 added weight to
    %    C(i, leadDrone) for every i and renormalised, but the summation
    %    only ever ran over Neighbors{i}. When the leader was out of
    %    range its weight was silently dropped and the remaining weights
    %    no longer summed to one, biasing the target toward the origin.
    %    Weights here are normalised over exactly the set that is summed.

    properties
        commRange      = 4.0    % [m] neighbour visibility radius
        selfBias       = 2.0    % Weight a drone gives its own estimate
        leaderBias     = 6.0    % Extra weight on a visible leader
        lookahead      = 1.5    % [m] cap on commanded position step
        avoidRadius    = 1.2    % [m] collision influence radius
        avoidGain      = 0.9    % Peak repulsion offset [m]
        predictHorizon = 0.4    % [s] velocity extrapolation
    end

    properties (SetAccess = private)
        shape       % 3xN formation offsets from the formation centroid
    end

    properties (SetAccess = immutable)
        leaderIdx
        numDrones
    end

    methods
        %% Construction
        function obj = sim_FormationPlanner(shapeTemplate, leaderIdx, order)
            % Constructor.
            %
            % Inputs:
            %   shapeTemplate - 3xN desired formation, any origin
            %       leaderIdx - Index of the lead drone
            %           order - Optional 1xN permutation assigning slots
            %                   to drones, leader kept on its own slot

            arguments (Input)
                shapeTemplate (3,:) double
                leaderIdx     (1,1) double
                order         (1,:) double = []
            end

            % Re-centre the template so offsets are relative to the
            % formation centroid. The law only uses differences
            % s_i - s_j, so the origin cannot affect the shape - but
            % centring makes the leader target mean what it says.
            obj.shape     = shapeTemplate - mean(shapeTemplate, 2);
            obj.leaderIdx = leaderIdx;
            obj.numDrones = size(shapeTemplate, 2);

            if ~isempty(order)
                obj.shape = obj.shape(:, order);
            end
        end

        %% Desired positions for the whole swarm
        function targets = desiredPositions(obj, pos, vel, leaderTarget)
            % desiredPositions: One consensus + avoidance update for
            % every drone.
            %
            % Inputs:
            %            pos - 3xN current positions [m]
            %            vel - 3xN current velocities [m/s]
            %   leaderTarget - 3x1 goal for the formation centroid [m]
            %
            % Output:
            %        targets - 3xN commanded positions [m]

            N = obj.numDrones;
            targets = zeros(3, N);

            % Pairwise distances, computed once for the whole swarm by
            % implicit expansion instead of a nested loop over pairs.
            dx = pos(1, :) - pos(1, :).';
            dy = pos(2, :) - pos(2, :).';
            dz = pos(3, :) - pos(3, :).';
            D  = sqrt(dx.^2 + dy.^2 + dz.^2);

            visible = D <= obj.commRange;
            visible(1:N+1:end) = false;   % never its own neighbour

            for i = 1:N
                if i == obj.leaderIdx
                    % The leader is the only drone with an absolute
                    % reference; everyone else agrees relative to it.
                    targets(:, i) = leaderTarget + obj.shape(:, i);
                else
                    targets(:, i) = obj.consensus( ...
                        i, pos, visible(i, :), leaderTarget);
                end
            end

            targets = targets + obj.avoidance(pos, vel, D);
            targets = obj.limitStep(targets, pos);
        end
    end

    methods (Access = private)
        %% Weighted agreement target for one follower
        function target = consensus(obj, i, pos, vis, leaderTarget)
            % consensus: x_i_des = sum_j c_ij*(x_j + s_i - s_j)
            %
            % Weights are normalised over exactly the index set that is
            % summed - self plus visible neighbours - so they always
            % total one and the fixed point is exact.

            nb = find(vis);

            if isempty(nb)
                % Out of contact: steer to the assigned slot around the
                % leader's goal, so an isolated drone rejoins rather
                % than freezing where it is.
                target = leaderTarget + obj.shape(:, i);
                return;
            end

            idx = [i, nb];

            w = ones(1, numel(idx));
            w(1) = obj.selfBias;

            % The leader anchors the formation and is trusted more - but
            % only when it is actually one of the summed terms.
            w(idx == obj.leaderIdx) = ...
                w(idx == obj.leaderIdx) + obj.leaderBias;

            w = w / sum(w);

            % s_i - s_j for every summed j, so the drone aims at its own
            % slot relative to each neighbour's slot.
            offsets = obj.shape(:, i) - obj.shape(:, idx);
            target  = (pos(:, idx) + offsets) * w(:);
        end

        %% Smooth inter-drone repulsion
        function offs = avoidance(obj, pos, vel, D)
            % avoidance: Repulsive position offsets.
            %
            % Project_3 applied a repulsion of constant magnitude within
            % 0.5 m - a discontinuous kick, at a radius smaller than the
            % 0.7 m rotor span it was meant to protect. Here the
            % magnitude ramps linearly to zero at the influence radius,
            % so the commanded position stays continuous.

            N = obj.numDrones;
            offs = zeros(3, N);

            % Extrapolating makes avoidance react to closing speed, not
            % only to current separation.
            future = pos + obj.predictHorizon * vel;
            gate   = obj.avoidRadius + obj.predictHorizon * 4;

            for i = 1:N
                for j = 1:N
                    if i == j || D(i, j) > gate
                        continue;
                    end

                    delta = future(:, i) - future(:, j);
                    d = norm(delta);

                    if d < obj.avoidRadius
                        mag = obj.avoidGain ...
                            * (obj.avoidRadius - d) / obj.avoidRadius;
                        offs(:, i) = offs(:, i) ...
                            + mag * delta / max(d, 1e-6);
                    end
                end
            end
        end

        %% Bound the commanded step
        function targets = limitStep(obj, targets, pos)
            % limitStep: Keep the commanded position within a fixed
            % distance of where the drone is now.
            %
            % Project_3 did this by building a full cubic trajectory -
            % allocating up to 375 points via linspace, for every drone,
            % on every step - and then reading one column out of it.
            % Since that "trajectory" was a straight linspace, the point
            % it wanted is one line of arithmetic.

            delta = targets - pos;
            d = sqrt(sum(delta.^2, 1));

            far = d > obj.lookahead;
            if any(far)
                targets(:, far) = pos(:, far) ...
                    + delta(:, far) .* (obj.lookahead ./ d(far));
            end
        end
    end
end
