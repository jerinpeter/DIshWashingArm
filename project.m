clc;
clear all;
close all;

% {'analytic','numeric','generalized'}
ik_method = "numeric";

r = loadrobot('abbIrb120','DataFormat','column');
q0 = zeros(6,1);
q = q0;

% Define task space boundaries (excluding robot's base area)
x_min = 0.0; x_max = 0.38;
y_min = -0.38; y_max = 0.38;
z_plate = 0.03;  % Plate height
minDistance = 0.28;  % Minimum distance between plates
baseRadius = 0.25;  % Radius of the base exclusion zone

% Number of plates
numPlates = 3;

% Function to generate random plate positions within task space
generateRandomPosition = @() [x_min + (x_max-x_min)*rand, y_min + (y_max-y_min)*rand, z_plate];

% Generate random positions for plates ensuring no overlap and avoiding base
platePositions = zeros(numPlates, 3);
for i = 1:numPlates
    isValid = false;
    while ~isValid
        newPosition = generateRandomPosition();
        distances = sqrt(sum((platePositions(1:i-1,1:2) - newPosition(1:2)).^2, 2));
        distanceFromBase = norm(newPosition(1:2));  % Distance from robot's base
        if all(distances > minDistance) && distanceFromBase > baseRadius
            platePositions(i, :) = newPosition;
            isValid = true;
        end
    end
end

% Initialize plot
figure;
ax = show(r, q, ...
    'Visuals', 'on', ...
    'PreservePlot', 0, ...
    'Fastupdate', 1); hold all;
drawFloor();

% Show all plates first
for idx = 1:numPlates
    PlatePosition = platePositions(idx, :);
    body = rigidBody(['Plate', num2str(idx), '_link']);
    addVisual(body, "Mesh", 'Dinner_Plate_v1.stl', [[0.003*eye(3), zeros(3,1)]; 0 0 0 1]);
    setFixedTransform(body.Joint, trvec2tform(PlatePosition));
    addBody(r, body, r.BaseName);
    show(r, 'Visuals', 'on', 'PreservePlot', 0, 'Frames', 'off', 'Parent', ax);
    drawnow;
end

% BFGSGradientProjection IK object
ik = inverseKinematics('RigidBodyTree', r);

% Set the orientation to ensure the z-axis points down
orientation = eul2quat([0, pi/2, 0]);  % Rotate 180 degrees around y-axis to point z-axis down

% PD Controller parameters
Kp = 20; % Proportional gain
Kd = 0.1; % Derivative gain
dt = 0.01; % Time step

% Initialize visited plates array
visitedPlates = false(numPlates, 1);

% Start position
currentPosition = [0, 0, 0];  % Assuming the robot starts at the origin

% Loop until all plates are visited
for visitCount = 1:numPlates
    % Find the nearest unvisited plate
    distances = sqrt(sum((platePositions(:,1:2) - currentPosition(1:2)).^2, 2));
    distances(visitedPlates) = inf;  % Ignore already visited plates
    [~, idx] = min(distances);  % Find the index of the nearest plate
    PlatePosition = platePositions(idx, :);
    visitedPlates(idx) = true;  % Mark this plate as visited

    % Generate Spiral Trajectory for the plate
    nPoints = 55;
    radius = 0.145;  % Random radius for each plate
    turns = 3;
    theta = linspace(0, 2*pi*turns, nPoints);
    radii = linspace(0, radius, nPoints);  % Linear increment in radius to form a spiral
    z_above_plate = 0.05;  % Height above the plate where the spiral is shown
    z = ones(1, nPoints) * (z_plate + z_above_plate);  % Constant z-coordinate above the plate
    x = radii .* cos(theta) + PlatePosition(1);
    y = radii .* sin(theta) + PlatePosition(2);

    % Initialize error terms for PD control
    prevError = zeros(6,1);

    % Draw spiral above the plate
    for i = 1:nPoints
        % Desired position and orientation for the plate
        Td = trvec2tform([x(i), y(i), z(i)]) * quat2tform(orientation);

        % Find pose with numerical IK
        [q_desired, solnInfo] = ik('tool0', Td, ones(6,1), q);

        % Calculate error for PD control
        error = q_desired - q;
        dError = (error - prevError) / dt;

        % PD control law
        u = Kp * error + Kd * dError;

        % Update joint positions
        q = q + u * dt;
        prevError = error;

        % Update plot
        show(r, q, ...
            'Visuals', 'on', ...
            'PreservePlot', 0, ...
            'Frames', 'off', ...
            'Parent', ax);
        plotTransforms(Td(1:3,4)', tform2quat(Td), ...
            'Parent', ax, ...
            'framesize', 0.05);
        plot3(ax, x, y, z, 'm', 'LineWidth', 1);
        drawnow;
    end

    % Update current position to the position of the last point in the spiral
    currentPosition = [x(end), y(end), z(end)];

    % Generate Transition Trajectory using cubicpolytraj if it's not the last plate
    if visitCount < numPlates
        % Find the nearest unvisited plate again for the next target
        distances = sqrt(sum((platePositions(:,1:2) - currentPosition(1:2)).^2, 2));
        distances(visitedPlates) = inf;  % Ignore already visited plates
        [~, nextIdx] = min(distances);  % Find the index of the nearest plate
        nextPlatePosition = platePositions(nextIdx, :);

        nTransitionPoints = 50;
        transitionTime = linspace(0, 1, nTransitionPoints);

        % Calculate the angles for the waypoints around the robot
        angle1 = atan2(currentPosition(2), currentPosition(1));
        angle2 = atan2(nextPlatePosition(2), nextPlatePosition(1));
        
        % Normalize angles to range [0, 2*pi]
        if angle1 < 0
            angle1 = angle1 + 2*pi;
        end
        if angle2 < 0
            angle2 = angle2 + 2*pi;
        end
        
        % Calculate the shortest arc
        if angle2 < angle1
            angle2 = angle2 + 2*pi;
        end
        clockwiseDistance = angle2 - angle1;
        counterClockwiseDistance = 2*pi - clockwiseDistance;
        
        if clockwiseDistance <= counterClockwiseDistance
            midAngles = linspace(angle1, angle2, 5);
        else
            midAngles = linspace(angle1, angle2 - 2*pi, 5);
        end

        midRadius = max(baseRadius + 0.1, 0.3);  % Ensure radius is large enough to avoid the robot

        % Create waypoints around the robot
        midPoints = [midRadius*cos(midAngles') midRadius*sin(midAngles') repmat(z_above_plate + 0.2, length(midAngles), 1)];
        waypoints = [currentPosition; midPoints; nextPlatePosition(1), nextPlatePosition(2), z_plate + z_above_plate];
        waypointsTime = linspace(0, 1, size(waypoints, 1));

        [transitionTraj, ~, ~] = cubicpolytraj(waypoints', waypointsTime, transitionTime);
        xTransition = transitionTraj(1, :);
        yTransition = transitionTraj(2, :);
        zTransition = transitionTraj(3, :);

        % Initialize error terms for PD control
        prevError = zeros(6,1);

        % Draw transition trajectory
        for i = 1:nTransitionPoints
            % Desired position and orientation for the transition
            Td = trvec2tform([xTransition(i), yTransition(i), zTransition(i)]) * quat2tform(orientation);
            
            % Find pose with numerical IK
            [q_desired, solnInfo] = ik('tool0', Td, ones(6,1), q);

            % Calculate error for PD control
            error = q_desired - q;
            dError = (error - prevError) / dt;

            % PD control law
            u = Kp * error + Kd * dError;

            % Update joint positions
            q = q + u * dt;
            prevError = error;

            % Update plot
            show(r, q, ...
                'Visuals', 'on', ...
                'PreservePlot', 0, ...
                'Frames', 'off', ...
                'Parent', ax);
            plotTransforms(Td(1:3,4)', tform2quat(Td), ...
                'Parent', ax, ...
                'framesize', 0.05);
            drawnow;
        end

        % Update current position to the position of the last point in the transition
        currentPosition = [xTransition(end), yTransition(end), zTransition(end)];
    end
end

% -----------------------------------------------------------------------------
% Function to draw the floor
% -----------------------------------------------------------------------------
function drawFloor()
    ax = gca;
    ax.CameraViewAngle = 5;
    p = patch([1 -1 -1 1].*0.5, [1 1 -1 -1]*0.5, [0 0 0 0]);
    p.FaceColor = [0.8, 0.8, 0.8];
    axis off;
    xlim([-0.75, 0.75]);
    ylim([-0.75, 0.75]);
    zlim([0, 0.75]);
end
