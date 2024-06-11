clc;
clear all;
close all;

r = loadrobot('abbIrb120','DataFormat','column');
q0 = zeros(6,1);
q = q0;

% Define task space boundaries (excluding robot's base area)
x_min = -0.38; x_max = 0.38;
y_min = -0.38; y_max = 0.38;
z_plate = 0.03;  % Plate height
minDistance = 0.25;  % Minimum distance between plates
baseRadius = 0.3;  % Radius of the base exclusion zone

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

% BFGSGradientProjection IK object
ik = inverseKinematics('RigidBodyTree', r);

% Set the orientation to ensure the z-axis points down
orientation = eul2quat([0, pi/2, 0]);  % Rotate 180 degrees around y-axis to point z-axis down

% Initialize plot
figure;
ax = show(r, q, 'Visuals', 'on', 'PreservePlot', 0, 'Fastupdate', 1); hold all;
drawFloor();

% Loop over each plate
for idx = 1:numPlates
    PlatePosition = platePositions(idx, :);
    
    body = rigidBody(['Plate', num2str(idx), '_link']);
    addVisual(body, "Mesh", 'Dinner_Plate_v1.stl', [[0.003*eye(3), zeros(3,1)]; 0 0 0 1]);
    setFixedTransform(body.Joint, trvec2tform(PlatePosition));
    addBody(r, body, r.BaseName);

    % Generate Spiral Trajectory for the plate
    nPoints = 50;
    radius = 0.15 + 0.05 * rand;  % Random radius for each plate
    turns = 3;
    theta = linspace(0, 2*pi*turns, nPoints);
    radii = linspace(0, radius, nPoints);  % Linear increment in radius to form a spiral
    z_above_plate = 0.05;  % Height above the plate where the spiral is shown
    z = ones(1, nPoints) * (z_plate + z_above_plate);  % Constant z-coordinate above the plate
    x = radii .* cos(theta) + PlatePosition(1);
    y = radii .* sin(theta) + PlatePosition(2);

    % Draw spiral above the plate
    for i = 1:nPoints
        % Desired position and orientation for the plate
        Td = trvec2tform([x(i), y(i), z(i)]) * quat2tform(orientation);

        % Find pose with numerical IK
        [q_des, solnInfo] = ik('tool0', Td, ones(6,1), q);

        % Use the desired joint angles directly
        q = q_des;

        % Update plot
        show(r, q, 'Visuals', 'on', 'PreservePlot', 0, 'Frames', 'off', 'Parent', ax);
        plotTransforms(Td(1:3,4)', tform2quat(Td), 'Parent', ax, 'framesize', 0.05);
        plot3(ax, x, y, z, 'm', 'LineWidth', 1);
        drawnow;
    end

    % If not the last plate, generate a trajectory to the next plate
    if idx < numPlates
        nextPlatePosition = platePositions(idx + 1, :);
        midPt = (PlatePosition + nextPlatePosition) / 2;
        midPt(3) = midPt(3) + 0.1; % Raise the midpoint for a smoother trajectory

        % Define time for each segment
        trajTimes = [0, 1, 2]; % Start, mid, and end times
        waypoints = [PlatePosition; midPt; nextPlatePosition]';

        % Generate cubic polynomial trajectory
        [q_traj, qd_traj, qdd_traj] = cubicpolytraj(waypoints, trajTimes, linspace(0, 2, 100));

        for i = 1:size(q_traj, 2)
            % Desired position and orientation
            Td = trvec2tform(q_traj(:, i)') * quat2tform(orientation);

            % Find pose with numerical IK
            [q_des, solnInfo] = ik('tool0', Td, ones(6,1), q);

            % Use the desired joint angles directly
            q = q_des;

            % Update plot
            show(r, q, 'Visuals', 'on', 'PreservePlot', 0, 'Frames', 'off', 'Parent', ax);
            plotTransforms(Td(1:3,4)', tform2quat(Td), 'Parent', ax, 'framesize', 0.05);
            plot3(ax, q_traj(1,:), q_traj(2,:), q_traj(3,:), 'm', 'LineWidth', 1);
            drawnow;
        end
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
