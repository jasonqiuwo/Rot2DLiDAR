lidarRange = 5;
fov = 270;
pivotAngle = 60; % Pivot angle
rotationSpeed = 30; % RPM
scanningFrequency = 15; % Scanning frequency (Hz)
duration = 4; 
angularResolution = 0.33;
movementSpeed = 0.5; % Speed along the x-axis (m/s)
rotationDistance = 0; % Distance from center
roomLength = 5; % Length (along x-axis)
roomWidth = 4; % Width (along y-axis)
roomHeight = 3; % Height (along z-axis)

pivotAngleRad = deg2rad(pivotAngle);
fovRad = deg2rad(fov);
rotationSpeedRadPerSecond = deg2rad(360 * (rotationSpeed / 60)); % Convert RPM to rad/s
rotationIncrementRad = rotationSpeedRadPerSecond / scanningFrequency;
angularResolutionRad = deg2rad(angularResolution);

numPoints = ceil(fov / angularResolution);
angles = linspace(-fovRad/2, fovRad/2, numPoints);

cosPivot = cos(pivotAngleRad);
sinPivot = sin(pivotAngleRad);

figure;
hold on;
grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
view(3);
title('3D LiDAR Scans');
xlim([-roomLength, roomLength]);
ylim([-roomWidth, roomWidth]);
zlim([-roomHeight, roomHeight]);

walls = [
    -roomLength/2, roomLength/2;  % x-bounds
    -roomWidth/2, roomWidth/2;    % y-bounds
    -roomHeight/2, roomHeight/2   % z-bounds
];

totalScans = scanningFrequency * duration;
for scanNum = 1:totalScans
    rotationAngle = scanNum * rotationIncrementRad;
    currentXPos = movementSpeed * (scanNum / scanningFrequency);
    
    xGlobal = [];
    yGlobal = [];
    zGlobal = [];
    for angleIdx = 1:numPoints
        angle = angles(angleIdx);
        
        % Calculate LiDAR beam in local frame
        xLocal = cos(angle);
        yLocal = sin(angle);
        
        % Apply the pivot angle
        xPivot = xLocal;
        yPivot = yLocal * cosPivot;
        zPivot = -xLocal * sinPivot;
        
        % Apply the rotation around the z-axis
        xRot = xPivot * cos(rotationAngle) - yPivot * sin(rotationAngle);
        yRot = xPivot * sin(rotationAngle) + yPivot * cos(rotationAngle);
        zRot = zPivot;
        
        % Offset for rotation distance (if no offset then no change)
        xRot = xRot + rotationDistance * cos(rotationAngle);
        yRot = yRot + rotationDistance * sin(rotationAngle);
        
        % Calculate the global position based on currentXPos
        xStart = currentXPos + rotationDistance * cos(rotationAngle);
        yStart = rotationDistance * sin(rotationAngle);
        zStart = 0;
        
        % Determine the distance to the nearest wall in the room
        tMax = inf; 
        if xRot ~= 0
            tX1 = (walls(1, 2) - xStart) / xRot;
            if tX1 > 0 && tX1 < tMax
                tMax = tX1;
            end
            tX2 = (walls(1, 1) - xStart) / xRot;
            if tX2 > 0 && tX2 < tMax
                tMax = tX2;
            end
        end
        
        if yRot ~= 0
            tY1 = (walls(2, 2) - yStart) / yRot;
            if tY1 > 0 && tY1 < tMax
                tMax = tY1;
            end
            tY2 = (walls(2, 1) - yStart) / yRot;
            if tY2 > 0 && tY2 < tMax
                tMax = tY2;
            end
        end
        
        if zRot ~= 0
            tZ1 = (walls(3, 2) - zStart) / zRot;
            if tZ1 > 0 && tZ1 < tMax
                tMax = tZ1;
            end
            tZ2 = (walls(3, 1) - zStart) / zRot;
            if tZ2 > 0 && tZ2 < tMax
                tMax = tZ2;
            end
        end
        
        % Calculate the global coordinates after wall detection
        xGlobal(end+1) = xStart + tMax * xRot;
        yGlobal(end+1) = yStart + tMax * yRot;
        zGlobal(end+1) = zStart + tMax * zRot;
    end
    
    scatter3(xGlobal, yGlobal, zGlobal, 'filled');
    drawnow;
    pause(1/scanningFrequency);
end

hold off;