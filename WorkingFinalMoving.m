lidarRange = 5; % Maximum range
fov = 270; 
pivotAngle = 90; % Pivot angle 
rpm = 100;
rotationSpeed = rpm / 60.0; % Revolutions per second
offset = 0; 
scansPerSecond = 15; % Scanning Frequency
duration = 1; % Simulation Time
angularResolution = 0.33; % Angular resolution in degrees
movementSpeed = 0; % Speed along the x-axis (m/s)

pivotAngleRad = deg2rad(pivotAngle);
fovRad = deg2rad(fov);
rotationSpeedRadPerScan = deg2rad(360 * rotationSpeed / scansPerSecond);
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
% title(['PointCloud for ', num2str(pivotAngle), ' deg and ', num2str(rpm), ' RPM']);

totalScans = scansPerSecond * duration;

for scanNum = 1:totalScans
    rotationAngle = scanNum * rotationSpeedRadPerScan;
    currentXPos = movementSpeed * (scanNum / scansPerSecond);
    distances = lidarRange * rand(1, numPoints);
    
    xLocal = distances .* cos(angles);
    yLocal = distances .* sin(angles);
    
    xPivot = xLocal;
    yPivot = yLocal * cosPivot;
    zPivot = yLocal * sinPivot; % Change to yLocal if scan vertical, otherwise xLocal
    
    xGlobal = xPivot * cos(rotationAngle) - yPivot * sin(rotationAngle) + offset + currentXPos;
    yGlobal = xPivot * sin(rotationAngle) + yPivot * cos(rotationAngle);
    zGlobal = zPivot;
    
    scatter3(xGlobal, yGlobal, zGlobal, 'filled');
    drawnow;
    pause(1/scansPerSecond);
end

hold off;