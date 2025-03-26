clear; clc; close all;

dt = 0.1;
T_sim = 30;

% Create the UAV scenario.
scene = uavScenario("UpdateRate",100,"StopTime",30);

% Add a ground plane.
color.Gray = 0.651*ones(1,3);
color.Green = [0.3922 0.8314 0.0745];
color.Red = [1 0 0];
addMesh(scene,"polygon",{[-250 -150; 200 -150; 200 180; -250 180],[-4 0]},color.Gray)

initial_pos = [-100,-100,0];
initial_or = [0,0,0];

%load("/Users/danieleturrini/Documents/MATLAB/Examples/R2024a/uav/UAVScenarioTutorialExample/flightData.mat")
% Set up platform
plat = uavPlatform("UAV",scene,"ReferenceFrame","NED");

% Set up platform mesh. Add a rotation to orient the mesh to the UAV body frame.
updateMesh(plat,"quadrotor",{30},color.Red,initial_pos,eul2quat(initial_or))

state = [initial_pos,initial_or];

show3D(scene);
xlim([-250 200])
ylim([-150 180])
zlim([0 150])
axis equal
hold on

GPS_model = gpsSensor('ReferenceFrame','NED');

GPS = uavSensor("GPS",plat,GPS_model,"MountingLocation",[0,0,-1]);
setup(scene);
for i = 1:dt:T_sim
    state(1,1) = state(1,1)+0.1;
    state(1,2) = state(1,2)+0.2;
    state(1,3) = state(1,3)+0.1;

    state(1,4) = 1/2 * cos(i);

    updateMesh(plat,"quadrotor",{30},color.Red,state(1,1:3),eul2quat(state(1,4:6)));

    % Advance the scenario simulation (updates internal timing).
    advance(scene);
    
    % Refresh the visualization.
    drawnow;
    
end