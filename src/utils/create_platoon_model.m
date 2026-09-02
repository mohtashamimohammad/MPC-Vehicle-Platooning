function model = create_platoon_model(params)
%CREATE_PLATOON_MODEL Build the centralized discrete-time model of the full platoon.
% Input:
%   params : Project parameters from platoon_parameters.
% Output:
%   model : Centralized A/B matrices plus each individual vehicle model.
% Central state order:
%   X = [p1 ... pM, v1 ... vM, a1 ... aM]'
% Dynamics:
%   X(k+1) = A*X(k) + B*U(k)
% Vehicles are dynamically independent in the plant model; coupling is introduced by the MPC cost and constraints.

% --- Read dimensions and initialize storage for individual vehicle models. ---
vehicleCount = params.vehicleCount;
sampleTime = params.sampleTime;

model.vehicleStateMatrix = cell(vehicleCount,1);
model.vehicleInputMatrix = cell(vehicleCount,1);

% --- Allocate diagonal blocks used to assemble the centralized A and B matrices. ---
positionFromAcceleration = zeros(vehicleCount);
velocityFromAcceleration = zeros(vehicleCount);
accelerationMemory = zeros(vehicleCount);
positionFromInput = zeros(vehicleCount);
velocityFromInput = zeros(vehicleCount);
accelerationFromInput = zeros(vehicleCount);

% --- Discretize each heterogeneous vehicle and place its coefficients on the diagonal blocks. ---
for vehicleIndex = 1:vehicleCount
    [vehicleA,vehicleB] = create_vehicle_model( ...
        params.actuatorTimeConstant(vehicleIndex),sampleTime);

    model.vehicleStateMatrix{vehicleIndex} = vehicleA;
    model.vehicleInputMatrix{vehicleIndex} = vehicleB;

    positionFromAcceleration(vehicleIndex,vehicleIndex) = vehicleA(1,3);
    velocityFromAcceleration(vehicleIndex,vehicleIndex) = vehicleA(2,3);
    accelerationMemory(vehicleIndex,vehicleIndex) = vehicleA(3,3);
    positionFromInput(vehicleIndex,vehicleIndex) = vehicleB(1);
    velocityFromInput(vehicleIndex,vehicleIndex) = vehicleB(2);
    accelerationFromInput(vehicleIndex,vehicleIndex) = vehicleB(3);
end

% --- Assemble the centralized state-space model in [p;v;a] state order. ---
identityVehicles = eye(vehicleCount);
zeroVehicles = zeros(vehicleCount);

model.stateMatrix = [identityVehicles, sampleTime*identityVehicles, positionFromAcceleration;
                     zeroVehicles,     identityVehicles,            velocityFromAcceleration;
                     zeroVehicles,     zeroVehicles,                 accelerationMemory];
model.inputMatrix = [positionFromInput; velocityFromInput; accelerationFromInput];
model.stateCount = 3*vehicleCount;
model.inputCount = vehicleCount;
end
