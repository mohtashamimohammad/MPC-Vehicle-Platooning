function command = human_driver_command(params,model,humanVehicleIndex,humanState,sampleIndex)
%HUMAN_DRIVER_COMMAND Compute the actual manual-driver command during manual mode.
% Inputs:
%   params            : Project parameters and physical limits.
%   model             : Platoon model containing the selected vehicle model.
%   humanVehicleIndex : Index of the manually controlled vehicle.
%   humanState        : [position; velocity; acceleration] of that vehicle.
%   sampleIndex       : Discrete-time sample k.
% Output:
%   command.input       : Admissible command applied to the human vehicle.
%   command.targetSpeed : Driver target speed for the current phase.
% Driver law near the target:
%   u_nom = Kp*(v_target-v) - Kd*a
% The nominal command is clipped so the next-sample speed and acceleration remain admissible.

% --- Read the human vehicle current speed and acceleration. ---
currentTime = sampleIndex*params.sampleTime;
currentSpeed = humanState(2);
currentAcceleration = humanState(3);

% --- Select the driver target and compute the nominal manual command for the current scenario phase. ---
if currentTime < params.stopTime
    targetSpeed = 0;
    if currentSpeed > params.stopSwitchSpeed
        nominalInput = params.minimumAcceleration;
    else
        nominalInput = params.humanVelocityGain*(targetSpeed-currentSpeed) - ...
            params.humanAccelerationGain*currentAcceleration;
    end
else
    targetSpeed = params.lowSpeedTarget;
    nominalInput = params.humanVelocityGain*(targetSpeed-currentSpeed) - ...
        params.humanAccelerationGain*currentAcceleration;
end

% --- Predict the next vehicle state as an affine function of the new command. ---
vehicleA = model.vehicleStateMatrix{humanVehicleIndex};
vehicleB = model.vehicleInputMatrix{humanVehicleIndex};
stateWithoutNewInput = vehicleA*humanState;

% --- Start with actuator limits and tighten them using next-sample speed and acceleration limits. ---
minimumAdmissibleInput = params.minimumAcceleration;
maximumAdmissibleInput = params.maximumAcceleration;

% Intersect the command interval with the admissible interval for velocity and acceleration.
for stateRow = [2 3]
    if stateRow == 2
        minimumState = params.minimumSpeed;
        maximumState = params.maximumSpeed;
    else
        minimumState = params.minimumAcceleration;
        maximumState = params.maximumAcceleration;
    end

    minimumAdmissibleInput = max(minimumAdmissibleInput, ...
        (minimumState-stateWithoutNewInput(stateRow))/vehicleB(stateRow));
    maximumAdmissibleInput = min(maximumAdmissibleInput, ...
        (maximumState-stateWithoutNewInput(stateRow))/vehicleB(stateRow));
end

% --- Saturate the nominal driver command to the final one-step admissible interval. ---
nominalInput = min(max(nominalInput,params.minimumAcceleration),params.maximumAcceleration);
appliedInput = min(max(nominalInput,minimumAdmissibleInput),maximumAdmissibleInput);

command.input = appliedInput;
command.targetSpeed = targetSpeed;
end
