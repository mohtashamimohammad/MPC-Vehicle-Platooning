function prediction = create_prediction_matrices(stateMatrix,inputMatrix,horizonLength)
%CREATE_PREDICTION_MATRICES Build the lifted prediction model for incremental inputs.
% Inputs:
%   stateMatrix   : Discrete-time state matrix A.
%   inputMatrix   : Discrete-time input matrix B.
%   horizonLength : Prediction horizon N.
% Output:
%   prediction : Lifted matrices used by the MPC.
% Plant and incremental-input model:
%   X(k+1) = A*X(k) + B*U(k)
%   U(k)   = U(k-1) + DeltaU(k)
% Lifted prediction:
%   Xpred = Phi*X(k) + Lambda*U(k-1) + Gamma*DeltaUstack
% In this file: Phi=stateFromCurrent, Lambda=stateFromPreviousInput, Gamma=stateFromInputChanges.

% --- Determine model dimensions and allocate Phi, Lambda, and Gamma. ---
stateCount = size(stateMatrix,1);
inputCount = size(inputMatrix,2);

stateFromCurrent = zeros(horizonLength*stateCount,stateCount);
stateFromPreviousInput = zeros(horizonLength*stateCount,inputCount);
stateFromInputChanges = zeros(horizonLength*stateCount,horizonLength*inputCount);

% Update A^j and the step response (I+A+...+A^(j-1))*B.
statePower = eye(stateCount);
stepInputResponse = zeros(stateCount,inputCount);
stepResponses = cell(horizonLength,1);

% --- Propagate powers of A and construct the lower block-triangular lifted input map. ---
for predictionStep = 1:horizonLength
    statePower = statePower*stateMatrix;
    stepInputResponse = stateMatrix*stepInputResponse + inputMatrix;

    stateRows = (predictionStep-1)*stateCount + (1:stateCount);
    stateFromCurrent(stateRows,:) = statePower;
    stateFromPreviousInput(stateRows,:) = stepInputResponse;
    stepResponses{predictionStep} = stepInputResponse;

    % Insert the effect of each past/future DeltaU block into the appropriate prediction rows.
    for inputStep = 1:predictionStep
        inputColumns = (inputStep-1)*inputCount + (1:inputCount);
        stateFromInputChanges(stateRows,inputColumns) = ...
            stepResponses{predictionStep-inputStep+1};
    end
end

% --- Store the lifted matrices and dimensions in a single prediction structure. ---
prediction.stateFromCurrent = stateFromCurrent;
prediction.stateFromPreviousInput = stateFromPreviousInput;
prediction.stateFromInputChanges = stateFromInputChanges;
prediction.horizonLength = horizonLength;
prediction.stateCount = stateCount;
prediction.inputCount = inputCount;
end
