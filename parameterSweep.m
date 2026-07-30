db = database();

function PLog = PowerConsumption(K_t, currentKnee, currentHipPitch, ...
    currentHipRoll, omegaKnee, omegaHipPitch, omegaHipRoll)
% Calculates the power consumption required for movement of the legged
% robot

    N = length(currentKnee);
    
    PLog = zeros(1,N);
    
    for k = 1:N
        P = K_t * ( ...
            abs(currentHipRoll(k))  * abs(omegaHipRoll(k))  + ...
            abs(currentHipPitch(k)) * abs(omegaHipPitch(k)) + ...
            abs(currentKnee(k))     * abs(omegaKnee(k)) );
    
        PLog(k) = P;
    end

end

function COTLog = CostOfTransport(E, m, g, d)
% Calculates the cost of transport for the legged robot
% E = total energy consumption

    N = length(E);
    COTLog = zeros(N,1);
    
    for k = 1:N
    
        COT = E(k) / (m * g * d);
        COTLog(k) = COT;
    
    end

end

hipPitchKp = [100 150];
hipPitchKd = [40 50];

kneeKp = [100 150];
kneeKd = [40 50];

results = [];
count = 1;

mass = db.dims.m_1 + db.dims.m_2 + 0.486;
g = 9.81;

for i = 1:length(hipPitchKp)
    for j = 1:length(hipPitchKd)
        for m = 1:length(kneeKp)
            for n = 1:length(kneeKd)
                
                % Loading defaults
                db = database();

                % Updating the controller gains
                db.PDC.hipK_pp = hipPitchKp(i);
                db.PDC.hipK_dp = hipPitchKd(j);

                db.PDC.kneeK_p = kneeKp(m);
                db.PDC.kneeK_d = kneeKd(n);

                % Sending database to Simulink
                assignin('base','db',db);

                % Running simulation
                tic
                simOut = sim('leggedRobotSim3DPrimitive.slx');
                toc

                t = simOut.currentKnee.time;
                currentKnee = simOut.currentKnee.Data;
                currentHipPitch = simOut.currentHipPitch.Data;
                currentHipRoll = simOut.currentHipRoll.Data;

                omegaKnee = simOut.omega_actKnee.Data;
                omegaHipPitch = simOut.omega_actHipPitch.Data;
                omegaHipRoll = simOut.omega_actHipRoll.Data;

                distance = abs(simOut.distanceTravelled.Data(end));

                power = PowerConsumption(...
                    db.PDC.K_t,...
                    currentKnee,...
                    currentHipPitch,...
                    currentHipRoll,...
                    omegaKnee,...
                    omegaHipPitch,...
                    omegaHipRoll);

                energy = trapz(t,power);

                COT = CostOfTransport(...
                    energy,...
                    mass,...
                    g,...
                    distance);

                disp([db.PDC.hipK_pp, db.PDC.hipK_dp,...
                    db.PDC.kneeK_p, db.PDC.kneeK_d])

                % Saving results
                results(count,:) = [...
                    db.PDC.hipK_pp ...
                    db.PDC.hipK_dp ...
                    db.PDC.kneeK_p ...
                    db.PDC.kneeK_d ...
                    distance ...
                    COT];

                count = count + 1;

            end
        end
    end
end

score = results(:,5)./results(:,6);

results = [results score];

resultsTable = array2table(results,...
    'VariableNames',...
    {'Hip Pitch K_p','Hip Pitch K_d',...
    'Knee K_p','Knee K_d',...
    'Distance (m)','COT', 'Score'});

disp(resultsTable)

% Extracting the combination that produces the highest distance
[bestDistance,index] = max(resultsTable.("Distance (m)"));

bestControllerDistance = resultsTable(index,:);
disp('------ Results for highest distance ------')
disp(bestControllerDistance)

% Extracting the combination that produces the lowest COT
[bestCOT,index] = min(resultsTable.COT);

bestControllerCOT = resultsTable(index,:);
disp('------ Results for lowest COT ------')
disp(bestControllerCOT)

% Extracting the combination that produces the highest score
[bestScore,index] = max(resultsTable.Score);

bestControllerScore = resultsTable(index,:);
disp('------ Results for highest score ------')
disp(bestControllerScore)
