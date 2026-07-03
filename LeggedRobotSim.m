% Simulating the legged robot model from 'Improving Legged Robot Locomotion
% by Quantifying Morphological Computation':
% - Gait generation
% - Dynamics of the joint & the PD controller
% - MC calculation
% - COT calculation

% Simulating the trajectory of the knee joint and foot from the gait
% generation equations (reference points)
clc, clearvars

% Loading the database
db = database();

% Extracting the relevant values from database
% Dimensions:
l_1 = db.dims.l_1;
l_2 = db.dims.l_2;

% Gait params.:
theta_ref = db.gait.theta_amp;
phi_hip = db.gait.phi_hip1;
phi_knee = db.gait.phi_knee1;
f_hip = db.gait.f_hip;
f_knee = db.gait.f_knee;

% Time:
t = db.time.t;
dt = db.time.dt;
timespan = db.time.timespan;

function theta = sineWave(theta_ref, t, f_hip, f_knee, phi_hip, phi_knee)
% Calculates the sine waves for the given parameters (this is what
% generates the gait for the hip and knee joints individually):
% theta_ref = joint angle positions (theta_1, theta_2) at time-step t
% t = time-step
% f = frequency
% phi = phase shift
    
    theta_ref_1 = theta_ref(1);
    theta_ref_2 = theta_ref(2);
    
    theta_1 = theta_ref_1 * sin(2 * pi * f_hip * t + phi_hip);
    theta_2 = theta_ref_2 * sin(2 * pi * f_knee * t + phi_knee);

    theta = [theta_1(:) theta_2(:)];

end 

function omega = sineWaveDeriv(theta_ref, t, f_hip, f_knee, phi_hip, ...
    phi_knee)
    % Calculates the derivative of the sine wave trajectory for the hip and
    % knee joints - the derivative has been simplified as theta_ref is time
    % invariant
    
    theta_ref_1 = theta_ref(1);
    theta_ref_2 = theta_ref(2);
    
    omega_1 = theta_ref_1 * 2 * pi * f_hip * cos(2 * pi * f_hip * t + ...
        phi_hip);
    omega_2 = theta_ref_2 * 2 * pi * f_knee * cos(2 * pi * f_knee * t + ...
        phi_knee);
    
    omega = [omega_1(:) omega_2(:)];

end

function CartesianCoords = forwardKinematics(l_1, l_2, theta)
% Calculates the forward kinematics of the foot position:
% l_1 = length connecting the hip and knee joints
% l_2 = length connecting the knee joint to the foot
    
    theta_1 = theta(:,1);
    theta_2 = theta(:,2);

    C_z = -(l_1 * cos(theta_1) + l_2 * cos(theta_1 + theta_2));
    C_x = l_1 * sin(theta_1) + l_2 * sin(theta_1 + theta_2);

    CartesianCoords = [C_x C_z];

end

% Calculating the Cartesian coordinates for the foot position (given 
% corresponding angles)

theta = sineWave(theta_ref, t, f_hip, f_knee, phi_hip, phi_knee);

figure(1)
subplot(2, 2, 1)
plot(t, theta(:,1))
hold on
plot(t, theta(:,2))
xlabel('Time (s)')
ylabel('Angle (rad)')
legend('Hip Joint', 'Knee Joint')
title('Gait Generated Angle Offsets over Time')

CartesianCoords = forwardKinematics(l_1, l_2, theta);

% Plotting the Cartesian coordinates of the foot position
subplot(2, 2, 2);
plot(CartesianCoords(:, 1), CartesianCoords(:, 2), 'b.')
xlabel('x Position (m)');
ylabel('z Position (m)');
title('Foot Trajectory');
grid on;

% Plotting the x and z coordinates against t
subplot(2, 2, 3);
plot(t, CartesianCoords(:,1), 'Color', [1 0.4 0.7])
hold on
plot(t, CartesianCoords(:,2), 'Color', [0.85 0.33 0.1])
hold on
xlabel('Time (s)')
ylabel('Spatial Position (m)')
title('x and z Spatial Foot Positions over Time')
legend('x','z')
grid on

% Calculating the x and z coordinates of the knee position
knee_x = l_1*sin(theta(:,1));
knee_z = -(l_1*cos(theta(:,1)));

% Calculating the x and z coordinates of the foot position
foot_x = CartesianCoords(:,1);
foot_z = CartesianCoords(:,2);

%v = VideoWriter('GaitGeneration.mp4', 'MPEG-4');
%v.FrameRate = 30;
%open(v)

%for i = 1:length(t)
%    figure(1)
%    subplot(2, 2, 4)
%    % Draws 3 points and marks them with an o, connects the dots
%    plot([0 knee_x(i) foot_x(i)], [0 knee_z(i) foot_z(i)], ...
%        '-o', 'Color', [0.5 0 0.5], 'LineWidth', 2)
%    axis equal
%    grid on
%    title('Knee Joint & Foot Spatial Positions over Time')
%    % Keeps the axes consistent, would keep being scaled for every
%    % time-step if not
%    xlim([-0.4 0.4])
%    xlabel('x Position (m)')
%    ylim([-0.5 0.1])
%    ylabel('z Position (m)')
%    drawnow
%    frame = getframe(gcf);
%    writeVideo(v, frame);

%end

%close(v);

%%
% Simulating the joint dynamics of the robot (actual points)

% Extracting the relevant values from database
% Dimensions:
m_1 = db.dims.m_1;
m_2 = db.dims.m_2;

% Joint dynamics params.:
I_1 = db.dynamics.I_1;
I_2 = db.dynamics.I_2;
r_1 = db.dynamics.r_1;
r_2 = db.dynamics.r_2;

%PD controller params.:
K_t = db.PDC.K_t;
K_d = db.PDC.K_d;
K_p = db.PDC.K_p;

% Initial conditions:
theta_act = db.initial.theta_act;
omega_act = db.initial.omega_act;

g = 9.81;
N = length(t);

% Initialising arrays to store theta, omega and current values at each 
% time step
thetaLog = zeros(N,2);
omegaLog = zeros(N,2);
currentLog = zeros(N,2);

function current = PDController(K_p, K_d, theta_ref, theta_act, ...
    omega_ref, omega_act)
% Calculates the current produced by the inner feedback loop:
% K_p = proportional gain
% K_d = derivative gain

    currentHip = K_p * (theta_ref(1) - theta_act(1)) + ...
        K_d * (omega_ref(1) - omega_act(1));

    currentKnee = K_p * (theta_ref(2) - theta_act(2)) + ...
        K_d * (omega_ref(2) - omega_act(2));
    
    current = [currentHip; currentKnee];

end

function tau = MotorCurrent(current, K_t)
% Calculates the torque produced by the PD controller

    tauHip = current(1) * K_t;
    tauKnee = current(2) * K_t;
    
    tau = [tauHip; tauKnee];

end

function [theta_act, omega_act] = dynamicsEuler(theta_act, ...
    omega_act, I_1, I_2, m_1, m_2, r_1, r_2, g, tau, dt, l_1)
% Calculates actual values for the trajectory of the joints based on
% dynamics:
% first link -> thigh
% second link -> shin
% I_1 = inertia of the first link
% I_2 = inertia of the second link
% m_1 = mass of the first link
% m_2 = mass of the second link
% r_1 = COM of the first link
% r_2 = COM of the second link
% l_1 = length of the first link

    % Ensures all vectors are the correct orientation
    theta_act = theta_act(:);
    omega_act = omega_act(:);
    tau = tau(:);

    theta_1 = theta_act(1);
    theta_2 = theta_act(2);
    
    omega_1 = omega_act(1);
    omega_2 = omega_act(2);

    % Inertia matrix
    M11 = I_1 + I_2 ...
        + m_1*r_1^2 ...
        + m_2*(l_1^2 + r_2^2 + 2*l_1*r_2*cos(theta_2));

    M12 = I_2 ...
        + m_2*(r_2^2 + l_1*r_2*cos(theta_2));

    M22 = I_2 + m_2*r_2^2;

    M = [M11 M12;
        M12 M22];

    % Coriolis matrix
    h = m_2*l_1*r_2*sin(theta_2);

    C = [-h*omega_2, -h*(omega_1+omega_2);
        h*omega_1, 0];

    % Gravity vector
    G1 = (m_1*r_1 + m_2*l_1)*g*sin(theta_1) ...
        + m_2*r_2*g*sin(theta_1 + theta_2);

    G2 = m_2*r_2*g*sin(theta_1 + theta_2);

    G = [G1;
        G2];

    % Acceleration 
    qdot = [omega_1;
        omega_2];

    % Rearranging the general robot arm dynamic equation so that the result
    % is alpha, representing q double dot
    alpha = M \ (tau - C*qdot - G); % Backslash performs left matrix
                                    % division efficiently (faster and more
                                    % numerically stable)

    % Integrating velocity numerically - explicit Euler method
    omega_new = omega_act + alpha * dt;

    % Integrating position numerically
    theta_new = theta_act + omega_new * dt;

    omega_act = omega_new;

    theta_act = theta_new;

end

function dx = dynamicsOde45(t, x, db)
% The t argument is required as this function is to be passed through the
% ode45 solver

    % Unpacking the state vectors
    q = x(1:2);
    dq = x(3:4);

    % Unpacking the variables from the database in the function workspace
    % Dimensions
    m_1 = db.dims.m_1;
    m_2 = db.dims.m_2;
    l_1 = db.dims.l_1;

    % Dynamic params.
    I_1 = db.dynamics.I_1;
    I_2 = db.dynamics.I_2;
    r_1 = db.dynamics.r_1;
    r_2 = db.dynamics.r_2;

    % PD controller params.
    K_p = db.PDC.K_p;
    K_d = db.PDC.K_d;
    K_t = db.PDC.K_t;

    % Gait params.
    theta_amp = db.gait.theta_amp;
    phi_hip = db.gait.phi_hip1;
    phi_knee = db.gait.phi_knee1;
    f_hip = db.gait.f_hip;
    f_knee = db.gait.f_knee;
    
    g = 9.81;

    % Gait trajectory
    theta_refTraj = sineWave(theta_amp, t, f_hip, f_knee, phi_hip, ...
        phi_knee);
    omega_refTraj = sineWaveDeriv(theta_amp, t, f_hip, f_knee, phi_hip, ...
        phi_knee);

    % Controller
    current = PDController(K_p, K_d, theta_refTraj, q, ...
        omega_refTraj, dq);

    % Actuator
    tau = MotorCurrent(current, K_t);
    % tau = [1/8 1/8]

    % Dynamics
    M11 = I_1 + I_2 ...
        + m_1*r_1^2 ...
        + m_2*(l_1^2 + r_2^2 + 2*l_1*r_2*cos(q(2)));

    M12 = I_2 ...
        + m_2*(r_2^2 + l_1*r_2*cos(q(2)));

    M22 = I_2 + m_2*r_2^2;

    M = [M11 M12;
        M12 M22];

    % Coriolis matrix
    h = m_2*l_1*r_2*sin(q(2));

    C = [-h*dq(2), -h*(dq(1)+dq(2));
        h*dq(1), 0];

    % Gravity vector
    G1 = (m_1*r_1 + m_2*l_1)*g*sin(q(1)) ...
        + m_2*r_2*g*sin(q(1) + q(2));

    G2 = m_2*r_2*g*sin(q(1) + q(2));

    G = [G1;
        G2];

    ddq = M \ (tau - C*dq - G);

    dx = [dq;
        ddq];

end

% Calculating the dynamic coordinates for the foot position
%dynamicCoords = forwardKinematics(l_1, l_2, thetaLog);

% Plotting the z-Coordinate of the movement of the leg from its mechanics 
% and the PD controller
%figure(7)
%subplot(3,1,1)
%plot(t, dynamicCoords(:,2))
%title('Measured z-Coordinate of Foot over Time')
%xlabel('Time (s)')
%ylabel('z Position (m)')
%grid on

% Calculated the gait generated trajectories of the joints for position and
% angular velocity
theta_refTraj = sineWave(theta_ref, t, f_hip, f_knee, phi_hip, phi_knee);
omega_refTraj = sineWaveDeriv(theta_ref, t, f_hip, f_knee, phi_hip, ...
    phi_knee);

for k = 1:N

    % Generating the reference theta and omega values at time step k
    theta_ref_k = theta_refTraj(k,:).';
    omega_ref_k = omega_refTraj(k,:).';

    % Calculating controller current
    current = PDController(K_p, K_d, theta_ref_k, theta_act, ...
        omega_ref_k, omega_act);

    % Converting current to motor torque
    tau = MotorCurrent(current, K_t);

    % Calculating the actual values for the next time step k+1
    [theta_act, omega_act] = dynamicsEuler(theta_act, ...
        omega_act, I_1, I_2, m_1, m_2, r_1, r_2, g, tau, dt, l_1);

    % Storing results
    thetaLog(k,:) = theta_act;
    omegaLog(k,:) = omega_act;
    currentLog(k,:) = current;

end

% Integrating using ode45
x0 = [0; 0; 0; 0]; % Starting from rest
tspan = [0 timespan];
[t_ode, x] = ode45(@(t_ode,x) dynamicsOde45(t_ode, x, db), tspan, x0);

N_ode = length(t_ode);

currentLog_ode = zeros(N_ode,2);
tauLog_ode = zeros(N_ode,2);

for k = 1:N_ode

    q  = x(k,1:2)';
    dq = x(k,3:4)';

    theta_refTraj_ode = sineWave(theta_ref, t_ode(k), f_hip, f_knee, ...
        phi_hip, phi_knee);

    omega_refTraj_ode = sineWaveDeriv(theta_ref, t_ode(k), f_hip, f_knee, ...
        phi_hip, phi_knee);

    current = PDController(K_p, K_d, theta_refTraj_ode, q, ...
        omega_refTraj_ode, dq);

    tau = MotorCurrent(current, K_t);

    currentLog_ode(k,:) = current';
    tauLog_ode(k,:)     = tau';

end

dynamicCoords = forwardKinematics(l_1, l_2, thetaLog);
C_zCoords = dynamicCoords(:,2);

thetaLog_ode = x(:,1:2);
dynamicOdeCoords = forwardKinematics(l_1, l_2, thetaLog_ode);
C_zCoords_ode = dynamicOdeCoords(:,2);

figure(2)

% Plotting ref Vs act for the hip joint
subplot(2,1,1)
plot(t, theta_refTraj(:,1), 'LineWidth', 1.5, 'Color', [1 0.4 0.7])
hold on
plot(t, thetaLog(:,1), '--', 'LineWidth', 1.5, 'Color', 'b')
legend('Generated Gait (Reference)','Dynamic Model (Actual)')
title('Hip Joint Angle (explicit Euler)')
ylabel('\theta_1 (rad)')
xlabel('Time (s)')
grid on

% Plotting ref Vs act for the knee joint
subplot(2,1,2)
plot(t, theta_refTraj(:,2), 'LineWidth', 1.5, 'Color', [1 0.4 0.7])
hold on
plot(t, thetaLog(:,2), '--', 'LineWidth', 1.5, 'Color', 'b')
legend('Generated Gait (Reference)','Dynamic Model (Actual)')
title('Knee Joint Angle (explicit Euler)')
ylabel('\theta_2 (rad)')
xlabel('Time (s)')
grid on

% Plotting the joint positions and angular velocities over time when solved
% using ode45
figure(3)

% Angular positions
subplot(2, 2, 1)
plot(t, thetaLog(:,1))
hold on
plot(t, thetaLog(:,2))
title('Joint Angular Positions (explicit Euler)')
legend('Hip Joint','Knee Joint')
ylabel('Angle (rad)')
xlabel('Time (s)')
grid on

subplot(2, 2, 2)
plot(t_ode, x(:,1)); hold on;
plot(t_ode, x(:,2));
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('Hip Joint','Knee Joint');
grid on;
title('Joint Angular Positions (ode45)');

% Angular velocities
subplot(2, 2, 3)
plot(t, omegaLog(:,1))
hold on
plot(t, omegaLog(:,2))
title('Joint Angular Velocities (explicit Euler)')
legend('Hip Joint','Knee Joint')
ylabel('Angular Velocity (rad/s)')
xlabel('Time (s)')
grid on

subplot(2, 2, 4)
plot(t_ode, x(:,3)); hold on;
plot(t_ode, x(:,4));
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
legend('Hip Joint','Knee Joint');
grid on;
title('Joint Angular Velocities (ode45)');

% Plotting the current generated by the PD controller for the hip and knee
% joints - explicit Euler
figure(4)

subplot(2, 1, 1)
plot(t, currentLog)
legend('Hip Joint','Knee Joint')
title('PD Controller Output - i(t) (MATLAB, explicit Euler)')
ylabel('Current (Amps)')
xlabel('Time (s)')
grid on

% Plotting current generated by the PD controller with ode45 solver
subplot(2, 1, 2)
plot(t_ode, currentLog_ode(:,1)); hold on;
plot(t_ode, currentLog_ode(:,2));
xlabel('Time (s)');
ylabel('Current (A)');
legend('Hip Joint','Knee Joint');
grid on;
title('PD Controller Output - i(t) (MATLAB, ode45)')

% Plotting the z-Coordinate of the movement of the leg from its mechanics 
% and the PD controller
figure(5)

% Explicit Euler
subplot(2, 1, 1)
plot(t, C_zCoords)
title('z-Coordinate of Foot over Time (MATLAB, explicit Euler)')
xlabel('Time (s)')
ylabel('z Position (m)')
grid on

% ode45
subplot(2, 1, 2)
plot(t_ode, C_zCoords_ode)
title('z-Coordinate of Foot over Time (MATLAB, ode45)')
xlabel('Time (s)')
ylabel('z Position (m)')
grid on

% Plotting torque generated by the motor with ode45 solver
%subplot(2, 1, 2)
%plot(t_ode, tauLog_ode(:,1)); hold on;
%plot(t_ode, tauLog_ode(:,2));
%xlabel('Time (s)');
%ylabel('Torque (Nm)');
%legend('Hip','Knee');
%grid on;
%title('Joint Torque');

figure(6)

subplot(3, 1, 1)
plot(t_ode, C_zCoords_ode)
title('Measured z-Coordinate of Foot over Time (MATLAB, ode45)')
xlabel('Time (s)')
ylabel('z Position (m)')
grid on

subplot(3, 1, 2)
plot(t_ode, x(:,4));
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
legend('Knee Joint');
grid on;
title('Measured Angular Velocity over Time (MATLAB, ode45)');

subplot(3, 1, 3)
plot(t_ode, currentLog_ode(:,2));
xlabel('Time (s)');
ylabel('Current (A)');
legend('Knee Joint');
grid on;
title('PD Controller Output - i(t) (MATLAB, ode45)')

%%
% Simulating the Morphological Computation (MC) measures

% Extracting the relevant values from database
% MC params.:
B = db.MC.B;
H = db.MC.H;

% Initialising an array to store the C_z values
C_zLog = zeros(N,1);

for k = 1:N
    
    % Calculating the C_z values for the foot position
    coords = forwardKinematics(l_1, l_2, thetaLog(k,:));
    C_zLog(k) = coords(:,2);

end

function MCVals = MCCalculation(B, H, C_zLog, currentLog, thetaLog, t)
% Quantification of Morphological Computation (MC)

    function discreteArray = discretization(B, continuousArray)
    % Pre-processing into discretized data by binning the continuous data
    % into a smaller number of bins:
    % 1) takes a continuous value
    % 2) determines which interval ('bin') it falls into
    % 3) replaces the continuous value with an integer label

        minVal = min(continuousArray);
        maxVal = max(continuousArray);

        discreteArray = floor((continuousArray - minVal) ./ ...
            (maxVal - minVal) * B); % Floor func. rounds each element down
        % to the nearest integer

        discreteArray(discreteArray == B) = B-1;

    end

    % Discretizing C_z, angular velocity and electrical current signal
    C_zDiscretized = discretization(B, C_zLog);
    iDiscretized = discretization(B, currentLog(:,2));
    omega_2 = gradient(thetaLog(:,2),t);
    omega_2Discretized = discretization(B, omega_2);

    % Calculates the world state at the current time step
    w = C_zDiscretized + H * omega_2Discretized;
    % Denotes the actuator state, represents the electric current signal to
    % the motor
    a = iDiscretized;

    w_now = w(1:end-1)';
    w_next = w(2:end)';
    a_now = a(1:end-1)';

    N = length(w_now);

    MCVals = zeros(N,1);

    for k = 1:N

        wp = w_next(k);
        wn = w_now(k);
        an = a_now(k);

        % p(w',w,a)
        p_wpwA = sum(w_next==wp & w_now==wn & a_now==an) / N;

        % p(w,a)
        p_wA = sum(w_now==wn & a_now==an) / N;

        % p(a)
        p_A = sum(a_now==an) / N;

        % p(w',a)
        p_wpA = sum(w_next==wp & a_now==an) / N;

        % Conditional probabilities
        alpha = p_wpwA / p_wA;
        p_wp_given_A = p_wpA / p_A;

        % Avoid log(0)
        if p_wpwA > 0 && alpha > 0 && p_wp_given_A > 0

            MC = p_wpwA * log2(alpha / p_wp_given_A);

            MCVals(k) = MC;

        end

    end

end

MCVals = MCCalculation(B, H, C_zLog, currentLog, ...
    thetaLog, t);

MCSmooth = smoothdata(MCVals, 'gaussian', 5);

%figure(4)
%plot(t(1:end-1), MCSmooth)
%xlabel('Time (s)')
%title('Morphological Computation over Time')
%grid on

%%
% Calculating the Cost of Transport (COT) from the simulation

function PLog = PowerConsumption(K_t, currentLog, omegaLog)
% Calculates the power consumption required for movement of the legged
% robot
    
    i_hip = currentLog(:,1);
    i_knee = currentLog(:,2);

    omega_hip = omegaLog(:,1);
    omega_knee = omegaLog(:,2);

    N = length(i_hip);

    PLog = zeros(1,N);
    ELog = zeros(1,N);

    for k = 1:N
        P = K_t * (abs(i_hip(k)) * abs(omega_hip(k)) + abs(i_knee(k)) * ...
            abs(omega_knee(k)));

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

% Extracting the relevant values from database
% Dimensions: 
m = db.dims.m;

g = 9.81;

% Calculating distance travelled
% - in the paper this is summed up the measured stride length over 20
% cycles
d = sum(sqrt(diff(foot_x).^2 + diff(foot_z).^2));

% Power consumption
P = PowerConsumption(K_t, currentLog, omegaLog);

% Cumulative energy consumption
ELog = cumtrapz(t,P);

% Cumulative cost of transport
COT = CostOfTransport(ELog,m,g,d);

%figure(5)
%plot(t, P)
%hold on
%plot(t, ELog)
%hold on
%plot(t, COT)
%xlabel('Time (s)')
%title('Movement Requirements')
%legend('Power Consumption', 'Cumulative Energy Consumption', ...
%    'Cumulative Cost of Transport')
%grid on

%figure(6)
%labels = categorical({'Cost of Transport','Morphological Computation'});
%means = [mean(COT), mean(MCSmooth)];
%stds  = [std(COT), std(MCSmooth)];
%
%b = bar(labels, means);
%b.FaceColor = 'flat';
%b.CData = [0.7 0.2 0.3; 0 0 0.8];
%hold on
%errorbar(labels, means, stds, 'k', 'LineStyle', 'none')
%ylabel('Value')
%title('Mean ± Standard Deviation of MC and COT')
%grid on


