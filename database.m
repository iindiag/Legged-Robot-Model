% Database for the legged robot simulation
% -> all of the variables and dimensions are placeholders as the actual
% values for these remain unknown

function db = database() % To call func in another script, must match the
% name of the file

    % Time
    db.time.t = linspace(0, 20, 1000); % seconds
    db.time.timespan = db.time.t(end) - db.time.t(1);
    db.time.dt = db.time.t(2) - db.time.t(1);
    
    % Robot dimensions
    db.dims.l_1 = 0.16; % m
    db.dims.l_2 = 0.16; 
    db.dims.m_1 = 0.35; % link masses (kg)
    db.dims.m_2 = 0.35;
    db.dims.m = 0.7;
    
    % Initial conditions
    db.initial.theta_act = [0 0];
    db.initial.omega_act = [0 0];
    
    % Gait params.
    db.gait.theta_amp = [pi / 8 pi / 16]; % Used as theta_ref in Eq 8
    db.gait.phi_hip1 = 0;
    db.gait.phi_knee1 = pi /2;
    db.gait.phi_hip2 = pi; % Only needed in the Simulink simulation as
    % simulating 2 legs (only 1 is tracked and used for calculations)
    db.gait.phi_knee2 = pi;
    db.gait.f_hip = 0.2;
    db.gait.f_knee = 0.2;
    
    % Joint dynamics params.
    db.dynamics.b = 1; % Viscous damping, for now equal for both joints
    % but needs to be changed when more data available
    db.dynamics.r_1 = 0.08; % COM locations
    db.dynamics.r_2 = 0.08;
    db.dynamics.I_1 = (1/12)*db.dims.m_1*db.dims.l_1^2;
    db.dynamics.I_2 = (1/12)*db.dims.m_2*db.dims.l_2^2;
    
    % PD controller params.
    db.PDC.K_t = 0.025; % Torque const. (NM/A)
    db.PDC.K_d = 1;
    db.PDC.K_p = 1;
    
    % MC params.
    db.MC.B = 100;
    db.MC.H = 100;

end

