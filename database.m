% Database for the legged robot simulation

function db = database() % To call func in another script, must match the
% name of the file

    % Time
    db.time.t = linspace(0, 8, 1000); % seconds
    db.time.timespan = db.time.t(end) - db.time.t(1);
    db.time.dt = db.time.t(2) - db.time.t(1);
    
    % Robot dimensions
    db.dims.l_1 = 0.16; % m
    db.dims.l_2 = 0.16; 
    db.dims.m_1 = 0.35; % link masses (kg)
    db.dims.m_2 = 0.35;
    db.dims.m = 0.7;
    db.dims.w = 0.03; % width of the square base of the leg in Simulink
    
    % Initial conditions
    db.initial.theta_act = [0 0];
    db.initial.omega_act = [0 0];
    
    % Gait params.
    db.gait.theta_amp = [pi / 12, (5 * pi) / 36]; % Used as theta_ref in Eq 8
    db.gait.phi_hip1 = 0;
    db.gait.phi_knee1 = (5 * pi) / 36;
    db.gait.phi_hip2 = pi; % Only needed in the Simulink simulation as
    % simulating 2 legs (only 1 is tracked and used for calculations)
    db.gait.phi_knee2 = (41 * pi) / 36;
    db.gait.f_hip = 0.875;
    db.gait.f_knee = 0.875;
    
    % Joint dynamics params.
    db.dynamics.r_1 = db.dims.l_1 / 2; % COM locations
    db.dynamics.r_2 = db.dims.l_2 / 2;
    db.dynamics.I_1 = (1/12)*db.dims.m_1*(db.dims.l_1^2 + db.dims.w^2);
    db.dynamics.I_2 = (1/12)*db.dims.m_2*(db.dims.l_2^2 + db.dims.w^2);
    
    % PD controller params.
    db.PDC.K_t = 0.025; % Torque const. (NM/A)
    db.PDC.K_tr = 4;
    db.PDC.K_d = 12.5;
    db.PDC.K_p = 37.5;
    db.PDC.hipK_dp = 50;
    db.PDC.hipK_pp = 150;
    db.PDC.hipK_dr = 50;
    db.PDC.hipK_pr = 150;
    db.PDC.kneeK_d = 50;
    db.PDC.kneeK_p = 150;

    % 3D Simulink sim params.
    db.threeD.sensorRadius = 0.006;
    db.threeD.feetMass = 0.2;
    db.threeD.staticFrictionCoeff = 0.5;
    db.threeD.dynamicFrictionCoeff = 0.3;
    
    % MC params.
    db.MC.B = 100;
    db.MC.H = 100;

end

