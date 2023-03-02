%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Lab Template for Project 1 TSFS09     %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% $Revision: 1.0 $
% $Date: 2014/06/03 $

%  Decide here if the script generates the validation plots or not by
%  changing the binary varable doPlot
doPlot = 0;                                     % [-] doPlot==1 generate validation plots, doPlot==0 the contrary.
Boost_control = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Load the engine data from Project 1a %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Add the measurement files to the path with: File - Set Path
load('Mätdata\EngineFriction.mat');
load('Mätdata\EngineMapTSFS09.mat');
load('Grupp7\airstep.mat'); % Load here your air step data 
load('Grupp7\fuelstep.mat'); % Load here your fuel step data
load('Grupp7\wgstep.mat'); % Load here your wastegate step data

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%      Extract Data From Engine Map     %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p_af            = EngineMap.p_af;               % [Pa]          Pressure air filter
p_bef_thr       = EngineMap.p_bef_thr;          % [Pa]          Pressure before throttle
p_im            = EngineMap.p_im;               % [Pa]          Pressure intake manifold
p_em            = EngineMap.p_em;               % [Pa]          Pressure exhaust manifold
p_es            = EngineMap.p_es;               % [Pa]          Pressure after turbine
T_af            = EngineMap.T_af;               % [K]           Temperature air filter 
T_bef_thr       = EngineMap.T_bef_thr;          % [K]           Temperature before throttle
T_im            = EngineMap.T_im;               % [K]           Temperature intake manifold
T_em            = EngineMap.T_em;               % [K]           Temperature exhaust manifold
T_es            = EngineMap.T_es;               % [K]           Temperature after turbine
N               = EngineMap.N;                  % [rps]         Engine speed
M_e             = EngineMap.M_e;                % [N*m]         Engine Torque
alpha           = EngineMap.alpha;              % [-]           Throttle angle
m_dot_at        = EngineMap.m_dot_at;           % [kg/s]        Mass flow after throttle
t_inj           = EngineMap.t_inj;              % [s]           Fuel injection time
lambda          = EngineMap.lambda;             % [-]           Continuous lambda signal before catalyst

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%      Set up the engine data     %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
l               = EngineMap.engine.l;           % [m]           Connecting rod length
a               = EngineMap.engine.a;           % [m]           Crank radius 
B               = EngineMap.engine.B;           % [m]           Bore 
n_cyl           = EngineMap.engine.n_cyl;       % [-]           Number of Cylinders
V_d             = 2*a*pi*B^2/4;                 % [m^3]         Displaced Volume (per cylinder)
n_r             = EngineMap.engine.n_r;         % [-]           Crank revolutions per cycle
V_im            = EngineMap.engine.V_im;        % [m^3]         Intake manifold volume
V_em            = EngineMap.engine.V_em;        % [m^3]         Exhaust manifold volume
V_es            = EngineMap.engine.V_es;        % [m^3]         Exhaust system vomume
V_D             = EngineMap.engine.V_D;         % [m^3]         Total displaced volume
r_c             = EngineMap.engine.r_c;         % [-]           Compression ratio

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     Set up the vehicle data    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
VehicleMass     = 1520;                         % [kg]          Vehicle mass
WheelRadius     = 0.3;                          % [m]           Wheel radius
VehicleArea     = 2.0;                          % [m^2]         Vehicle frontal area

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%    Set upp other constants     %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
R_exh           = EngineMap.airfuel.R_exh;      % [J/(kg*K)]    Exhaust gas constant
R_air           = EngineMap.airfuel.R_air;      % [J/(kg*K)]    Air gas constant
gamma_air       = EngineMap.airfuel.gamma_air;  % [J/(kg*K)]    Air ratio of specific heats
gamma_exh       = EngineMap.airfuel.gamma_exh;  % [J/(kg*K)]    Exhaust gas ratio of specific heats
cv_air          = R_air/(gamma_air-1);          % [J/(kg*K)]    Specific heat at constant volume, air @T=[-50..40]C
cv_exh          = R_exh/(gamma_exh-1);          % [J/(kg*K)]    Specific heat at constant volume, exhaust gas 
cp_air          = gamma_air*cv_air;             % [J/(kg*K)]    Specific heat at constant pressure, air @T=[-50..40]C
cp_exh          = gamma_exh*cv_exh;             % [J/(kg*K)]    Specific heat at constant pressure, exhaust gas

p_amb           = max(EngineMap.p_af);          % [kPa]         Ambient pressure
T_amb           = mean(EngineMap.T_af);         % [K]           Ambient temperature, mean of air filter temperature

q_LHV           = EngineMap.airfuel.q_LHV;      % [MJ/kg]       Fuel lower heating value
AFs             = EngineMap.airfuel.AFs;        % [-]           Stochiometric A/F for isooctane


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%    Set upp turbocharger constants     %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

J_tc            = 2.55e-5;                      % [kg*m^2]      Turbocharger Inertia, (From Westin 2002 for MHI TD04HL-15T)

V_ic            = 10e-3;                        % [m^3]         Volume of the intercooler, and pipes between the compressor and the intake manifold
c_tc_fric       = 1e-6;                         % [N*m/(rad/s)] Turbo shaft friction constant
Cd_wg           = 0.8;                          % [-]           Assumed value for WG discharge constant
A_max_wg        = 0.035^2/4*pi;                 % [m^2]         Measured approximation of maximum opening area of WG valve
dp_thrREF       = 10e3;                         % [kPa]         Default desired pressure loss over the throttle

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%      Set up the driver model   %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
KpDriver        = 0.04;                         % [-]           Driver model Kp value
KiDriver        = 0.02;                         % [-]           Driver model Ki value

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     Initialize Boost Control   %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Boost_control   = 0;                            % [-]           0 -> Boost control disabled, 1st project (binary variable)

%The following constants are required to avoid conflict with the
%turbocharger subsystem, the values will be modified for project 2
KpThr           = 1e-6;                         % [-]           Throttle controller feedback setup
TiThr           = 0.1;                          % [-]           Throttle controller feedback setup

KpWg            = 1e-6;                         % [-]           Wastegate controller setup
TiWg            = 4;                            % [-]           Wastegate controller setup

tau_wg          = 0.1;                          % [s]           Response time of wastegate dynamics        
T_ic            = mean(T_bef_thr);              % [K]           Temperature in intercooler control volume (Assume isoterm model)
dC2             = EngineMap.p_af;                        % [m]           Outer compressor impeller diameter, measured by the students.
dT1             = EngineMap.p_af;                        % [m]           Turbine inlet diameter, measured by the students.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     Computations for the Gas Pedal     %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% First plot the measured data to determine the time constant by visual
% inspection.

if doPlot  %Here doPlot is used, avoids the plot if it is set to 0
    figure(1),clf
    plot(airstep.time,airstep.alpha,'r')
    hold on;
    plot(airstep.time,airstep.alpha_ref,'b')
    grid on
    xlabel('Time [s]')
    ylabel('Gas Pedal position [%]')
    legend('Measured','Reference')    
end
% tau_th is determined to be 0.05 s with the given data (time to reach the
% 63% of the final step value)
tau_th      = 0.04;                             %[s]            Determined gas pedal time constant

% print the value
disp(' ')
disp('Gas pedal:')
disp(['tau_th = ' num2str(tau_th) ' [s]'])

% Proceed to validate the model using the simulink model comparation
% template(MeasModelCompare_throttle.slx)

%SimOut = sim('MeasModelCompare_throttle.slx');          % Run the simulink model

%if doPlot
%    figure(2)
%    plot(airstep.time,airstep.alpha_ref,'b')
%    hold on;
%    plot(airstep.time,airstep.alpha,'r')
%    plot(GasPedalValidation_Alpha.time,GasPedalValidation_Alpha.signals.values,'k')
%    xlabel('Time [s]')
%    ylabel('Gas Pedal position [%]')
%    legend('Reference','Measured','Model')    
%end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Computations for the efective area after thottle      %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

PI = p_im./p_bef_thr;

good_points = [];
for i = 1:237
    if PI(i) < 0.73
        good_points = [good_points ; i];
    end
end

PI_lim = max(PI(good_points), (2/(gamma_air+1))^(gamma_air/(gamma_air-1)));
Psi = sqrt(2*gamma_air/(gamma_air-1)*(PI_lim.^(2/gamma_air)-PI_lim.^((gamma_air+1)/gamma_air)));

T_bef_thr_mean = mean(T_bef_thr);

B = (m_dot_at(good_points).*sqrt(R_air*T_bef_thr_mean))./(p_bef_thr(good_points).*Psi);

A = [ones(length(good_points),1), alpha(good_points), alpha(good_points).^2];

A_eff_const = inv(A'*A)*A'*B;

m_dot_at_model = (p_bef_thr(good_points).*( A_eff_const(1) + A_eff_const(2).*alpha(good_points) + A_eff_const(3).*alpha(good_points).^2).*Psi)./(sqrt(R_air*T_bef_thr_mean));

if doPlot  %Here doPlot is used, avoids the plot if it is set to 0
    figure(1),clf
    plot(m_dot_at_model,'Color','r','Linestyle','--')
    ylim([0, max(m_dot_at_model)]);
    hold on;
    plot(m_dot_at(good_points),'b')
    grid on
    xlabel('Sample')
    ylabel('Air mass flow [kg/s]')
    legend('Modelled','Measured')    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Computations of model Intake manifold    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

numerator = m_dot_at*R_air.*T_im.*n_r;
denominator = p_im*V_d*n_cyl.*N;
B = numerator./denominator;

A = [ones(237,1), sqrt(p_im), sqrt(N)];

Intake_mainfold_const = inv(A'*A)*A'*B;

eta_vol_model = Intake_mainfold_const(1) + Intake_mainfold_const(2).*sqrt(p_im) + Intake_mainfold_const(3).*sqrt(N);
m_dot_ac_model = (eta_vol_model.*p_im.*V_d.*n_cyl.*N)./(R_air*T_im*n_r);

if doPlot  %Here doPlot is used, avoids the plot if it is set to 0
    figure(1),clf
    plot(m_dot_ac_model,'r')
    hold on;
    plot(m_dot_at,'b')
    grid on
    xlabel('Sample')
    ylabel('Intake manifold preasure [Pa]')
    legend('Modelled','Measured')    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Computations of model Fuel injector    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

numerator = m_dot_at.*n_r;
denominator = lambda*AFs.*N*n_cyl;
B = numerator./denominator;

A = [t_inj, -ones(237,1)];

Fuel_injection_const = inv(A'*A)*A'*B;
Fuel_injection_const(2) = Fuel_injection_const(2)/Fuel_injection_const(1);
Fuel_injection_const;

m_dot_fi_model = (N*n_cyl*Fuel_injection_const(1).*(t_inj-Fuel_injection_const(2)))/n_r;

m_dot_fi = m_dot_at./(lambda.*AFs);

if doPlot  %Here doPlot is used, avoids the plot if it is set to 0
    figure(1),clf
    plot(m_dot_fi_model,'r')
    hold on;
    plot(m_dot_fi,'b')
    grid on
    xlabel('Sample')
    ylabel('Fuel mass flow rate [kg/s]')
    legend('Modelled','Measured')    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Computations of model Cylinder, Moment    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

FrictionWork = 2*pi*n_r*EngineFriction.M_f;

B = FrictionWork./V_D;
A = [ones(length(EngineFriction.N),1), 60*EngineFriction.N/1000 , (60*EngineFriction.N/1000).^2];

FMEP_const = inv(A'*A)*A'*B;

FrictionWork_model = V_D.*(FMEP_const(1) + FMEP_const(2).*(60*EngineFriction.N/1000) + FMEP_const(3).*(60*EngineFriction.N/1000).^2);

if doPlot  %Here doPlot is used, avoids the plot if it is set to 0
    figure(1),clf
    plot(FrictionWork_model,'Color','r','Linestyle','--')
    hold on;
    plot(FrictionWork,'b')
    grid on
    xlabel('Sample')
    ylabel('Friction work [Nm]')
    legend('Modelled','Measured')    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Computations of model Cylinder, Temperature    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

B = T_em;
m_dot_fi = m_dot_at./(lambda*AFs);
A = [ones(237,1), sqrt(m_dot_at+m_dot_fi)];

Temp_const = inv(A'*A)*A'*B;

T_em_model = Temp_const(1) + Temp_const(2)*sqrt(m_dot_at+m_dot_fi);

if doPlot  %Here doPlot is used, avoids the plot if it is set to 0
    figure(1),clf
    plot(T_em_model,'Color','r','Linestyle','--')
    hold on;
    plot(T_em,'b')
    grid on
    xlabel('Sample')
    ylabel('Temperature [K]')
    legend('Modelled','Measured')    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Computations of model Exhaust flow    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

m_dot_fi = m_dot_at./(lambda*AFs);
B = m_dot_at + m_dot_fi;

A = sqrt(p_es.*(p_es - min(p_es))./(R_exh*T_es));

Exhaust_flow_const = inv(A'*A)*A'*B;

m_dot_exh_model = Exhaust_flow_const*sqrt(p_es.*(p_es - min(p_es))./(R_exh*T_es));

m_dot_exh = m_dot_at + m_dot_fi;

if doPlot  %Here doPlot is used, avoids the plot if it is set to 0
    figure(1),clf
    plot(m_dot_exh_model,'Color','r','Linestyle','--')
    hold on;
    plot(m_dot_exh,'b')
    grid on
    xlabel('Sample')
    ylabel('Mass flow exhaust [kg/s]')
    legend('Modelled','Measured')    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Computations of model Lambda sensor    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if doPlot  %Here doPlot is used, avoids the plot if it is set to 0
    figure(1),clf
    plot(fuelstep.time,fuelstep.lambda,'r')
    hold on;
    plot(fuelstep.time,fuelstep.t_inj,'b')
    grid on
    xlabel('Time [s]')
    ylabel('Lambda sensor [%]')
    legend('Lambda','Injection time')    
end

% lambda_th is determined to be ??? s with the given data (time to reach the
% 63% of the final step value)
tau_d = 0.069; %(delay)
tau_mix = 0.085; %(dynamic);    %[s]            Determined lambda time constant

% print the value
disp(' ')
disp('Lambda:')
disp(['tau_d = ' num2str(tau_d) ' [s]'])
disp(['tau_mix = ' num2str(tau_mix) ' [s]'])

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Computations of model lambda feed forward    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lambda_feedforward_const = (t_inj-Fuel_injection_const(2)).*N./m_dot_fi\ones(237,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Computations of model Catalyst light off    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load('Mätdata\lightOff.mat')

if doPlot  %Here doPlot is used, avoids the plot if it is set to 0
    figure(1),clf
    plot(lightOff.time,lightOff.lambda_bc_disc,'r')
    hold on;
    plot(lightOff.time,lightOff.lambda_ac_disc,'b')
    grid on
    xlabel('Time [s]')
    ylabel('Lambda sensor [V]')
    legend('Lambda before catalyst','Lambda after catalyst')    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% <<<< Until here is for project 1b / Next lines need to be
%%%%%%%%%%%%% <<<< uncommented for project 1c
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize I/O abstraction layer %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % For more information about the I / O block look under Project1c/ECU / "I / O abstraction layer"
% % This can be used to do special tests with manually selected parameters:
% % Engine speed, throttle reference, wastegate (project2) and pedal position.
% % When all {property}_manual = 0, this block is disconnected by default.
N_e_manual = 0; N_e_step = 1; NINI = 2000; NEND = NINI;  NeST=30; NeSlope = 1; NeStartTime = 60; NeRampInit = 800;
alpha_REF_manual = 0; alphaINI = 0.0; alphaEND = alphaINI; alphaST=30;
wg_REF_manual = 0; wgINI = 100; wgEND = wgINI; wgST=30;
pedPos_manual = 0; pedINI = 0.2; pedEND = 1.0; pedST=30;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Calculate rolling and air resistance %% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
F = [0 410 1050]';                  % [N] Force vector
v = [0 80 160]';                    % [km/h] Speed vector
rho_air=p_amb/(R_air*T_amb);        % [kg/m^3] Air density
A=[VehicleMass*ones(size(v)) VehicleMass*v/3.6 0.5*VehicleArea*rho_air*(v/3.6).^2];
x=A\F;
% Rolling resistance coeficcients
c_r1=x(1);
c_r2=x(2);
%Air drag coeficcient
C_d=x(3);

%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Load cycle Data     %%
%%%%%%%%%%%%%%%%%%%%%%%%%%
init_drivecycle;
% 
% % Loaded data for the European driving cycle:
% % Driving Scenario: Matrix from which the following information is extracted
% % S: Reference Speed [km/h]
% % U: Clutch position [-]
% % G: Selected gear [0-5]
% % T: Time Vector [s]
% 
% % Forming vectors for the driver model
Clutch = [T U];
Gear   = [T max(G,1)];
Speed  = [T S];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ---------------------------------------------- %
% Aftertreatment when the simulation is complete %
% ---------------------------------------------- %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%     Light-Off computation    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% lightOff =

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%      Compute emissions       %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Inputs to calcEmissions
%% tout: Time Vector from Simulink
%% lambda: Continuous lambda
%% Distance: Distance traveled in meters
%% dmacAct: Mass air flow to the cylinder in kg / s
%% dmfAct: Fuel flow to the cylinder in kg / s
%% lightoff: Time in seconds until the light-Off

% calcEmissions(tout, lambda, Distance, dmacAct, dmfcAct, lightOff);

% Calculate fuel consumption
% fuelCons = 
  
% disp(sprintf('Fuel Consumption: %1.2f [l/(10 mil)]',fuelCons))
