%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Template file for Project 1b      %%%
%%% TSFS09 - Modeling and Control of  %%%
%%%          Engines and Drivelines   %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
% This file is a template suitable for using in the first part of 
% Project 1b - the Analysis part. By writing commands heare instead
% of in the matlab prompt the calculations are easier to reproduce
% and change in case of error. To help you get started a few commands
% needed for Project 1 is already included.
%
%
% IMPORTANT: Make sure that you use SI-units for all calculations! 
% Other units can be better for plots to make them more easy to 
% interpret, for example its usally easier to have degrees in plots
% instead or radians.
%
%
% Don't forget to check that your results are reasonable!
% Something that is easy to control is for example the cylinder volume.
% 
%
%  Glöm inte att kontrollera att de resultat ni får är rimliga! Något som 
%  är ganska lätt att kontrollera är till exempel cylindervolymen.
%  Överensstämmer den slagvolym ni får med den slagvolym som anges i
%  laborationskompendiet och är kompressionsförhållandet rätt? 
%
%  (File uppdated 2018-09-25 by Robin Holmbom)
%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         
%%          EXERCISE 1           %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc

load('Mätdata\pCylHigh.mat')  % Load data file
load('Mätdata\pCylLow.mat')

pcyl   = pCylHigh.cyl_press;        % Cylinder pressure [Pa]   
M      = pCylHigh.M_b;        % Engine torque [Nm]
N      = pCylHigh.N;        % Engine speed [rpm]
theta  = pCylHigh.theta;        % Crank angle [rad]
r_c    = pCylHigh.r_c;		  % Compression ratio
T_im   = pCylHigh.T_im;        % Intake Manifold Temperature [K]
p_im   = pCylHigh.p_im;		  % Intake Manifold Pressure [Pa]
l      = 0.1438;   % Vevstakens längd [m]
a      = 0.0466;	  % Half the stroke length[m]
B      = 0.0820;  % Cylinder diameter [m]
ncyl   = 4;       % Number of cylinders
VD     = 2*a*(B/2)^2*pi*ncyl;  % Total volume [m³]
Vd     = VD/ncyl;
Vc     = Vd/(r_c-1);
qLHV   = 44e6;    % Lower heating value of fuel, default = 44e6
R      =  298;     % Specific gas sconstant
lambda = 1;       % Normalized air/fuel ratio
gamma  = 1.3;     % Ratio of specific heats
AFs    = 14;      % Stochiometric air/fuel ratio

% Volume per angle
s = a*cos(theta) + sqrt(l.^2 - (a.^2)*sin(theta).^2);
V = Vc + pi*(B/2)^2*(l + a - s);

figure(1); clf;
plot(V*1e3,pcyl*1e-5)
title('pV-diagram (alla cykler, pCylLow)');
xlabel('Volym [dm^3]');
ylabel('Tryck [bar]');

M_f = 15; % Friction moment
Wcalc = trapz(V,pcyl); % Calculate work
Mcalc = (Wcalc/(4*pi))*ncyl; % Calculate instantaneous torque
Mmeas = mean(M); % Calculate mean torque (measured)

disp(['W - beräknat: ', num2str(Wcalc)]); 
disp(['M - beräknat: ', num2str(Mcalc)]);
disp(['M - uppmätt:  ', num2str(Mmeas)]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%          EXERCISE 2           %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
clc

p1 = min(p_im); % First corner
T1 = min(T_im);
V1 = Vd + Vc;

p2 = p1*r_c^gamma;  % Second corner
V2 = Vc;
T2 = ((p2*V2*T1)/(p1*V1));

c1 = p1*V1^gamma;
p_comp = (c1./(V.^gamma));

T3 = max(T_im);
p3=(p2*T3/T2);  % Third corner
V3 = V2;

V4 = V1;    % Forth corner
p4 = p3*(V3/V4)^gamma;

c2 = p3*V3^gamma;
p_exp = (c2./(V.^gamma));

figure(1)
hold on
P1 = plot(V*10^3,p_comp*10^-5,'b');          %isentropic compression
P5 = plot(V*1e3,pcyl*1e-5,'r'); % Measured
P2 = plot([V2 V3]*10^3,[p2 p3]*10^-5,'b');    %constant volume heat addition
P3 = plot(V*10^3,p_exp*10^-5,'b');           %isentropic expansion
P4 = plot([V4 V1]*10^3,[p4,p1]*10^-5,'b');    %constant volume heat rejection

legend('Ideal Otto cycle','Measured Otto cycle')
xlabel('Volume [dm^3]')
ylabel('Pressure [bar]')
title({'PV Diagram';  'Otto cycle'})

trapz(V*10^3,pcyl*10^-5)
trapz(V*10^3,p_exp*10^-5 - p_comp*10^-5)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%          EXERCISE 3           %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc

%%% Load the engine map data
EngineMap = load('Mätdata\EngineMapTSFS09.mat');

m_dot_at = EngineMap.EngineMap.m_dot_at;
M_e = EngineMap.EngineMap.M_e;
N = EngineMap.EngineMap.N;
lambda = EngineMap.EngineMap.lambda;
m_dot_fi = m_dot_at./(lambda.*AFs);

sfc = m_dot_fi./(M_e*2*pi.*N);

sfc_plot(M_e, N, sfc, lambda);


