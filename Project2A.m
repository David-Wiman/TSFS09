%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Student code for project2A   %%%
%%% TSFS09 - Fordonssystem       %%%
%%% Vaheed Nezhadali 2015-10-22  %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc

figNr=1;
figpath='Figures/';

doPlot=0;
doExpFig=0;
doSimulinkFig=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%      Sätter upp turbomotordata med antaganden     %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Läs in modellparametrar från Projekt1 för att få basmotorn alla parametrar

%clear all; close all;
load turboMap
load EnginemapTSFS09

% 01 = af (Compressor inlet)
% 02 = im
% 03 = em
% 04 = es

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Ladda ny mätdata och parametrar %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

NcCorr = comp.NcCorr;
m_dot_cCorr = comp.m_dot_cCorr;
p01    = comp.p01;
T01    = comp.T01;
pCref  = comp.pCref;
TCref  = comp.TCref;
PiC    = comp.PiC;
etaC   = comp.etaC;
etaCmin=min(etaC/2); %will be used in Simulink models

p04  = turb.p04;
T03  = turb.T03;
TFP  = turb.TFP;
PiT  = turb.PiT;
TSP  = turb.TSP;
etaT = turb.etaT;
p03  = p04./PiT;
etaTmin=min(turb.etaT)/2; %will be used in Simulink models

% Compressor och Turbindiameter
dComp = 56e-3;
rComp = dComp/2;
dTurb = 52.2e-3;
rTurb = dTurb/2;
cp_exh = 1.2133e+03;            % [J/(kg*K)]
gamma_exh = 1.3;                % [J/(kg*K)]
cp_air =  980.0000;
gamma_air = 1.4;

%Beräknade storheter
Nc  = NcCorr*sqrt(T01/TCref);
Nt  = TSP.*sqrt(T03);
m_dot_c  = m_dot_cCorr*(p01/pCref)/(sqrt(T01/TCref));
Uc2 = rComp*Nc*2*pi/60;  % Speed of the compressor blades in [m/s] if Nc is in [RPM] and rComp is in [m]
Ut2 = rTurb*Nt*2*pi/60;
m_dot_t  = TFP.*p03*1e-3./sqrt(T03);
BSR = Ut2 ./ sqrt( 2*cp_exh*T03*( 1-PiT.^((gamma_exh-1)/gamma_exh) ) );

% create matrices where every column represents the data from same turbo
% speed
m_dot_cCorr_M = reshape([m_dot_cCorr ; NaN],7,5);
PiC_M         = reshape([PiC ; NaN],7,5);
NcCorr_M      = reshape([NcCorr ; NaN],7,5);
etaC_M        = reshape([etaC ; NaN],7,5);
Nc_M          = reshape([Nc ; NaN],7,5);
m_dot_c_M     = reshape([m_dot_c ; NaN],7,5);
Uc2_M         = reshape([Uc2 ; NaN],7,5);

TSP_M  = reshape(TSP,6,5);
TFP_M  = reshape(TFP,6,5);
PiT_M  = reshape(PiT,6,5);
etaT_M = reshape(etaT,6,5);
BSR_M  = reshape(BSR(:,1),6,5);
m_dot_t_M   = reshape(m_dot_t,6,5);
Nt_M   = reshape(Nt,6,5);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Compressor modell    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% COMPRESSOR Mass flow model

% Initial data
% a = [Psi_max, m_dot_cCorr_max]
a_0 = [1.0016    0.1805];
xdata = [Uc2, PiC];
ydata = m_dot_cCorr;

% Nonlinear least squares fit
f = @(a, xdata)( a(2)*sqrt( 1-( xdata(:,2)./(((xdata(:,1).^2)./(2*cp_air*T01))*a(1) + 1).^( gamma_air/(gamma_air-1)) ).^2 ) );
k = lsqcurvefit(f, a_0, xdata, ydata);
Psi_max = k(1);
m_dot_cCorr_max = k(2);

if doPlot
% Plot
figure(1)
subplot(2,2,1)
times = linspace( Uc2(1), Uc2(end), 34 )';
plot(m_dot_cCorr_M(:,1), PiC_M(:,1), 'bo-', 'DisplayName', 'Compressor map');
hold on
grid on
set(gca,'GridLineStyle','--')
plot(m_dot_cCorr_M(:,2:5), PiC_M(:,2:5), 'bo-','HandleVisibility','off');
xlabel('Corrected compressor mass flow [kg/s]')
ylabel('Pressure ratio over compressor [-]')
xlim([0,0.2])
ylim([1,3.5])
model_data = f(k,xdata);
plot(model_data(1:7), PiC_M(:,1), 'rs--', 'DisplayName', 'Fitted model');
plot(model_data(8:14), PiC_M(:,2), 'rs--','HandleVisibility','off')
plot(model_data(15:21), PiC_M(:,3), 'rs--','HandleVisibility','off')
plot(model_data(22:28), PiC_M(:,4), 'rs--','HandleVisibility','off')
plot([model_data(29:end) ; NaN], PiC_M(:,5), 'rs--','HandleVisibility','off')
legend show
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% COMPRESSOR efficiency model

% Initial data
% a = [eta_c_max, m_dot_c_Corr_at, PiC_at, Q11, Q12, Q22]
a_0 = [0.8206    0.0842    1.9778   90.5045   -6.4116    0.8223];
xdata = [m_dot_cCorr, PiC];
ydata = etaC;

% Nonlinear least squares fit
f = @(a, xdata)( a(1) - a(4)*(xdata(:,1) - a(2)).^2 - 2*a(5)*(xdata(:,1) - a(2)).*(sqrt(xdata(:,2)-1) - a(3)+1) - a(6)*(sqrt(xdata(:,2)-1) - a(3)+1).^2 );
k = lsqcurvefit(f, a_0, xdata, ydata);
eta_c_max = k(1);
m_dot_c_Corr_at = k(2);
PiC_at = k(3);
Q11 = k(4);
Q12 = k(5);
Q22 = k(6);

if doPlot
% Plot
subplot(2,2,2)
times = linspace( PiC(1), PiC(end), 34 )';
plot(m_dot_cCorr_M(:,1), etaC_M(:,1), 'bo-', 'DisplayName', 'Compressor map');
hold on
grid on
set(gca,'GridLineStyle','--')
plot(m_dot_cCorr_M(:,2:5), etaC_M(:,2:5), 'bo-','HandleVisibility','off');
xlabel('Corrected compressor mass flow [kg/s]')
ylabel('Compressor efficiency [-]')
xlim([0,0.2])
ylim([0.55,0.9])
model_data = f(k,xdata);
plot(m_dot_cCorr_M(:,1), model_data(1:7), 'rs--', 'DisplayName', 'Fitted model');
plot(m_dot_cCorr_M(:,2), model_data(8:14), 'rs--','HandleVisibility','off')
plot(m_dot_cCorr_M(:,3), model_data(15:21), 'rs--','HandleVisibility','off')
plot(m_dot_cCorr_M(:,4), model_data(22:28), 'rs--','HandleVisibility','off')
plot(m_dot_cCorr_M(:,5), [model_data(29:end) ; NaN], 'rs--','HandleVisibility','off')
legend show
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Turbin modell    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TURBIN Mass flow model

% Initial data
% a = [k_0, k_1]
a_0 = [0.0054, 1.4504];
xdata = PiT;
ydata = TFP;

% Nonlinear least squares fit
f = @(a, xdata)( a(1).*sqrt(1-xdata.^a(2)) );
k = lsqcurvefit(f, a_0, xdata, ydata);
k_0 = k(1);
k_1 = k(2);

if doPlot
% Plot
subplot(2,2,3)
times = linspace( PiT(1), PiT(end), 30 )';
plot(1./PiT_M(:,1), TFP_M(:,1), 'bo-', 'DisplayName', 'Turbine map');
hold on
grid on
set(gca,'GridLineStyle','--')
plot(1./PiT_M(:,2:5), TFP_M(:,2:5), 'bo-','HandleVisibility','off');
xlabel('Inverse of pressure ratio over turbine [-]')
ylabel('TFP [kg/s K^0^.^5 /kPa]')
xlim([1, 3.5])
ylim([1.5e-3, 5e-3])
model_data = f(k,xdata);
plot(1./PiT_M(:,1), model_data(1:6), 'rs--', 'DisplayName', 'Fitted model');
plot(1./PiT_M(:,2), model_data(7:12), 'rs--','HandleVisibility','off')
plot(1./PiT_M(:,3), model_data(13:18), 'rs--','HandleVisibility','off')
plot(1./PiT_M(:,4), model_data(19:24), 'rs--','HandleVisibility','off')
plot(1./PiT_M(:,5), model_data(25:end), 'rs--','HandleVisibility','off')
legend('Location','southeast')
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% TURBIN efficiency model

% Initial data
% a = [eta_t_max, BSR_max]
a_0 = [0.8073, 0.6790];
xdata = BSR;
ydata = etaT;

% Nonlinear least squares fit
f = @(a, xdata)( a(1)*(1-((xdata-a(2))/a(2)).^2 ) );
k = lsqcurvefit(f, a_0, xdata, ydata);
eta_t_max = k(1);
BSR_max = k(2);

if doPlot
% Plot
subplot(2,2,4)
times = linspace( BSR(1), BSR(end), 30 )';
plot(BSR_M(:,1), etaT_M(:,1), 'bo-', 'DisplayName', 'Turbine map');
hold on
grid on
set(gca,'GridLineStyle','--')
plot(BSR_M(:,2:5), etaT_M(:,2:5), 'bo-','HandleVisibility','off');
xlabel('BSR [-]')
ylabel('Turbine efficiency [-]')
xlim([0.55,0.8])
ylim([0.77,0.83])
model_data = f(k,xdata);
plot(BSR_M(:,1), model_data(1:6), 'rs--', 'DisplayName', 'Fitted model');
plot(BSR_M(:,2), model_data(7:12), 'rs--','HandleVisibility','off')
plot(BSR_M(:,3), model_data(13:18), 'rs--','HandleVisibility','off')
plot(BSR_M(:,4), model_data(19:24), 'rs--','HandleVisibility','off')
plot(BSR_M(:,5), model_data(25:end), 'rs--','HandleVisibility','off')
legend show
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% BMEP model    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

M_e = EngineMap.M_e;
n_r = EngineMap.engine.n_r;
a = EngineMap.engine.a;
B = EngineMap.engine.B;
V_d = 2*a*pi*B^2/4;  
n_cyl = EngineMap.engine.n_cyl;
p_im = EngineMap.p_im;

BMEP = (2*pi*M_e*n_r) ./ (V_d*n_cyl);

B = BMEP;
A = [-ones(237,1) p_im];
k = A\B;
C_P0 = k(1);
C_P1 = k(2);
BMEP_model = -C_P0 + C_P1*p_im;

if doPlot
% Plot
figure(5)
times = linspace( BMEP(1), BMEP(end), 237 )';
plot(p_im/1000, BMEP/1000, 'bo-', 'DisplayName', 'Measured BMEP');
hold on
grid on
set(gca,'GridLineStyle','--')
xlabel('Intake manifold pressure [kPa]')
ylabel('BMEP [kPa]')
plot(p_im/1000, BMEP_model/1000, 'rs--', 'DisplayName', 'BMEP model');
legend('Location','southeast')
end