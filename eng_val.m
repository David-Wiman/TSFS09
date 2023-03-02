%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Labskelett för Projekt1a,c TSFS09  %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% $Revision: 1.1 $
% $Date: 2014/06/02 $

%% Init
clear all
clc
close all


show1D = 0;     %type 1 for 1 dimension plots
show2D = 0;     %type 1 for 2 dimension plots
show3D = 1;     %type 1 for 3 dimension plots


%% Description
% Case 1 - One Dimension Model
% Assume that:
% A1,D1 are measured data
% You want a model for D1 as a function of A1
% D1mod is the output of your D1-model, and you want to validate this model

% Case 2 - Two Dimension Model
% Assume that:
% A2,B2,D2 are measured data
% You want a model for D2 as a function of A2,B2
% D2mod is the output of your D2-model, and you want to validate this model

% Case 3 - Higher Dimension Model
% Assume that:
% A3,B3,C3,D3 are measured data
% You want a model for D3 as a function of A3,B3,C3
% D3mod is the output of your D3-model, and you want to validate this model


%% Data to validate
% This creates some nonsense data, do not look to much at this
a = 0:.25:2; b = 3:-.5:1; c = [3,-2,2,1];
A1 = fliplr(a);
D1mod = 7*A1-12; 
w = rand(size(D1mod)).*rand(size(D1mod)); D1=D1mod+2*w-mean(w(:))+.1;

[B2,A2] = meshgrid(b,a);
D2mod = A2.^2-B2.^2-11;
w = rand(size(D2mod)).*rand(size(D2mod)); D2=D2mod+5*w-7*mean(w(:))+1*A2;

[A3,B3,C3] = meshgrid(a,b,c);
A3=A3(:); B3=B3(:); C3=C3(:);
D3mod = A3+.1*(A3.*B3)-B3-C3.^2+C3;
w = rand(size(D3mod)).*rand(size(D3mod)); D3=D3mod-3*w+2*mean(w(:))-.5*C3;


%% 1D Figures
if show1D
figure(11); clf; hold on
plot(D1,'ro')
plot(D1mod,'b*')
legend('D','Dmod','location','NorthWest')

figure(12); clf; hold on
[ds,do] = sort(D1);
plot(ds,'k+')
plot(D1mod(do),'g*')
legend('D1','D1mod','location','NorthWest')

figure(13); clf; hold on
plot(A1,D1,'ro')
plot(A1,D1mod,'k-')
grid on
xlabel('b')

figure(14); clf; hold on
plot(A1,D1,'ro')
plot(A1,D1mod,'k-')
legend('D','Dmod','location','NorthWest')

figure(15); clf; hold on
plot(A1,zeros(size(A1)),'k')
plot(A1,D1-D1mod,'r*')
title('absolute error')

end

%% 2D Figures
if show2D
figure(21); clf; hold on
plot(D2,'ro')
plot(D2mod,'k*')
legend('D2','D2mod','location','NorthWest')

figure(22); clf; hold on
plot(D2(:)-D2mod(:),'k')
title('absolute error')

figure(23); clf; hold on
plot3(A2,B2,D2,'ko')
plot3(A2,B2,D2mod,'k*')
title('model and measurement')

figure(24); clf; hold on
mesh(D2)
mesh(D2mod)
view(-45,30)
title('model and measurement')

figure(25); clf; hold on
mesh((D2-D2mod)./D2)
view(-45,45)
title('relative error')

figure(26); clf; hold on
subplot(1,2,1); mesh(A2,B2,D2)
subplot(1,2,2); mesh(A2,B2,D2mod)

end
%% 3D Figures
if show3D
figure(31); clf; hold on
plot(D3,'r+')
plot(D3mod,'b*')
legend('D3','D3mod','location','NorthWest')

figure(32); clf; hold on
[ds,do] = sort(D3);
plot(ds,'r+')
plot(D3mod(do),'b*')
legend('D3','D3mod','location','NorthWest')

figure(33); clf; hold on
l3 = length(D3);
plot((D3-D3mod)./D3,'k+')
plot(1:l3,zeros(1,l3),'m:')
legend('relative error','location','NorthWest')

figure(34); clf; hold on
l3 = length(D3);
plot(C3,D3-D3mod,'k+')
plot([min(C3),max(C3)],[0,0],'m:')
legend('absolute error','location','NorthWest')
end