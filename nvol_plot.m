function nvol_plot(inpr,rps,nvol,fignmbr);
%
%
%          3D-diagram över fyllnadsgrad
%
%          funktion nvol_plot(inlpr,rps,nvol,fignmbr)
%
%               inpr    = tryck i insugningsröret [Pa]
%               rps     = varv per sekund [rps]
%               nvol    = fyllnadsgrad 
%               fignmbr = figurnummer
%
%
%          Funktionen skapar en 3D-plot över fyllnadsgrad som 
%          funktion av insugningstryck och varvtal. Plotten ritas 
%          i den figur som anges i 'fignmbr'. 
%
%          (Uppdaterad 2002-09-24)
%        
%
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Skriven av YN    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
% Skapar ett rutnät, och interpolerar sedan fram en 
% tredimensionell yta ovanpå detta
  
  pManVect = (min(inpr):(max(inpr)-min(inpr))/25:max(inpr))/1e3;
  NVect    = min(rps):(max(rps)-min(rps))/40:max(rps);
  [NVecti, pManVecti] = meshgrid(NVect, pManVect);
  
  Z = griddata(inpr/1e3,rps,nvol,pManVecti,NVecti);
  
  % Plottar ett musseldiagram
  
  if nargin < 4
  fignmbr = 8;
  end
  figure(fignmbr); clf;
  mesh(pManVecti,60*NVecti,Z);
  xlabel('Insugningstryck [kPa]');
  ylabel('Varvtal [rpm]');
  zlabel('\eta_{vol}');
  title('Fyllnadsgrad');
  %view(-60,25);
  


