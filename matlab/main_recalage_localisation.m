%% Question 3 
% Recalage avec syst`eme de localisation
clear all; close all;
%% Setup
InitConfig 

%% On estime la transformation marqueur instrument par deux mesures
% Ax = xB
% Teff1->eff2 x = x Tm1->m2
flag_noise = false;
marker_idx = 2;
n_iterations = 10;
xRa = zeros(3,3,n_iterations); 
xta = zeros(3,1,n_iterations);
xRb = zeros(3,3,n_iterations); 
xtb = zeros(3,1,n_iterations);
positions = zeros(3,1,n_iterations);
% boucle de mesures de calibration
for ii = 1 : n_iterations
    positions(:,:,ii) = randn(3,1);
    MoveEffPosition(positions(:,:,ii));
    % A: Teff1->eff2
    transf_base_eff = GetRobotCurrentPosition(flag_noise);
    xRa(:,:,ii) = transf_base_eff(1:3,1:3);
    xta(:,:,ii) = transf_base_eff(1:3,4);
    % B: Tm1->m2
    mesure = GetLocalizerInformation(flag_noise);
    % Separe sa rotation et translation
    xRb(:,:,ii) = mesure.mark(marker_idx).T(1:3,1:3);
    xtb(:,:,ii) = mesure.mark(marker_idx).T(1:3,4);
end