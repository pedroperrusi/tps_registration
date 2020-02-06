%% Question 3 
% Recalage avec syst`eme de localisation
clear all; close all;
%% Setup
InitConfig 
flag_draw = false;
flag_noise = false;
cam_idx = 1;
instr_idx = 2;
n_measures = 3; % >= 3
%% On estime la transformation marqueur instrument par deux mesures
% Ax = xB
% Teff1->eff2 x = x Tm1->m2
% Une information pour chaque position
offset_position = GetRobotCurrentPosition(flag_noise);
offset_position = offset_position(1:3, 4);
sig_position = [50 50 100]'; % std deviation of position
positions = zeros(3, 1, n_measures);
transf_base_effector = zeros(4, 4, n_measures);
mesures = zeros(4, 4, n_measures);
% Equations pour chaque pair
n_pairs = n_measures*(n_measures-1)/2; % combination of n elements, 2x2
xRa = zeros(3,3,n_pairs); 
xta = zeros(3,1,n_pairs);
xua = zeros(n_pairs,3);
xRb = zeros(3,3,n_pairs); 
xtb = zeros(3,1,n_pairs);
xub = zeros(n_pairs,3);
% boucle de mesures de calibration
for ii = 1 : n_measures
    positions(:,:,ii) = offset_position + sig_position.*randn(3,1);
    MoveEffPosition(positions(:,:,ii));
    % A: Teff1->eff2
    transf_base_effector(:, :, ii) = GetRobotCurrentPosition(flag_noise);
    % B: Tm1->m2
    measure = GetLocalizerInformation(flag_noise);
    mesures(:,:,ii) = measure.mark(instr_idx).T;
    if flag_draw
        DisplayConfig;
        pause;
    end
end
% boucle de creation de pairs pour l'equation Ax = xB
ip = 0; % pairs counter
for ii = 1 : n_measures % population counter
    for jj = ii+1 : n_measures
        ip = ip + 1;
        % Robot (a)
        Teffj_effi = inv(transf_base_effector(:,:,jj))*transf_base_effector(:,:,ii); 
        xRa(:,:,ip) = Teffj_effi(1:3,1:3);
        xta(:,:,ip) = Teffj_effi(1:3,4);
        [~, xua(ip,:)] = r2thetau(xRa(:,:,ip)); % axis de rotation
        % Markers (b)
        Tmj_mi = inv(mesures(:,:,jj))*mesures(:,:,ii); 
        xRb(:,:,ip) = Tmj_mi(1:3,1:3);
        xtb(:,:,ip) = Tmj_mi(1:3,4);
        [~, xub(ip,:)] = r2thetau(xRb(:,:,ip)); % axis de rotation
    end
end
%% Calcul de la transformation
% On retrouve la rotation optimale
M = xua'*xub;
[U, ~, V] = svd(M);
xR = U*V';
% in case of reflection
if det(xR) < 0
    xR = U*[1 0 0; 0 1 0; 0 0 -1]*V';
    disp('Warning: transformation is a reflection');
end
% On retrouve les translations
G = zeros(3*n_pairs, 3);
y = zeros(3*n_pairs, 1);
ip = 1; % position index
for ii = 1:n_pairs
    % Vertical concatenation
    G(ip:ip+2, :) = xRa(:,:,ii) - eye(3);
    y(ip:ip+2) = xR*xtb(:,:,ii) - xta(:,:,ii);
    ip = ip + 3;
end
xt = pinv(G)*y;
disp('Effector to instrument marker calibration done')
T_eff_instr_mark = [xR xt; 0 0 0 1];

%% On est capable de retrouver le profondeur de la cieble par triangulation
K = [400 0 380;
     0 400 285;
     0   0  1];
iK = inv(K);
n_measures = 2;
targets_img = zeros(3, n_measures);
targets_cam = zeros(3, n_measures);
targets_img(:,1) = iK*h_pack(GetTargetPosition(flag_noise));
loc{1} = GetLocalizerInformation(flag_noise);
% move camera to get a new perspective
trans = [10; 15; 30];
angle = 0;
MoveCamera(trans, angle);
targets_img(:,2) = iK*h_pack(GetTargetPosition(flag_noise));   
loc{2} = GetLocalizerInformation(flag_noise);
% get camera rigid transformation
T12 = Tcam_mark_cam*inv(loc{1}.mark(cam_idx).T)*loc{2}.mark(cam_idx).T*inv(Tcam_mark_cam);
[targets_cam(:,1), targets_cam(:,2), err2d] = multi_view_triangulation(T12, targets_img(:,1), targets_img(:,2));
disp('2D triangulation error')
disp(err2d)

% Validate triangulation
iTcam_mark_cam = inv(Tcam_mark_cam);
Tloc_cam_mark{1} = loc{1}.mark(cam_idx).T;
Tloc_cam{1} = Tloc_cam_mark{1}*iTcam_mark_cam;
target_loc{1} = h_unpack(Tloc_cam{1}*h_pack(targets_cam(:,1)));

Tloc_cam_mark{2} = loc{2}.mark(cam_idx).T;
Tloc_cam{2} = Tloc_cam_mark{2}*iTcam_mark_cam;
target_loc{2} = h_unpack(Tloc_cam{2}*h_pack(targets_cam(:,2)));
disp('3D triangulation error')
disp(norm(target_loc{1} - target_loc{2}))


%% On estime la position des trocarts
% a partir de quelques mouvements du robot on peut deduire le position des
% trocarts. Ils sont l'intersection entre tous les droites
% effecteur-instrument.

% On a besoin de ce information une fois que l'orientation de l'effecteur
% terminal n'est pas controle - c'est une joint passive.

% Par triangulation
% On a besoin de ce information une fois que l'orientation de l'effecteur
% terminal n'est pas controle - c'est une joint passive.
get_translation = @(Th) h_unpack(Th*[0 0 0 1]');
get_orientation = @(p1, p2) (p2-p1)./norm(p2-p1);
% On peut s'utiliser des memes mesures obtenues pour le hand eye calibration
eff_position = reshape(positions(:,:,1:2), 3, 2);
instr_position(:,1) = get_translation(transf_base_effector(:,:,1)*Teff_inst);
d_instr(:,1) = get_orientation(eff_position(:,1), instr_position(:,1));
instr_position(:,2) = get_translation(transf_base_effector(:,:,2)*Teff_inst);

d_instr(:,2) = get_orientation(eff_position(:,2), instr_position(:,2));
% On assemble le systeme lineaire
% Trocard = eff1 - x(1)*d_instr1 = eff2 + x(2)*d_instr2
% [d_instr1 d_instr2]*x = [eff1; eff2]
A = [d_instr(:, 1) -d_instr(:, 2)];
b =  eff_position(:,2) - eff_position(:,1);
x = A\b;
trocard_robot = eff_position(:,2) + x(2)*d_instr(:,2);

disp('Trocar triangulation error')
erreur = norm(eff_position(:,1) + x(1)*d_instr(:,1) - (eff_position(:,2) + x(2)*d_instr(:,2)));
disp(erreur)
%% On retrouve la transformation robot -> camera
target = targets_cam(:,end);
loc = GetLocalizerInformation(flag_noise);
Trobot_effector = GetRobotCurrentPosition(flag_noise);
iTloc_instr_mark = inv(loc.mark(instr_idx).T);
Tloc_cam_mark = loc.mark(cam_idx).T;
iTcam_mark_cam = inv(Tcam_mark_cam);
% On calcule l'ensemble de transformations
Trobot_loc = Trobot_effector*T_eff_instr_mark*iTloc_instr_mark;
Tloc_cam = Tloc_cam_mark*iTcam_mark_cam;
Trobot_cam = Trobot_loc*Tloc_cam;

%% On donne une cieble example
target_robot = h_unpack(Trobot_cam*h_pack(target));
% On retrouve la direction de translation
dir = get_orientation(target_robot, trocard_robot);
decalage = h_unpack(Teff_inst(3,4)*h_pack(dir));
% On applique la transformation entre effecteur et l'instrument
command = decalage + target_robot;
MoveEffPosition(command)
DisplayConfig