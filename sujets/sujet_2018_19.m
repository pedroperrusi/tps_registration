clear all; close all;
%% Exercice 2
mark_obj = [0 60 50 0 30;
            0 0  50 50 30;
            0 0 0 0 30];
im_entry = [210 270 -250]';
im_target = [310 350 -210]';
im_mark = [350 434 397.5 309.5 403;
           170 255.5 282 210.5 203;
           -220 -220.5 -179.5 -179 -178.5];
% Intrinsics parameters
px_spacing = [0.5 0.5];
sz = [512 512];
center = [sz/2 0];
% Intrinsics Matrix
alpha = [1./px_spacing 20/20];
K = [diag(alpha) center';
     0 0 0 1];

% On passe  dans le repere IRM
iK = inv(K);
m_entry     = h_unpack(iK*h_pack(im_entry));
m_target    = h_unpack(iK*h_pack(im_target));
m_mark      = h_unpack(iK*h_pack(im_mark));

%% Positionnement de l'instrument
dir_z = (m_target-m_entry)/norm(m_target-m_entry);
% on deplace l'aguille a 20 mm du point d'entree vers l'exterieur
m_aig = m_entry - 20*dir_z;
% on choisi des orientations arbitraires pour le plan x et y
dir_x = [1 0 0]';
% assure l'orthonormalite
dir_y = -cross(dir_x, dir_z)/norm(cross(dir_x, dir_z));
dir_x = -cross(dir_z, dir_y)/norm(cross(dir_z, dir_y));
aigPose = [dir_x dir_y dir_z m_aig;
           zeros(1,3) 1];
       
%% Calcul de la pose du marqueur
[ T, R, t, reproj_error ] = horn(m_mark, mark_obj);