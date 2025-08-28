%% helicopter linearization and dynamic analysis %%
% 6 DoF linearized rigid body dynamics with quasi-steady rotor flapping 
% requires a steady symmetryc flight condition: null angular rates
% calculates stability derivatives with centered finite differences 
% x = [u w q theta v p r phi psi], u = [dA dB dc dp] -> [cyc_lat cyc_lon coll ped]

% specify simulation model
mdl = "helicopter_sim";

% select mode
mode = 2;

% load the model
load_system(mdl)

% disable euler dynamics
set_param(mdl+"/HELICOPTER MODEL/6DOF RIGID BODY DYNAMICS",'Commented','on')

% disable blocks for real time sim
set_param(mdl+"/GCS gamepad",'Commented','on')
set_param(mdl+"/3D_visualization",'Commented','on')

% simulation time (long enough to reach steady flapping)
time = '2';

% constants
I_mix = Ixx*Izz-Ixz^2;
grav = 9.81;
% ----------------------
function [delta_F,delta_M] = step_input(mdl,time,Delta,x,i) % performs numerical differentiation of 
                                                            % forces and moments w.r.t. x variable

    x0 = evalin('base', x); 
    x1 = x0;
    x1(i) = x1(i) + Delta;
    assignin('base', x, x1);
    SimOut = sim(mdl, 'StopTime', time); 
    Fap_pos = SimOut.Fap(:,:,end);
    Map_pos = SimOut.Map(:,:,end);
    assignin('base', x, x0);

    x1 = x0;
    x1(i) = x1(i) - Delta;
    assignin('base', x, x1);
    SimOut = sim(mdl, 'StopTime', time); 
    Fap_neg = SimOut.Fap(:,:,end);
    Map_neg = SimOut.Map(:,:,end);
    assignin('base', x, x0);

    delta_F = Fap_pos-Fap_neg;
    delta_M = Map_pos-Map_neg;


end
%-----------------------

% start:
disp('%%%%%%%%%%%% STARTING LINEARIZATION PROCEDURE %%%%%%%%%%%%')
%% speed derivatives
disp('calculating speed derivatives..')
if abs(V0(1))<=1
    Delta = 0.1; % [m/s]
else 
    Delta = 0.1*abs(V0(1)); % [m/s] 
end
% u derivatives
[delta_F,delta_M] = step_input(mdl,time,Delta,'V0',1);

Xu_b = (delta_F(1))/(2*Delta*m); 
Yu_b = (delta_F(2))/(2*Delta*m); 
Zu_b = (delta_F(3))/(2*Delta*m); 
Lu_b = Izz/I_mix*(delta_M(1))/(2*Delta)+Ixz/I_mix*(delta_M(3))/(2*Delta); 
Mu_b = (delta_M(2))/(2*Delta*Iyy);
Nu_b = Ixz/I_mix*(delta_M(1))/(2*Delta)+Ixx/I_mix*(delta_M(3))/(2*Delta);

Delta = 0.1; % [m/s]
% v derivatives
[delta_F,delta_M] = step_input(mdl,time,Delta,'V0',2);

Xv_b = (delta_F(1))/(2*Delta*m); 
Yv_b = (delta_F(2))/(2*Delta*m); 
Zv_b = (delta_F(3))/(2*Delta*m); 
Lv_b = Izz/I_mix*(delta_M(1))/(2*Delta)+Ixz/I_mix*(delta_M(3))/(2*Delta); 
Mv_b = (delta_M(2))/(2*Delta*Iyy);
Nv_b = Ixz/I_mix*(delta_M(1))/(2*Delta)+Ixx/I_mix*(delta_M(3))/(2*Delta);

% w derivatives
[delta_F,delta_M] = step_input(mdl,time,Delta,'V0',3);

Xw_b = (delta_F(1))/(2*Delta*m); 
Yw_b = (delta_F(2))/(2*Delta*m); 
Zw_b = (delta_F(3))/(2*Delta*m); 
Lw_b = Izz/I_mix*(delta_M(1))/(2*Delta)+Ixz/I_mix*(delta_M(3))/(2*Delta); 
Mw_b = (delta_M(2))/(2*Delta*Iyy);
Nw_b = Ixz/I_mix*(delta_M(1))/(2*Delta)+Ixx/I_mix*(delta_M(3))/(2*Delta);

%% angular rate derivatives
disp('calculating angular rate derivatives..')
Delta = 0.01; % [rad/s]
% p derivatives
[delta_F,delta_M] = step_input(mdl,time,Delta,'omega0',1);

Xp_b = (delta_F(1))/(2*Delta*m); 
Yp_b = (delta_F(2))/(2*Delta*m); 
Zp_b = (delta_F(3))/(2*Delta*m); 
Lp_b = Izz/I_mix*(delta_M(1))/(2*Delta)+Ixz/I_mix*(delta_M(3))/(2*Delta); 
Mp_b = (delta_M(2))/(2*Delta*Iyy);
Np_b = Ixz/I_mix*(delta_M(1))/(2*Delta)+Ixx/I_mix*(delta_M(3))/(2*Delta);

% q derivatives
[delta_F,delta_M] = step_input(mdl,time,Delta,'omega0',2);

Xq_b = (delta_F(1))/(2*Delta*m); 
Yq_b = (delta_F(2))/(2*Delta*m); 
Zq_b = (delta_F(3))/(2*Delta*m); 
Lq_b = Izz/I_mix*(delta_M(1))/(2*Delta)+Ixz/I_mix*(delta_M(3))/(2*Delta); 
Mq_b = (delta_M(2))/(2*Delta*Iyy);
Nq_b = Ixz/I_mix*(delta_M(1))/(2*Delta)+Ixx/I_mix*(delta_M(3))/(2*Delta);

% r derivatives
[delta_F,delta_M] = step_input(mdl,time,Delta,'omega0',3);

Xr_b = (delta_F(1))/(2*Delta*m); 
Yr_b = (delta_F(2))/(2*Delta*m); 
Zr_b = (delta_F(3))/(2*Delta*m); 
Lr_b = Izz/I_mix*(delta_M(1))/(2*Delta)+Ixz/I_mix*(delta_M(3))/(2*Delta); 
Mr_b = (delta_M(2))/(2*Delta*Iyy);
Nr_b = Ixz/I_mix*(delta_M(1))/(2*Delta)+Ixx/I_mix*(delta_M(3))/(2*Delta);

%% control derivatives
disp('calculating control derivatives..')
Delta = 0.01; % normalized 
% dA derivatives
[delta_F,delta_M] = step_input(mdl,time,Delta,'pilot_trim',1);

XdA_b = (delta_F(1))/(2*Delta*m); 
YdA_b = (delta_F(2))/(2*Delta*m); 
ZdA_b = (delta_F(3))/(2*Delta*m); 
LdA_b = Izz/I_mix*(delta_M(1))/(2*Delta)+Ixz/I_mix*(delta_M(3))/(2*Delta); 
MdA_b = (delta_M(2))/(2*Delta*Iyy);
NdA_b = Ixz/I_mix*(delta_M(1))/(2*Delta)+Ixx/I_mix*(delta_M(3))/(2*Delta);

% dB derivatives
[delta_F,delta_M] = step_input(mdl,time,Delta,'pilot_trim',2);

XdB_b = (delta_F(1))/(2*Delta*m); 
YdB_b = (delta_F(2))/(2*Delta*m); 
ZdB_b = (delta_F(3))/(2*Delta*m); 
LdB_b = Izz/I_mix*(delta_M(1))/(2*Delta)+Ixz/I_mix*(delta_M(3))/(2*Delta); 
MdB_b = (delta_M(2))/(2*Delta*Iyy);
NdB_b = Ixz/I_mix*(delta_M(1))/(2*Delta)+Ixx/I_mix*(delta_M(3))/(2*Delta);

% dc derivatives
[delta_F,delta_M] = step_input(mdl,time,Delta,'pilot_trim',3);

Xdc_b = (delta_F(1))/(2*Delta*m); 
Ydc_b = (delta_F(2))/(2*Delta*m); 
Zdc_b = (delta_F(3))/(2*Delta*m); 
Ldc_b = Izz/I_mix*(delta_M(1))/(2*Delta)+Ixz/I_mix*(delta_M(3))/(2*Delta); 
Mdc_b = (delta_M(2))/(2*Delta*Iyy);
Ndc_b = Ixz/I_mix*(delta_M(1))/(2*Delta)+Ixx/I_mix*(delta_M(3))/(2*Delta);

% dp derivatives
[delta_F,delta_M] = step_input(mdl,time,Delta,'pilot_trim',4);

Xdp_b = (delta_F(1))/(2*Delta*m); 
Ydp_b = (delta_F(2))/(2*Delta*m); 
Zdp_b = (delta_F(3))/(2*Delta*m); 
Ldp_b = Izz/I_mix*(delta_M(1))/(2*Delta)+Ixz/I_mix*(delta_M(3))/(2*Delta); 
Mdp_b = (delta_M(2))/(2*Delta*Iyy);
Ndp_b = Ixz/I_mix*(delta_M(1))/(2*Delta)+Ixx/I_mix*(delta_M(3))/(2*Delta);


%% Body-axis dynamics matrices:
% x = [u w q theta v p r phi psi], u = [dA dB dc dp] -> [cyc_lat cyc_lon coll ped]

A = [ Xu_b , Xw_b ,          Xq_b-V0(3)           ,           -grav*cos(euler0(2))     , Xv_b ,      Xp_b      ,            Xr_b+V0(2)         ,                    0                  0 ;
      Zu_b , Zw_b ,          Zq_b+V0(1)           , -grav*cos(euler0(1))*sin(euler0(2)), Zv_b ,   Zp_b-V0(2)   ,              Zr_b             , -grav*sin(euler0(1))*cos(euler0(2))   0 ;    
      Mu_b , Mw_b ,             Mq_b              ,                   0                , Mv_b ,       Mp_b     ,              Mr_b             ,                    0                  0 ;
        0  ,  0   ,              1                ,                   0                ,   0  ,        0       ,         -sin(euler0(2))       ,                    0                  0 ;
      Yu_b , Yw_b ,             Yq_b              , -grav*sin(euler0(1))*sin(euler0(2)), Yv_b ,   Yp_b+V0(3)   ,            Yr_b-V0(1)         ,  grav*cos(euler0(1))*cos(euler0(2))   0 ;
      Lu_b , Lw_b ,             Lq_b              ,                   0                , Lv_b ,       Lp_b     ,              Lr_b             ,                    0                  0 ;
      Nu_b , Nw_b ,             Nq_b              ,                   0                , Nv_b ,       Np_b     ,              Nr_b             ,                    0                  0 ;
        0  ,   0  , sin(euler0(1))*tan(euler0(2)) ,                   0                ,   0  ,         1      , cos(euler0(1))*tan(euler0(2)) ,                    0                  0 ;
        0  ,   0  ,               0               ,                   0                ,   0  ,         0      ,          1/cos(euler0(2))     ,                    0                  0 ];

B = [ XdA_b, XdB_b, Xdc_b, Xdp_b;  
      ZdA_b, ZdB_b, Zdc_b, Zdp_b;  
      MdA_b, MdB_b, Mdc_b, Mdp_b;     
        0  ,   0  ,   0  ,   0  ;  
      YdA_b, YdB_b, Ydc_b, Ydp_b;  
      LdA_b, LdB_b, Ldc_b, Ldp_b;  
      NdA_b, NdB_b, Ndc_b, Ndp_b;     
        0  ,   0  ,   0  ,   0  ;     
        0  ,   0  ,   0  ,   0  ];

C = eye(9);
D = zeros(9,4);

% separation of longituginal and lateral dynamics:
% x_lon = [u w q theta], u_lon = [coll cyc_lon]
A_lon = A(1:4,1:4);
B_lon = B(1:4,1:2);

% x_lat = [v p r phi],   u_lat = [cyc_lat ped]
A_lat = A(5:8,5:8);
B_lat = B(5:8,3:4);

%% transfer functions

% lon cyclic
[num,den] = ss2tf(A,B,C,D,2);
theta_dB = zpk(tf(num(4,:),den));
q_dB = zpk(tf(num(3,:),den));
p_dB = zpk(tf(num(6,:),den));
u_dB = zpk(tf(num(1,:),den));

% lat cyclic
[num,den] = ss2tf(A,B,C,D,1);
phi_dA = zpk(tf(num(8,:),den));
q_dA = zpk(tf(num(3,:),den));
p_dA = zpk(tf(num(6,:),den));
v_dA = zpk(tf(num(5,:),den));

% collective
[num,den] = ss2tf(A,B,C,D,3);
u_dc  = zpk(tf(num(1,:),den));
w_dc  = zpk(tf(num(2,:),den));
q_dc  = zpk(tf(num(3,:),den));
az_dc = tf('s')*w_dc-V0(1)*q_dc;
h_dc  = -az_dc*1/tf('s')^2;

% pedals
[num,den] = ss2tf(A,B,C,D,4);
psi_dp = zpk(tf(num(9,:),den));
r_dp = zpk(tf(num(7,:),den));
p_dp = zpk(tf(num(6,:),den));

% save ff_60kn_lin@100ft.mat A B C D A_lon B_lon A_lat B_lat theta_dB q_dB p_dB u_dB phi_dA q_dA p_dA v_dA u_dc w_dc q_dc az_dc h_dc psi_dp r_dp p_dp
disp('%%%%%%%%%%%% LINEARIZATION SUCCESSFULLY COMPLETED %%%%%%%%%%%%')

% reset default mode
mode = 0;

% enable euler dynamics
set_param(mdl+"/HELICOPTER MODEL/6DOF RIGID BODY DYNAMICS",'Commented','off')

% clear variables
clear mdl time grav Delta delta_F delta_M num den I_mix