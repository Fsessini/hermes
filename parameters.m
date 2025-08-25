%% helicopter simulation parameters %%
% non-linear 9-DoF dynamic model for real-time simulation of
% conventional configuration helicopter
% (6-DoF for the rigid body dynamics + 3-DoF for main rotor flapping)
% for the mathematical model see Talbot, NASA technical memorandum 84281
% for the example helicopter data see Prouty "Helicopter performance stability and control"
% SI units (sometimes converted from imperial)
clear variables
clc
%% conversion factors
Lb_Kg   = 0.453592; %lb to kilograms
Ft_m    = 0.3048; %feet to meters
Kn_mps  = 0.514444; %knots to meters per second
Slug_Kg = 14.593903; %slugs to kilograms
hp_W    = 745.700; %horsepower to watts

%% solver parameters
%RK4
ST = 0.001; % time step [s]

%% mode selector
mode = 0; % switch for trim and linearization scripts, default:0

%% flight condition
% steady and symmetrical, NED frame
fc  = [  100*Kn_mps   ;  % horizontal velocity [m/s] (leave 0.1 for hover)
          0    ;  % side velocity [m/s]
          0    ]; % rate of climb [m/s]

%% initial conditions 
% position:
Xe_0   = [ 16404 0 -1000 ]'*Ft_m; % initial position in NED (north east down) frame [ft-->m]
latref = 39.24744998520548; % reference latitude for earth fixed-origin [deg]
lonref = 9.050782438199448; % reference longitude for earth fixed-origin [deg]
href   = 0; % reference altitude for earth-fixed origin (zero if on surface but not necessarely at sea level!) [m]
% trim variables:
a_0    = [ 0 0 0 ]'*pi/180; % initial flapping angles (hub-body axis) [a0 a1s b1s] [deg-->rad]
euler0 = [ 0 0 0 ]'*pi/180; % initial attitude (roll, pitch, yaw, angles) [deg-->rad]
V0     = eul2rotm(euler0','XYZ')'*[ fc(1) fc(2) fc(3) ]'; % initial velocity in body frame [m/s] 
omega0 = [ 0 0 0 ]'; % initial rotational velocity (roll, pitch, yaw rate) [rad/s] 

%% inertial parameters
m = 20000*Lb_Kg; % take-off mass [lb-->kg]
Ixx = 5000*Slug_Kg*Ft_m^2;
Iyy = 40000*Slug_Kg*Ft_m^2;
Izz = 35000*Slug_Kg*Ft_m^2;
Ixz = 0*Slug_Kg*Ft_m^2;
inertia = [ Ixx 0 -Ixz; 0 Iyy 0; -Ixz 0 Izz ]; % matrix of inertia in body frame [slug*ft^2-->kg*m^2]
STACG = 24.4*Ft_m; % station line of CG [m]
BLCG  = 0; % buttline of CG [m]
WLCG  = 9.2*Ft_m; % water line of CG [m]

%% environmental conditions
wind_velocity_e = [ 0 0 0 ]'; % wind speed in NED frame [m/s]
wind_rate       = [ 0 0 0 ]'; % wind rate in body frame [rad/s]
dyn_viscosity   = 17.89e-6; % reference air dynamic viscosity [Pa*s]

%% controls input range ([deg])
lon_cyc_range = [ -15 15 ]; % positive aft
lat_cyc_range = [ -15 15 ]; % positive right
mr_coll_range = [   0 25 ]; 
tr_coll_range = [   0 20 ]; 

%% main rotor parameters 
% second order equation for tip-path-plane flapping dynamics is solved to
% calculate unsteady hub forces and moments averaged over one rotation (with closed form equations)
% uniform inflow is calculated with Glauert's momentum theory
% linear inflow correction is applied only to model lateral flapping in forward flight
chi_mr  = 1; % positive for counter-clockwise rotation (not completely implemented, feathering angles? flapping?)
i_s     = 0; % forward tilt of main rotor mast
AOP     = 0; % precone angle (required only for teetering rotor) [rad]
OMEGA   = 206.9*(pi/30); % angular speed (supposed constant) [rpm]-->[rad/s]           
BLADES  = 4; % number of blades 
ROTOR   = 30*Ft_m; % rotor radius [ft-->m]
CHORD   = 2*Ft_m; % mean aerodynamic chord [ft-->m]
ASLOPE  = 6; % blade lift curve slope [1/rad]
AKONE   = 0; % pitch-flap coupling ratio (tan(delta_3)) 
AKBETA  = 0; % flapping spring constant [Nm/rad] (models hingeless rotors)
EPSLN   = 0.05; % offset ratio of flapping hinge (models articulated and hingeless rotors)
GAMMA   = 8.1; % Lock number: ratio of aerodynamic to inertial flapping moment
THETT   = -10*(pi/180); % amount of twist from tip to root (theta_tip-theta_hub) [rad]
                        % Theta = Theta0 + THETT*r; r=y/R; Theta0: blade root collective pich
m_blade = 0.372*Slug_Kg/Ft_m; % blade mass per unit span [slug/ft-->Kg/m]
Jbeta   = 1/3*m_blade*ROTOR^3*(1-EPSLN)^3; % blade moment of inertia around flap hinge [slug*ft2-->Kg*m2]
                                           % (the blade is approximated as an homogeneus slender rod)
T_flap  = (16/GAMMA/OMEGA)/((1-EPSLN)^4*((1+1/3*EPSLN)/(1-EPSLN))); % time constant for main rotor flapping
%three-term profile drag polar: C_D = C_D0+C_D1*alpha+C_D2*alpha^2
CD_mr   = [0.0107 -0.151 1.72]; %[CD0,CD1,CD2]
k_ind   = 1; % induced power correction factor 
STAMR   = 23.9*Ft_m; % station line of main rotor [ft-->m]
BLMR    = 0; % butt line of main rotor [ft-->m]
WLMR    = 16.7*Ft_m; % water line of main rotor [ft-->m]
max_flap_mr = 20*pi/180; % maximum flapping angle allowed by hinge [deg-->rad]
alpha_stall = 15; % blade section stall angle [deg]
max_load    = 3*m*9.81; % maximum axial load (thrust) bearable by the hub [N]
rated_power = 4170*hp_W; % transmission power rating

%% tail rotor parameters
% supposed behaveour of teetering rotor (zero hinge offset), 
% flapping dynamics neglected due to high angular speed (steady state solution to t.p.p. angles),
% no cyclic pitch, effect of downwash of main rotor not considered
chi_tr    = 1; % positive for bottom-forward rotation (not implemented) 
AOPTR     = 0; % precone angle (required only for teetering rotor) [rad]
OMEGATR   = 954.93*(pi/30); % angular speed (supposed constant) [rpm]-->[rad/s]           
BLADESTR  = 3; % number of blades 
ROTORTR   = 6.5*Ft_m; % rotor radius [ft-->m]
CHORDTR   = 1*Ft_m; % mean aerodynamic chord [ft-->m]
ASLOPETR  = 6; % blade lift curve slope [1/rad]
AKONETR   = tan(30*(pi/180)); % pitch-flap coupling ratio (tan(delta_3)) 
GAMMATR   = 4; % Lock number: ratio of inertial to aerodynamic flapping moment
THETTTR   = -5*(pi/180); % amount of twist from tip to root (theta_tip-theta_hub) [rad]
                         % Theta = Theta0 + THETT*r; r=y/R; Theta0: blade root collective pich
CD_tr     = [ 0.0107 -0.151 1.72 ]; %[CD0,CD1,CD2]
k_ind_tr  = 1; % induced power correction factor                           
Ttrb  = [ 1 0 0; 0 0 1; 0 -1 0]; % rotation matrix from body to tail rotor hub axis 
STATR = 61.4*Ft_m; % station line of tail rotor [ft-->m]
BLTR  = -1.8*Ft_m; % butt line of tail rotor [ft-->m]
WLTR  = 15.2*Ft_m; % water line of tail rotor [ft-->m]
max_flap_tr = 20*pi/180; % maximum flapping angle allowed by hinge 

%% horizontal stabilizer parameters
% effect of main rotor downwash is considered on direction and magnitude of the apparent wind
% the hor.stab. is supposed to always be inside the completely developed rotor wake (hover and low speed forward flight)
% symmetrical NACA0012 profile
a_hs  = 6; % lift slope [rad-1]
S_hs  = 18*(Ft_m^2); % planform area of hor.stab. [ft2-->m2]
AR_hs = 4.5; % aspect ratio of hor.stab.
a0_hs = 0*(pi/180); % angle between chord and zero lift line [deg-->rad]
i_hs  = -3*(pi/180)+a0_hs; % incidence of hor.stab. (measured from zero lift line) [deg-->rad]
e_hs  = 0.8; % Oswald parameter for 3D effects
CL_max_hs = 1.2; % stall lift coefficient of hor.stab.
Lambda_hs = 13*(pi/180); % sweep angle of hor.stab. mean chord line [deg-->rad]
STAHS = 57.4*Ft_m; % station line of hor.stab. [ft-->m]
BLHS  = 0; % butt line of hor.stab. [ft-->m]
WLHS  = 7.7*Ft_m; % water line of hor.stab. [ft-->m]

%% vertical stabilizer parameters
% effect of tail rotor downwash is considered in the sideforce drag
% no effect of main rotor downwash is considered
% cambered profile
a_vs  = 6; % lift slope [rad-1]
S_vs  = 33*(Ft_m^2); % planform area of ver.stab. [ft2-->m2]
AR_vs = 1.8; % aspect ratio of ver.stab.
a0_vs = -5*(pi/180); % angle between chord and zero lift line [deg-->rad]
i_vs  = 0*(pi/180)+a0_vs; % incidence of ver.stab. (measured from zero lift line) [deg-->rad]
e_vs  = 0.8; % Oswald parameter for 3D effects
kv_tr = 0.8; % fraction of ver.stab. covered by tail rotor, positive for puller configuration
CL_max_vs = 1.2; % stall lift coefficient of ver.stab.
Lambda_vs = 27*(pi/180); % sweep angle of ver.stab. mean chord line [deg-->rad]
STAVS = 59.4*Ft_m; % station line of ver stab [ft-->m]
BLVS  = 0; % butt line of ver stab [ft-->m]
WLVS  = 12.2*Ft_m; % water line of ver stab [ft-->m]

%% fuselage parameters
% note that the implemented model is valid for small attack and sideslip angles <=15deg
% effect of rotor downwash is considered only on the fuselage a.o.a.
CD_f  = [1.774 0.2043 7]; % drag coefficients
CL_f  = [-0.4279 10.33]; % lift coefficients
CY_f  = [-0.0359 -16.987]; % sideforce coefficients
CLL_f = [0.0696 6.336]; % rolling moment coefficients
CM_f  = [-4.4961 49.522]; % pitching moment coefficients
CN_f  = [0.0396 -21.699]; % yawing moment coefficients
STARPF = 23.9*Ft_m;%24.9*Ft_m; % station line of fuselage reference point [ft-->m]
BLRPF  = 0; % butt line of fuselage reference point [ft-->m]
WLRPF  = 12.2*Ft_m; % waterline line of fuselage reference point [ft-->m]
