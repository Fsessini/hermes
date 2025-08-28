%% helicopter trim script %%
% solves for pilot controls and helicopter attitude in a prescribed steady flight condition
% limited to hover, level forward flight, steady climb/descent 
% imposes equilibrium of forces and moments in a Newton-Raphson algorithm
% 6x6 Jacobian matrix is approximated by finite differences 
% unknown vector: x = [ lat_cyc lon_cyc coll pedals bank pitch ]

% specify simulation model
mdl = "helicopter_sim";

% select mode
mode = 1;

% load the model
load_system(mdl)

% disable euler dynamics
set_param(mdl+"/HELICOPTER MODEL/6DOF RIGID BODY DYNAMICS",'Commented','on')

% disable blocks for real time sim
set_param(mdl+"/GCS gamepad",'Commented','on')
set_param(mdl+"/3D_visualization",'Commented','on')

% Initial guess for unknowns
x0 = [ 0 0 0 0 0 0]';

% maximum attitude angles [deg]:
max_att = 15; 

% simulation time (long enough to reach steady flapping)
time = '3';

% solver parameters
h = 1e-6;
tol = 1e-4;
it_max = 10;

% reset initial conditions
a_0    = [ 0 0 0 ]'*pi/180; % initial flapping angles (hub-body axis) [a0 a1s b1s] [deg-->rad]
euler0 = [ 0 0 0 ]'*pi/180; % initial attitude (roll, pitch, yaw, angles) [deg-->rad]
V0     = eul2rotm(euler0','XYZ')'*[ fc(1) fc(2) fc(3) ]'; % initial velocity in body frame [m/s] 

% ----------------------
function y = run_heli_ss(x,max_att,mdl,time)

    % Inputs unpacking
    pilot_trim   = [ x(1);   % lateral cyclic
                     x(2);   % longitudinal cyclic
                     x(3);   % collective
                     x(4) ]; % pedals (tail rotor collective)
    euler_trim   = [ x(5);   % roll
                     x(6);   % pitch
                      0   ]; % yaw

    % dimensionalize attitude angles
    euler0 = [ euler_trim(1)*abs(max_att) euler_trim(2)*abs(max_att) euler_trim(3)*abs(max_att) ]'*pi/180;

    % assign variables to workspace
    assignin('base','pilot_trim',pilot_trim)
    assignin('base','euler0',euler0)
    
    % run simulation and extract outputs
    simOut = sim(mdl, 'StopTime', time);  
    F = simOut.F_tot(:,:,end);
    M = simOut.Map(:,:,end);
    
    % collect output vector
    y = [ F(1);
          F(2);
          F(3);
          M(1);
          M(2);
          M(3) ];
end
% ----------------------

% start:
disp('%%%%%%%%%%%% STARTING TRIM PROCEDURE %%%%%%%%%%%%')
n = length(x0);
F0 = run_heli_ss(x0,max_att,mdl,time);
it = 1;
while norm(F0,inf)>tol && it<=it_max
    J = zeros(n,n);
    for i=1:n
        x0h = x0;
        x0h(i) = x0(i)+h;
        Fh = run_heli_ss(x0h,max_att,mdl,time);
        J(:,i) = (Fh-F0)/h;
    end
    x = x0+J\(-F0);
    x = max(min(x, 1), -1); % keep inputs inside bounds
    F0 = run_heli_ss(x,max_att,mdl,time);
    fprintf('iteration %d completed \n',it)
    disp('input:')
    disp(x)
    fprintf('normalized distance from tolerance: %f \n',norm(F0,inf)/tol)
    x0 = x;
    it = it+1;
end

% write output only if calculation converged:
if it>=it_max
    fprintf(['<strong> RESULT MAY BE INCORRECT </strong> \n ' ...
             'number of iterations higher than %f \n'],it_max)
else
    % result:
    pilot_trim = [ x(1); x(2); x(3); x(4) ]; % normalized [-1,1]
    euler_trim = [ x(5); x(6); 0 ]; % normalized [-1,1]
    
    % store trimmed initial conditions:
    euler0 = [ euler_trim(1)*abs(max_att) euler_trim(2)*abs(max_att) euler_trim(3)*abs(max_att) ]'*pi/180; % initial attitude (roll, pitch, yaw, angles) [deg-->rad]
    V0     = eul2rotm(euler0','XYZ')'*[ fc(1) fc(2) fc(3) ]'; % initial velocity in body frame [m/s] 
    
    % run longer simulation and save steady state flapping angles:
    time_long = '5';
    simOut = sim(mdl, 'StopTime', time_long);
    flap_ss = pi/180*simOut.flapping(:,:,end); % hub-body axes [rad]
    a_0    = [ flap_ss(1) flap_ss(2) flap_ss(3) ]'; % initial flapping angles in hub-body axes [a0 a1s b1s] [rad]

    % console output:
    fprintf('<strong> Main Rotor Thrust </strong>%f \n',simOut.thrust_mr(end))
    fprintf('<strong> Tail Rotor Thrust </strong>%f \n',simOut.thrust_tr(end))
    fprintf('<strong> a_0 </strong>%f \n',simOut.flapping(1,:,end))
    fprintf('<strong> a_1s </strong>%f \n',simOut.flapping(2,:,end))
    fprintf('<strong> b_1s </strong>%f \n',simOut.flapping(3,:,end))
    fprintf('<strong> Theta </strong>%f \n',euler0(2)*180/pi)
    fprintf('<strong> Phi </strong>%f \n',euler0(1)*180/pi)
    if fc(1) ~= 0
        fprintf('<strong> Beta </strong>%f \n',asin(V0(2)/V0(1))*180/pi)
    end
    %fprintf('<strong> A1 </strong>%f \n',(pilot_trim(1)*0.5*(lat_cyc_range(2)-lat_cyc_range(1))+0.5*(lat_cyc_range(2)+lat_cyc_range(1))))
    %fprintf('<strong> B1 </strong>%f \n',(pilot_trim(2)*0.5*(lon_cyc_range(2)-lon_cyc_range(1))+0.5*(lon_cyc_range(2)+lon_cyc_range(1))))
    fprintf('<strong> A1 </strong>%f \n',simOut.AB(1,:,end)*180/pi)
    fprintf('<strong> B1 </strong>%f \n',simOut.AB(2,:,end)*180/pi)
    fprintf('<strong> MR Collective </strong>%f \n',(pilot_trim(3)*0.5*(mr_coll_range(2)-mr_coll_range(1))+0.5*(mr_coll_range(2)+mr_coll_range(1))))
    fprintf('<strong> TR Collective </strong>%f \n',(pilot_trim(4)*0.5*(tr_coll_range(2)-tr_coll_range(1))+0.5*(tr_coll_range(2)+tr_coll_range(1))))
    disp('%%%%%%%%%%%% TRIM SUCCESSFULLY COMPLETED %%%%%%%%%%%%')
end


% reset default mode
mode = 0;

% enable euler dynamics
set_param(mdl+"/HELICOPTER MODEL/6DOF RIGID BODY DYNAMICS",'Commented','off')

% clear variables
clear simOut mdl x x0 time h tol it_max n F0 J x0h Fh time_long flap_ss it i