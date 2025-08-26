%% step response script %% 
% compare the response to a step input between the complete model and the linearized one

% specify simulation model
mdl = "helicopter_sim";

% disable blocks for real time sim
set_param(mdl+"/GCS gamepad",'Commented','on')
set_param(mdl+"/3D_visualization",'Commented','on')

% parameters
Delta = 0.05; % input amplitude (hov: ~0.2, ff:~.05)
t = 5; % simulation time
time = string(t);
cmap = colororder();

%-----------------------
function SimOut = step_input(mdl,time,Delta,x,i)

    x0 = evalin('base', x); 
    x1 = x0;
    x1(i) = x1(i) + Delta;
    assignin('base', x, x1);
    SimOut = sim(mdl, 'StopTime', time); 
    assignin('base', x, x0);

end
%-----------------------

%% longitudinal cyclic
figure()
title('longitudinal cyclic')
hold on
xlabel('time')

% non-linear response 
SimOut = step_input(mdl,time,Delta,'pilot_trim',2);
yyaxis left
ax = gca; ax.YColor = 'k';
ylabel('angles [deg] and rates [deg/s]')
plot(SimOut.simtime(:),(SimOut.PQR(:,2))*180/pi,'-','Color',cmap(1,:),'DisplayName', 'q')
plot(SimOut.simtime(:),(SimOut.PQR(:,1))*180/pi,'-','Color',cmap(2,:),'DisplayName', 'p')
plot(SimOut.simtime(:),((SimOut.Euler(:,2))-euler0(2))*180/pi,'-','Color',cmap(3,:),'DisplayName', 'theta')
yyaxis right
ax = gca; ax.YColor = 'k';
ylabel('velocities [m/s]')
plot(SimOut.simtime(:),((SimOut.UVW(:,1))-V0(1)),'-','Color',cmap(4,:),'DisplayName', 'u');

% linear response
yyaxis left
plot(0:0.001:t,step(Delta*q_dB,0:0.001:t)*180/pi,'--','Color',cmap(1,:),'DisplayName', 'q_{lin}')
plot(0:0.001:t,step(Delta*p_dB,0:0.001:t)*180/pi,'--','Color',cmap(2,:),'DisplayName', 'p_{lin}')
plot(0:0.001:t,step(Delta*theta_dB,0:0.001:t)*180/pi,'--','Color',cmap(3,:),'DisplayName', 'theta_{lin}')
yyaxis right
plot(0:0.001:t,step(Delta*u_dB,0:0.001:t),'--','Color',cmap(4,:),'DisplayName', 'u_{lin}')

hold off
legend()
clear simOut;

%% lateral cyclic
figure()
title('lateral cyclic')
hold on
xlabel('time')

% non-linear response 
SimOut = step_input(mdl,time,Delta,'pilot_trim',1);
yyaxis left
ax = gca; ax.YColor = 'k';
ylabel('angles [deg] and rates [deg/s]')
plot(SimOut.simtime(:),(SimOut.PQR(:,2))*180/pi,'-','Color',cmap(1,:),'DisplayName', 'q')
plot(SimOut.simtime(:),(SimOut.PQR(:,1))*180/pi,'-','Color',cmap(2,:),'DisplayName', 'p')
plot(SimOut.simtime(:),((SimOut.Euler(:,1))-euler0(1))*180/pi,'-','Color',cmap(3,:),'DisplayName', 'phi')
yyaxis right
ax = gca; ax.YColor = 'k';
ylabel('velocities [m/s]')
plot(SimOut.simtime(:),((SimOut.UVW(:,2))-V0(2)),'-','Color',cmap(4,:),'DisplayName', 'v')

% linear response
yyaxis left
plot(0:0.001:t,step(Delta*q_dA,0:0.001:t)*180/pi,'--','Color',cmap(1,:),'DisplayName', 'q_{lin}')
plot(0:0.001:t,step(Delta*p_dA,0:0.001:t)*180/pi,'--','Color',cmap(2,:),'DisplayName', 'p_{lin}')
plot(0:0.001:t,step(Delta*phi_dA,0:0.001:t)*180/pi,'--','Color',cmap(3,:),'DisplayName', 'phi_{lin}')
yyaxis right
plot(0:0.001:t,step(Delta*v_dA,0:0.001:t),'--','Color',cmap(4,:),'DisplayName', 'v_{lin}')

hold off
legend()
clear simOut;

%% collective
figure()
title('collective')
hold on
xlabel('time')

% non-linear response 
SimOut = step_input(mdl,time,Delta,'pilot_trim',3);
yyaxis left
ax = gca; ax.YColor = 'k';
ylabel('angles [deg] and rates [deg/s]')
plot(SimOut.simtime(:),(SimOut.PQR(:,2))*180/pi,'-','Color',cmap(1,:),'DisplayName', 'q')
yyaxis right
ax = gca; ax.YColor = 'k';
ylabel('velocities [m/s]')
plot(SimOut.simtime(:),(SimOut.UVW(:,3))-V0(3),'-','Color',cmap(2,:),'DisplayName', 'w')
plot(SimOut.simtime(:),(SimOut.UVW(:,1))-V0(1),'-','Color',cmap(3,:),'DisplayName', 'u')
%plot(SimOut.simtime(:),(SimOut.UVW_dot(:,3)),'-','Color',cmap(4,:),'DisplayName', 'a_z')

% linear response
yyaxis left
plot(0:0.001:t,step(Delta*q_dc,0:0.001:t)*180/pi,'--','Color',cmap(1,:),'DisplayName', 'q_{lin}')
yyaxis right
plot(0:0.001:t,step(Delta*w_dc,0:0.001:t),'--','Color',cmap(2,:),'DisplayName', 'w_{lin}')
plot(0:0.001:t,step(Delta*u_dc,0:0.001:t),'--','Color',cmap(3,:),'DisplayName', 'u_{lin}')
%plot(0:0.001:t,step(Delta*az_dc,0:0.001:t),'--','Color',cmap(4,:),'DisplayName', 'a_z_{lin}')

hold off
legend()
clear simOut;

%% pedals
figure()
title('pedals')
hold on
xlabel('time')

% non-linear response 
SimOut = step_input(mdl,time,Delta,'pilot_trim',4);
ylabel('angles [deg] and rates [deg/s]')
plot(SimOut.simtime(:),(SimOut.PQR(:,3))*180/pi,'-','Color',cmap(1,:),'DisplayName', 'r')
plot(SimOut.simtime(:),((SimOut.Euler(:,3))-euler0(3))*180/pi,'-','Color',cmap(2,:),'DisplayName', 'psi')
plot(SimOut.simtime(:),(SimOut.PQR(:,1))*180/pi,'-','Color',cmap(3,:),'DisplayName', 'p')

% linear response
plot(0:0.001:t,step(Delta*r_dp,0:0.001:t)*180/pi,'--','Color',cmap(1,:),'DisplayName', 'r_{lin}')
plot(0:0.001:t,step(Delta*psi_dp,0:0.001:t)*180/pi,'--','Color',cmap(2,:),'DisplayName', 'psi_{lin}')
plot(0:0.001:t,step(Delta*p_dp,0:0.001:t)*180/pi,'--','Color',cmap(3,:),'DisplayName', 'p_{lin}')

hold off
legend()
clear SimOut mdl Delta t time cmap ax 

