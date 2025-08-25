%% stability script %%
% trim and linearize the system to evaluate stability for a range of forward flight speeds
% runs the trim and linearization routines 
% saves and plots trim control inputs and eigenvalues against flight speed
% save file: 'results/stab.mat'

% initialize parameters
run('parameters.m')

% initialize forward velocity array 
V_vec = [0,60,90,120,150,160]*0.514444; % knots to m/s
V_vec(1) = 0.1; % hover

% initialize storing variables
trim_mat = zeros(4,length(V_vec));
eig_mat  = zeros(8,length(V_vec));
eig_mat_lon = zeros(4,length(V_vec));
eig_mat_lat = zeros(4,length(V_vec));

% start
for k=1:length(V_vec)
    fc(1) = V_vec(k);
    fprintf('<strong> velocity </strong>:%f m/s \n',fc(1))
    run('trim.m')
    trim_mat(:,k) = pilot_trim(:);
    run('linearization.m')
    eig_mat(:,k) = eig(A(1:8,1:8));
    eig_mat_lon(:,k) = eig(A_lon);
    eig_mat_lat(:,k) = eig(A_lat);
end
V_vec = [0,60,90,120,150,160]; % speed vector in knots
save results/stab.mat trim_mat eig_mat eig_mat_lon eig_mat_lat V_vec
clear k

%% plot control inputs
cmap = colororder();

figure()
title('cyclic pitch input')
hold on; grid on
xlabel('forward speed [kts]')
ylabel('controls deflection [deg]')
plot(V_vec,trim_mat(1,:)*0.5*(lat_cyc_range(2)-lat_cyc_range(1))+0.5*(lat_cyc_range(2)+lat_cyc_range(1)),...
    '--.','MarkerSize',15,'DisplayName', '\delta_A: lateral cyc')
plot(V_vec,trim_mat(2,:)*0.5*(lon_cyc_range(2)-lon_cyc_range(1))+0.5*(lon_cyc_range(2)+lon_cyc_range(1)),...
    '--.','MarkerSize',15,'DisplayName', '\delta_B: longitudinal cyc')
xlim([V_vec(1),V_vec(end)]); ylim([-15,0]);
hold off
legend()

figure()
title('main and tail rotor collective pitch input')
hold on; grid on
xlabel('forward speed [kts]')
ylabel('controls deflection [deg]')
plot(V_vec,trim_mat(3,:)*0.5*(mr_coll_range(2)-mr_coll_range(1))+0.5*(mr_coll_range(2)+mr_coll_range(1)),...
    '--.','MarkerSize',15,'DisplayName', '\delta_c: collective')
plot(V_vec,trim_mat(4,:)*0.5*(tr_coll_range(2)-tr_coll_range(1))+0.5*(tr_coll_range(2)+tr_coll_range(1)),...
    '--.','MarkerSize',15,'DisplayName', '\delta_p: pedals')
xlim([V_vec(1),V_vec(end)]); ylim([0,25]);
hold off
legend()

%% plot eigenvalues
cmap = colororder();

% hover 
[~, idx] = sort(abs(eig_mat(:,1)));
eig_hov = eig_mat(idx,1);
figure();
hold on; grid on; axis equal;
% keep only the poles with positive imaginary part
eig_hov = eig_hov(imag(eig_hov) >= 0);
eig_hov([2 3]) = eig_hov([3 2]);
%eig_hov = reshape(eig_hov, [], n_param);
n_modes = length(eig_hov);
for k = 1:n_modes
    plot(real(eig_hov(k,:)), imag(eig_hov(k,:)), '.', 'MarkerSize',15,'Color',cmap(k,:));
end
xlabel('Re'); ylabel('Im');
xlim([-9,1]); ylim([-.5,4])
title('Eigenvalues in Hover');
%legend show;

% coupled dynamics in forward flight 
eig_ff = eig_mat(:,2:end);
[n_modes, n_param] = size(eig_ff);

% sort eigenvalues
eig_sorted = zeros(n_modes, n_param);
[~, idx] = sort(abs(eig_ff(:,1)));
eig_sorted(:,1) = eig_ff(idx,1);
for j = 2:n_param
    prev = eig_sorted(:,j-1);    
    curr = eig_ff(:,j);         
    idx = zeros(n_modes,1);
    available = 1:n_modes;
    for k = 1:n_modes
        [~,ii] = min(abs(curr(available) - prev(k))); % eucledian distance
        idx(k) = available(ii);
        available(ii) = []; % remove matched index
    end
    eig_sorted(:,j) = curr(idx);
end

% keep only the poles with positive imaginary part
eig_sorted = eig_sorted(imag(eig_sorted) >= 0);
eig_sorted = reshape(eig_sorted, [], n_param);
[n_modes, n_param] = size(eig_sorted);

% plot 
figure();
hold on; grid on; axis equal;
for k = 1:n_modes
    plot(real(eig_sorted(k,:)), imag(eig_sorted(k,:)), '-','Color',cmap(k,:));
    plot(real(eig_sorted(k,1)), imag(eig_sorted(k,1)), 'square','MarkerSize',3,'Color',cmap(k,:),'MarkerFaceColor',cmap(k,:)); % mark starting
    plot(real(eig_sorted(k,n_param)), imag(eig_sorted(k,n_param)), '^','MarkerSize',3,'Color',cmap(k,:),'MarkerFaceColor',cmap(k,:)); % mark ending
end

xlabel('Re'); ylabel('Im');
xlim([-9,1]); ylim([-0.5,4])
title('Eigenvalues Trajectories with Forward Speed');
%legend show;
eig_ff_cp = eig_sorted;
clear eig_hov n_modes n_param k eig_ff eig_sorted j prev curr idx available ii 



% longitudinal dynamics
% prepare longitudinal coupled dynamics:
eig_ff_cp_lon = eig_ff_cp([2,3,5],:);
eig_ff_cp_lon([1 3],:) = eig_ff_cp_lon([3 1],:);
% prepare longitudinal uncoupled dynamics:
eig_mat_lon_ff = eig_mat_lon(:,2:end);
[n_modes, n_param] = size(eig_mat_lon_ff);
% sort eigenvalues
eig_sorted = zeros(n_modes, n_param);
eig_sorted(:,1) = eig_mat_lon_ff(:,1);
for j = 2:n_param
    prev = eig_sorted(:,j-1);    
    curr = eig_mat_lon_ff(:,j);         
    idx = zeros(n_modes,1);
    available = 1:n_modes;
    for k = 1:n_modes
        [~,ii] = min(abs(curr(available) - prev(k))); % eucledian distance
        idx(k) = available(ii);
        available(ii) = []; % remove matched index
    end
    eig_sorted(:,j) = curr(idx);
end
% keep only the poles with positive imaginary part
eig_sorted = eig_sorted(imag(eig_sorted) >= 0);
eig_sorted = reshape(eig_sorted, [], n_param);
[n_modes, n_param] = size(eig_sorted);
% plot:
figure()
sgtitle('Longitudinal Eigenvalues Trajectories with Forward Speed')
j = 0;
for k = 1:n_modes
    % % determine axis limits
    % x_min = min([min(real(eig_ff_cp_lon(k,:))),min(real(eig_sorted(k,:)))])
    % x_max = max([max(real(eig_ff_cp_lon(k,:))),max(real(eig_sorted(k,:)))])
    % y_min = min([min(imag(eig_ff_cp_lon(k,:))),min(imag(eig_sorted(k,:)))])
    % y_max = max([max(imag(eig_ff_cp_lon(k,:))),max(imag(eig_sorted(k,:)))])
    % if y_min==0 || y_max==0
    %     y_min = -.25*abs(x_max-x_min); y_max = .25*abs(x_max-x_min);
    % end

    % coupled (zoom in previous plot)
    subplot(3,2,k+j)
    hold on; grid on; axis equal;
    plot(real(eig_ff_cp_lon(k,:)), imag(eig_ff_cp_lon(k,:)), '-','Color',cmap(k,:));
    plot(real(eig_ff_cp_lon(k,1)), imag(eig_ff_cp_lon(k,1)), 'square','MarkerSize',3,'Color',cmap(k,:),'MarkerFaceColor',cmap(k,:)); % mark starting
    plot(real(eig_ff_cp_lon(k,n_param)), imag(eig_ff_cp_lon(k,n_param)), '^','MarkerSize',3,'Color',cmap(k,:),'MarkerFaceColor',cmap(k,:)); % mark ending
    xlabel('Re'); ylabel('Im');
    %xlim([x_min,x_max]); ylim([y_min,y_max])
    title('coupled');
    
    % uncoupled
    subplot(3,2,k+j+1)
    hold on; grid on; axis equal;
    plot(real(eig_sorted(k,:)), imag(eig_sorted(k,:)), '-','Color',cmap(k,:));
    plot(real(eig_sorted(k,1)), imag(eig_sorted(k,1)), 'square','MarkerSize',3,'Color',cmap(k,:),'MarkerFaceColor',cmap(k,:)); % mark starting
    plot(real(eig_sorted(k,n_param)), imag(eig_sorted(k,n_param)), '^','MarkerSize',3,'Color',cmap(k,:),'MarkerFaceColor',cmap(k,:)); % mark ending
    xlabel('Re'); ylabel('Im');
    %xlim([x_min,x_max]); ylim([y_min,y_max])
    title('uncoupled');
    %legend show;
    j = j+1;
end
clear eig_hov n_modes n_param k eig_ff eig_sorted j prev curr idx available ii 


% lateral dynamics
% prepare lateral coupled dynamics:
eig_ff_cp_lat = eig_ff_cp([1,4,6],:);
eig_ff_cp_lat([1 3],:) = eig_ff_cp_lat([3 1],:);
% prepare lateral uncoupled dynamics:
eig_mat_lat_ff = eig_mat_lat(:,2:end);
[n_modes, n_param] = size(eig_mat_lat_ff);
eig_sorted = zeros(n_modes, n_param);
eig_sorted(:,1) = eig_mat_lat_ff(:,1);
for j = 2:n_param
    prev = eig_sorted(:,j-1);    
    curr = eig_mat_lat_ff(:,j);         
    idx = zeros(n_modes,1);
    available = 1:n_modes;
    for k = 1:n_modes
        [~,ii] = min(abs(curr(available) - prev(k))); % eucledian distance
        idx(k) = available(ii);
        available(ii) = []; % remove matched index
    end
    eig_sorted(:,j) = curr(idx);
end
% keep only the poles with positive imaginary part
eig_sorted = eig_sorted(imag(eig_sorted) >= 0);
eig_sorted = reshape(eig_sorted, [], n_param);
[n_modes, n_param] = size(eig_sorted);

% plot:
figure()
sgtitle('Lateral-directional Eigenvalues Trajectories with Forward Speed')
j = 0;
for k = 1:n_modes
    % % determine axis limits
    % x_min = min([min(real(eig_ff_cp_lon(k,:))),min(real(eig_sorted(k,:)))])
    % x_max = max([max(real(eig_ff_cp_lon(k,:))),max(real(eig_sorted(k,:)))])
    % y_min = min([min(imag(eig_ff_cp_lon(k,:))),min(imag(eig_sorted(k,:)))])
    % y_max = max([max(imag(eig_ff_cp_lon(k,:))),max(imag(eig_sorted(k,:)))])
    % if y_min==0 || y_max==0
    %     y_min = -.25*abs(x_max-x_min); y_max = .25*abs(x_max-x_min);
    % end

    % coupled (zoom in previous plot)
    subplot(3,2,k+j)
    hold on; grid on; axis equal;
    plot(real(eig_ff_cp_lat(k,:)), imag(eig_ff_cp_lat(k,:)), '-','Color',cmap(k,:));
    plot(real(eig_ff_cp_lat(k,1)), imag(eig_ff_cp_lat(k,1)), 'square','MarkerSize',3,'Color',cmap(k,:),'MarkerFaceColor',cmap(k,:)); % mark starting
    plot(real(eig_ff_cp_lat(k,n_param)), imag(eig_ff_cp_lat(k,n_param)), '^','MarkerSize',3,'Color',cmap(k,:),'MarkerFaceColor',cmap(k,:)); % mark ending
    xlabel('Re'); ylabel('Im');
    %xlim([x_min,x_max]); ylim([y_min,y_max])
    title('coupled');
    
    % uncoupled
    subplot(3,2,k+j+1)
    hold on; grid on; axis equal;
    plot(real(eig_sorted(k,:)), imag(eig_sorted(k,:)), '-','Color',cmap(k,:));
    plot(real(eig_sorted(k,1)), imag(eig_sorted(k,1)), 'square','MarkerSize',3,'Color',cmap(k,:),'MarkerFaceColor',cmap(k,:)); % mark starting
    plot(real(eig_sorted(k,n_param)), imag(eig_sorted(k,n_param)), '^','MarkerSize',3,'Color',cmap(k,:),'MarkerFaceColor',cmap(k,:)); % mark ending
    xlabel('Re'); ylabel('Im');
    %xlim([x_min,x_max]); ylim([y_min,y_max])
    title('uncoupled');
    %legend show;
    j = j+1;
end
clear eig_hov n_modes n_param k eig_ff eig_sorted j prev curr idx available ii 
