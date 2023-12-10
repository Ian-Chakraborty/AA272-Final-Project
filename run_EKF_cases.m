close all
clear
clc


%% Run EKF for all cases:
load sensor_data2.mat

[mu_1, sigma_1] = run_EKF(t_meas_1, lidar_range_1, p_XYZ_1, v_1, sigma_sq_p_1);

[mu_2, sigma_2] = run_EKF(t_meas_2, lidar_range_2, p_XYZ_2, v_2, sigma_sq_p_2);

[mu_3, sigma_3] = run_EKF(t_meas_3, lidar_range_3, p_XYZ_3, v_3, sigma_sq_p_3);

[mu, sigma] = run_EKF(t_meas, lidar_range, p_XYZ, v, sigma_sq_p);


function [mu, sigma] =  run_EKF(t_meas, lidar_range, p_XYZ, v, sigma_sq_p)
% Sample from initial distribution
sigma_IC_t = diag([400,10]);
sigma_IC_d = diag([1,1]);
IC_t = mvnrnd(zeros(2,1),sigma_IC_t);
IC_d = mvnrnd(zeros(2,1),sigma_IC_d) ;

% IC_d = [0,0];

IC = [IC_t';IC_d'];
sigma_IC = diag([400,10,1,1]);

t = milliseconds(t_meas - t_meas(1))/1E3;
lidar_range = lidar_range./100;

% Q 
Q = diag([5,5,0.1,0.1]);


mu = nan*ones(length(IC),length(t));
mu(:,1) = IC;

sigma = {};
sigma{1} = sigma_IC;

for i=1:length(t)-1
    dt = t(i+1) - t(i);
    vt = v(i,:);
    [mu_pred, sigma_pred] = EKF_predict(mu(:,i), sigma{i}, vt, Q, dt);

    meas = [p_XYZ(i+1,1:2)'; lidar_range(i+1)];
    R = Rmat(meas,sigma_sq_p(i+1));
    [mu_upd, sigma_upd] = EKF_update(mu_pred, sigma_pred, meas, R);
    
    mu(:,i+1) = mu_upd;
    sigma{i+1} = sigma_upd;


end


figure()
plot(mu(2,:),mu(1,:),'b+')
hold on
plot(mu(4,:),mu(3,:),'r*')
% plot(p_XYZ(:,2),p_XYZ(:,1),'g')
grid on
legend('target','drone')


end

%% Functions


function state_next = nonlinear_dynamics(state, control, dt)
    xt = state(1); yt = state(2); xd = state(3); yd = state(4);
    vxd = control(1); vyd = control(2);
    
    state_next = [xt; yt; xd + vxd*dt; yd + vyd*dt];

end


function R = Rmat(meas,sigma_sq_p)
    rho = meas(3);
    sigma_rho = rho/400;
    R = diag([sigma_sq_p,sigma_sq_p,sigma_rho.^2]);
end

function y = meas_model(state)
    xt = state(1); yt = state(2); xd = state(3); yd = state(4);
    
    y = [xd; yd; sqrt((xt-xd).^2 + (yt-yd).^2)];

end

function A = Amat()
    A = eye(4);
end

function C = Cmat(state)
    xt = state(1); yt = state(2); xd = state(3); yd = state(4);
    rho = sqrt((xt-xd).^2 + (yt-yd).^2);
    C = [0, 0, 1, 0; 0, 0, 0, 1; (xt-xd)/rho, (yt-yd)/rho, (xd-xt)/rho, (yd-yt)/rho];

end


function [mu_pred, sigma_pred] = EKF_predict(mu_upd, sigma_upd, v, Q, dt)
    mu_pred = nonlinear_dynamics(mu_upd, v, dt);
    A = Amat();
    sigma_pred = A*sigma_upd*A + Q;
end

function [mu_upd, sigma_upd] = EKF_update(mu_pred, sigma_pred, meas,R)
    C = Cmat(mu_pred);
    K = sigma_pred*C'*inv(C*sigma_pred*C' + R);

    mu_upd = mu_pred + K*(meas - meas_model(mu_pred));
    sigma_upd = sigma_pred - K*C*sigma_pred;


end