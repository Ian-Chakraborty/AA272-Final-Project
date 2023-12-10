close all
clear
clc
load heading_information.mat
load held_ekf_results.mat
load flight_ekf_results.mat
load sensor_data2.mat
%% Target Truth

[lat0, lon0, ref_alt, ~, ~, ~] = truth_data("table_truth");
geoid_height_N = geoidheight(lat0,lon0+360);
alt0 = ref_alt - geoid_height_N;
lla0 = [lat0,lon0,alt0];

%% Handheld Case 1


[lat_t, lon_t, ~] = XYZtoLLa([mu_1(1,:)',mu_1(2,:)',p_XYZ_1(:,3)],heading_angles(2),lla0);


n_points = 100;
sigma_plot = sqrt(sigma_1{1}(1));

X = mu_1(1,1);
Y = mu_1(2,1);
Z = p_XYZ_1(3,1);
center = [X,Y,Z];
[center_lat, center_lon, ~] = XYZtoLLa(center,heading_angles(2),lla0);


xvals = linspace(-sigma_plot,sigma_plot,n_points);
ytop = sqrt(sigma_plot^2-xvals.^2);
ybot = -sqrt(sigma_plot^2-xvals.^2);
error_ellipse = [xvals' + X,ytop' + Y, Z*ones(n_points,1); flip([xvals' + X, ybot' + Y, Z*ones(n_points,1)])];
% error_ellipses{i} = ned2lla(error_ellipses{i},lla0,'flat');
[ee_lat, ee_lon, ee_ref_alt] = XYZtoLLa(error_ellipse,heading_angles(2),lla0);

Xf = mu_1(1,end);
Yf = mu_1(2,end);
Zf = p_XYZ_1(3,end);
sigma_plot = double(sqrt(sigma_1{end}(1)));
xvals = linspace(-sigma_plot,sigma_plot,n_points);
ytop = sqrt(sigma_plot^2-xvals.^2);
ybot = -sqrt(sigma_plot^2-xvals.^2);
error_ellipse = [xvals' + Xf,ytop' + Yf, Z*ones(n_points,1); flip([xvals' + Xf, ybot' + Yf, Zf*ones(n_points,1)])];
% error_ellipses{i} = ned2lla(error_ellipses{i},lla0,'flat');
[ee_lat_f, ee_lon_f, ~] = XYZtoLLa(error_ellipse,heading_angles(2),lla0);




figure()
plot(mu_1(2,:),mu_1(1,:),'b-*','linewidth',1.5)
hold on
plot(mu_1(4,:),mu_1(3,:),'r-*','linewidth',1.5)
plot(p_XYZ_1(:,2),p_XYZ_1(:,1),'g-*','linewidth',1.5)
plot(0,0,'k+','markersize',15,'linewidth',3)
grid on

xlabel('Y (m)','fontweight','bold','FontSize',12)
ylabel('X (m)','fontweight','bold','FontSize',12)
legend('Target Estimate','Drone Estimate','Drone Path','True Target','fontsize',12,'fontweight','bold','location','nw')
title('Hand Test 1 Local Space','fontsize',14,'FontWeight','bold')


figure()
geoplot(lat0,lon0,'y+','MarkerSize',20,'linewidth',2)
hold on
geoplot(lat_t,lon_t,'b-*','linewidth',2)
geoplot(lat_1,lon_1,'g-*','linewidth',1)
geoplot(ee_lat,ee_lon,'c-','linewidth',2)
geoplot(ee_lat_f,ee_lon_f,'m-','linewidth',2)
geobasemap satellite
legend('True Target Location','Target Estimate','Drone Path','Initial Variance','Final Variance','fontsize',12,'fontweight','bold')
title('Hand Test 1 Geoplot','fontsize',14,'FontWeight','bold')

%% Handheld Case 2


[lat_t, lon_t, ~] = XYZtoLLa([mu_2(1,:)',mu_2(2,:)',p_XYZ_2(:,3)],heading_angles(2),lla0);


n_points = 100;
sigma_plot = sqrt(sigma_2{1}(1));

X = mu_2(1,1);
Y = mu_2(2,1);
Z = p_XYZ_2(3,1);
center = [X,Y,Z];
[center_lat, center_lon, ~] = XYZtoLLa(center,heading_angles(2),lla0);


xvals = linspace(-sigma_plot,sigma_plot,n_points);
ytop = sqrt(sigma_plot^2-xvals.^2);
ybot = -sqrt(sigma_plot^2-xvals.^2);
error_ellipse = [xvals' + X,ytop' + Y, Z*ones(n_points,1); flip([xvals' + X, ybot' + Y, Z*ones(n_points,1)])];
% error_ellipses{i} = ned2lla(error_ellipses{i},lla0,'flat');
[ee_lat, ee_lon, ee_ref_alt] = XYZtoLLa(error_ellipse,heading_angles(2),lla0);

Xf = mu_2(1,end);
Yf = mu_2(2,end);
Zf = p_XYZ_2(3,end);
sigma_plot = double(sqrt(sigma_2{end}(1)));
xvals = linspace(-sigma_plot,sigma_plot,n_points);
ytop = sqrt(sigma_plot^2-xvals.^2);
ybot = -sqrt(sigma_plot^2-xvals.^2);
error_ellipse = [xvals' + Xf,ytop' + Yf, Z*ones(n_points,1); flip([xvals' + Xf, ybot' + Yf, Zf*ones(n_points,1)])];
% error_ellipses{i} = ned2lla(error_ellipses{i},lla0,'flat');
[ee_lat_f, ee_lon_f, ~] = XYZtoLLa(error_ellipse,heading_angles(2),lla0);




figure()
plot(mu_2(2,:),mu_2(1,:),'b-*','linewidth',1.5)
hold on
plot(mu_2(4,:),mu_2(3,:),'r-*','linewidth',1.5)
plot(p_XYZ_2(:,2),p_XYZ_2(:,1),'g-*','linewidth',1.5)
plot(0,0,'k+','markersize',15,'linewidth',3)
grid on

xlabel('Y (m)','fontweight','bold','FontSize',12)
ylabel('X (m)','fontweight','bold','FontSize',12)
legend('Target Estimate','Drone Estimate','Drone Path','True Target','fontsize',12,'fontweight','bold','location','nw')
title('Hand Test 2 Local Space','fontsize',14,'FontWeight','bold')


figure()
plot(mu_2(2,:),mu_2(1,:),'b-*','linewidth',1.5)
hold on
plot(mu_2(4,:),mu_2(3,:),'r-*','linewidth',1.5)
plot(p_XYZ_2(:,2),p_XYZ_2(:,1),'g-*','linewidth',1.5)
plot(0,0,'k+','markersize',15,'linewidth',3)
grid on

xlabel('Y (m)','fontweight','bold','FontSize',12)
ylabel('X (m)','fontweight','bold','FontSize',12)
legend('Target Estimate','Drone Estimate','Drone Path','True Target','fontsize',12,'fontweight','bold','location','se')
title('Hand Test 2 Local Space, Zoomed','fontsize',14,'FontWeight','bold')
xlim([-2.2 0.5])
ylim([-14.2,0.2])



figure()
geoplot(lat0,lon0,'y+','MarkerSize',20,'linewidth',2)
hold on
geoplot(lat_t,lon_t,'b-*','linewidth',2)
geoplot(lat_2,lon_2,'g-*','linewidth',1)
geoplot(ee_lat,ee_lon,'c-','linewidth',2)
geoplot(ee_lat_f,ee_lon_f,'m-','linewidth',2)
geobasemap satellite
legend('True Target Location','Target Estimate','Drone Path','Initial Variance','Final Variance','fontsize',12,'fontweight','bold')
title('Hand Test 2 Geoplot','fontsize',14,'FontWeight','bold')


%% Handheld Case 3


[lat_t, lon_t, ~] = XYZtoLLa([mu_3(1,:)',mu_3(2,:)',p_XYZ_3(:,3)],heading_angles(2),lla0);


n_points = 100;
sigma_plot = sqrt(sigma_3{1}(1));

X = mu_3(1,1);
Y = mu_3(2,1);
Z = p_XYZ_3(3,1);
center = [X,Y,Z];
[center_lat, center_lon, ~] = XYZtoLLa(center,heading_angles(2),lla0);


xvals = linspace(-sigma_plot,sigma_plot,n_points);
ytop = sqrt(sigma_plot^2-xvals.^2);
ybot = -sqrt(sigma_plot^2-xvals.^2);
error_ellipse = [xvals' + X,ytop' + Y, Z*ones(n_points,1); flip([xvals' + X, ybot' + Y, Z*ones(n_points,1)])];
% error_ellipses{i} = ned2lla(error_ellipses{i},lla0,'flat');
[ee_lat, ee_lon, ee_ref_alt] = XYZtoLLa(error_ellipse,heading_angles(2),lla0);

Xf = mu_3(1,end);
Yf = mu_3(2,end);
Zf = p_XYZ_3(3,end);
sigma_plot = double(sqrt(sigma_3{end}(1)));
xvals = linspace(-sigma_plot,sigma_plot,n_points);
ytop = sqrt(sigma_plot^2-xvals.^2);
ybot = -sqrt(sigma_plot^2-xvals.^2);
error_ellipse = [xvals' + Xf,ytop' + Yf, Z*ones(n_points,1); flip([xvals' + Xf, ybot' + Yf, Zf*ones(n_points,1)])];
% error_ellipses{i} = ned2lla(error_ellipses{i},lla0,'flat');
[ee_lat_f, ee_lon_f, ~] = XYZtoLLa(error_ellipse,heading_angles(2),lla0);




figure()
plot(mu_3(2,:),mu_3(1,:),'b-*','linewidth',1.5)
hold on
plot(mu_3(4,:),mu_3(3,:),'r-*','linewidth',1.5)
plot(p_XYZ_3(:,2),p_XYZ_3(:,1),'g-*','linewidth',1.5)
plot(0,0,'k+','markersize',15,'linewidth',3)
grid on
xlabel('Y (m)','fontweight','bold','FontSize',12)
ylabel('X (m)','fontweight','bold','FontSize',12)
legend('Target Estimate','Drone Estimate','Drone Path','True Target','fontsize',12,'fontweight','bold','location','sw')
title('Hand Test 3 Local Space','fontsize',14,'FontWeight','bold')



figure()
geoplot(lat0,lon0,'y+','MarkerSize',20,'linewidth',2)
hold on
geoplot(lat_t,lon_t,'b-*','linewidth',2)
geoplot(lat_3,lon_3,'g-*','linewidth',1)
geoplot(ee_lat,ee_lon,'c-','linewidth',2)
geoplot(ee_lat_f,ee_lon_f,'m-','linewidth',2)
geobasemap satellite
legend('True Target Location','Target Estimate','Drone Path','Initial Variance','Final Variance','fontsize',12,'fontweight','bold')
title('Hand Test 3 Geoplot','fontsize',14,'FontWeight','bold')


%% Flight case

[lat_t, lon_t, ~] = XYZtoLLa([mu(1,:)',mu(2,:)',p_XYZ(:,3)],heading_angles(2),lla0);


n_points = 100;
sigma_plot = sqrt(sigma{1}(1));

X = mu(1,1);
Y = mu(2,1);
Z = p_XYZ(3,1);
center = [X,Y,Z];
[center_lat, center_lon, ~] = XYZtoLLa(center,heading_angles(2),lla0);


xvals = linspace(-sigma_plot,sigma_plot,n_points);
ytop = sqrt(sigma_plot^2-xvals.^2);
ybot = -sqrt(sigma_plot^2-xvals.^2);
error_ellipse = [xvals' + X,ytop' + Y, Z*ones(n_points,1); flip([xvals' + X, ybot' + Y, Z*ones(n_points,1)])];
% error_ellipses{i} = ned2lla(error_ellipses{i},lla0,'flat');
[ee_lat, ee_lon, ee_ref_alt] = XYZtoLLa(error_ellipse,heading_angles(2),lla0);

Xf = mu(1,end);
Yf = mu(2,end);
Zf = p_XYZ(3,end);
sigma_plot = double(sqrt(sigma{end}(1)));
xvals = linspace(-sigma_plot,sigma_plot,n_points);
ytop = sqrt(sigma_plot^2-xvals.^2);
ybot = -sqrt(sigma_plot^2-xvals.^2);
error_ellipse = [xvals' + Xf,ytop' + Yf, Z*ones(n_points,1); flip([xvals' + Xf, ybot' + Yf, Zf*ones(n_points,1)])];
% error_ellipses{i} = ned2lla(error_ellipses{i},lla0,'flat');
[ee_lat_f, ee_lon_f, ~] = XYZtoLLa(error_ellipse,heading_angles(2),lla0);




figure()
plot(mu(2,:),mu(1,:),'b-*','linewidth',1.5)
hold on
plot(mu(4,:),mu(3,:),'r-*','linewidth',1.5)
plot(p_XYZ(:,2),p_XYZ(:,1),'g-*','linewidth',1.5)
plot(0,0,'k+','markersize',15,'linewidth',3)
grid on

xlabel('Y (m)','fontweight','bold','FontSize',12)
ylabel('X (m)','fontweight','bold','FontSize',12)
legend('Target Estimate','Drone Estimate','Drone Path','True Target','fontsize',12,'fontweight','bold','location','se')
title('Flight Test Local Space','fontsize',14,'FontWeight','bold')


figure()
plot(mu(2,:),mu(1,:),'b-*','linewidth',1.5)
hold on
plot(mu(4,:),mu(3,:),'r-*','linewidth',1.5)
plot(p_XYZ(:,2),p_XYZ(:,1),'g-*','linewidth',1.5)
plot(0,0,'k+','markersize',15,'linewidth',3)
grid on

xlabel('Y (m)','fontweight','bold','FontSize',12)
ylabel('X (m)','fontweight','bold','FontSize',12)
legend('Target Estimate','Drone Estimate','Drone Path','True Target','fontsize',12,'fontweight','bold','location','se')
title('Flight Test Local Space, Zoomed','fontsize',14,'FontWeight','bold')
xlim([-1.2 1.2])
ylim([-12.5,6])



figure()
geoplot(lat0,lon0,'y+','MarkerSize',20,'linewidth',2)
hold on
geoplot(lat_t,lon_t,'b-*','linewidth',2)
geoplot(lat,lon,'g-*','linewidth',1)
geoplot(ee_lat,ee_lon,'c-','linewidth',2)
geoplot(ee_lat_f,ee_lon_f,'m-','linewidth',2)
geobasemap satellite
legend('True Target Location','Target Estimate','Drone Path','Initial Variance','Final Variance','fontsize',12,'fontweight','bold')
title('Flight Test Geoplot','fontsize',14,'FontWeight','bold')


%% Error Results
ekf_err_1 = vecnorm(mu_1(1:2,:));
ekf_err_2 = vecnorm(mu_2(1:2,:));
ekf_err_3 = vecnorm(mu_3(1:2,:));
ekf_err = vecnorm(mu(1:2,:));

figure()
plot(ekf_err_1,'-','linewidth',2)
hold on
plot(ekf_err_2,'-','linewidth',2)
plot(ekf_err_3,'-','linewidth',2)
plot(ekf_err,'-','linewidth',2)
legend('Held 1','Held 2','Held 3','Flight','fontweight','bold','FontSize',12)
grid on
ylim([0 15])
xlabel('Timestep','fontweight','bold','FontSize',12)
ylabel('Error (m)','fontweight','bold','FontSize',12)
title('||EKF Error||','fontweight','bold','FontSize',14)
yticks([1 2 3 4 5 10 15])


%%
function [lat, lon, ref_alt] = XYZtoLLa(p_XYZ,psi,lla0)

% rotate to NED by positive heading angle
X = p_XYZ(:,2);
p_XYZ(:,2) = -p_XYZ(:,1);
p_XYZ(:,1) = X;
R_XYtoNED = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
for i=1:length(p_XYZ(:,1))
    p_NED(i,:) = R_XYtoNED*p_XYZ(i,:)';

end


lla = ned2lla(p_NED,lla0,'flat');
lat = lla(:,1); lon = lla(:,2); alt = lla(:,3);
geoid_height_N = geoidheight(lat,lon+360);
ref_alt = alt - geoid_height_N;


end