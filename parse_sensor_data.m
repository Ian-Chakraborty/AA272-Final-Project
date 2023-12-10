close all
clear
clc
%%


heading_angles = (307 + [16.0287,32.8887,18.0606,16.0287+20.7387])*pi/180;

[lat0, lon0, ref_alt, ~, ~, ~] = truth_data("table_truth");
geoid_height_N = geoidheight(lat0,lon0+360);
alt0 = ref_alt - geoid_height_N;
lla0 = [lat0,lon0,alt0];


[t_meas, lidar_range, p_XYZ, v, lat, lon, sigma_sq_p] = align_data("video", "flight", lla0, heading_angles(4));
[t_meas_1, lidar_range_1, p_XYZ_1, v_1, lat_1, lon_1, sigma_sq_p_1] = align_data("table_hold", "hold_1", lla0, heading_angles(1));
[t_meas_2, lidar_range_2, p_XYZ_2, v_2, lat_2, lon_2, sigma_sq_p_2] = align_data("table_hold", "hold_2", lla0, heading_angles(2));
[t_meas_3, lidar_range_3, p_XYZ_3, v_3, lat_3, lon_3, sigma_sq_p_3] = align_data("table_hold", "hold_3", lla0, heading_angles(3));
