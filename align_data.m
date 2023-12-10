

function [t_meas, lidar_range, p_XYZ,v, lat, lon, sigma_sq_p] = align_data(tlog_file, lidar_file, lla0, psi)
    % Load fused GPS data
    dialect = mavlinkdialect("./ardupilotmega.xml");
    tlogReader = mavlinktlog("logs/" + tlog_file + ".tlog",dialect);
    gps_fused = readmsg(tlogReader,"MessageName","GLOBAL_POSITION_INT").Messages{1};
    ekf = readmsg(tlogReader,"MessageName","EKF_STATUS_REPORT").Messages{1};
    lat = double(gps_fused.lat)./(1E7);
    lon = double(gps_fused.lon)./(1E7);
    sigma_sq_p = ekf.pos_horiz_variance;
    v = [double(gps_fused.vx)./100, double(gps_fused.vy)./100];
    ref_alt = double(gps_fused.alt)./1E3;
    t_gps = gps_fused.Time;
    
    geoid_height_N = geoidheight(lat,lon+360);
    alt = ref_alt - geoid_height_N;

    p_NED = lla2ned([lat,lon,alt],lla0,'flat');
    p_XYZ = 0.*p_NED;
    
    R_NEDtoXY = [cos(-psi) sin(-psi) 0; -sin(-psi) cos(-psi) 0; 0 0 1];
    for i=1:length(p_NED)
        p_XYZ(i,:) = R_NEDtoXY*p_NED(i,:)';

    end
    
    
    % Load lidar data
    
    lidar_data = readtimetable("logs/" + lidar_file + ".csv");

    trim_end = 200;

    if lidar_file == "hold_1"
        trim_end = 800;
    end
    if lidar_file == "hold_2"
        trim_end = 1500;
    end

    if lidar_file == "flight"
        trim_end = 300;
    end

    
    t_lidar = lidar_data.Time(100:end-trim_end); %Trim 5s and 10s from each end
    range = lidar_data.Var1(100:end-trim_end);
    signal_strength = lidar_data.Var2;
    
    % Format lidar time
    ref_day = t_gps(1);
    day_of_data = datetime(ref_day.Year,ref_day.Month,ref_day.Day);
    t_lidar = t_lidar + day_of_data;
    t_lidar.TimeZone = t_gps.TimeZone;
    t_lidar.Format = t_gps.Format;
    
    
    tdiff_start = milliseconds(t_lidar(1)-t_gps);
    t_start_idx = find(abs(tdiff_start) == min(abs(tdiff_start))); % start idx for gps
    tdiff_stop = milliseconds(t_lidar(end)-t_gps);
    t_stop_idx = find(abs(tdiff_stop) == min(abs(tdiff_stop))); % stop idx for gps
    
    % Not pretty way to snap lidar measurements to gps measurements
    t_meas = t_gps(t_start_idx:t_stop_idx);
    lat = lat(t_start_idx:t_stop_idx);
    lon = lon(t_start_idx:t_stop_idx);
    p_XYZ = p_XYZ(t_start_idx:t_stop_idx,:);
    v = v(t_start_idx:t_stop_idx,:);
    sigma_sq_p = sigma_sq_p(t_start_idx:t_stop_idx,:);

    lidar_range = nan*ones(1,length(t_meas));
    for i=1:length(t_meas)
        tdiff = milliseconds(t_meas(i) - t_lidar);
        range_idx = find(abs(tdiff) == min(abs(tdiff)));
        lidar_range(i) = range(range_idx(1));
  
    
    
    end
    
    good_idxs = ~(lidar_range == 65535);
    
    
    t_meas = t_meas(good_idxs);
    lidar_range = lidar_range(good_idxs)';
    p_XYZ = p_XYZ(good_idxs,:);
    X = p_XYZ(:,2);
    p_XYZ(:,2) = p_XYZ(:,1);
    p_XYZ(:,1) = -X;
    lat = lat(good_idxs);
    lon = lon(good_idxs);
    v = v(good_idxs,:);
    sigma_sq_p = sigma_sq_p(good_idxs);
end



