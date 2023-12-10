

function [yaw_angle] = get_heading(tlog_file, lidar_file)
    % Load fused GPS data
    dialect = mavlinkdialect("./ardupilotmega.xml");
    tlogReader = mavlinktlog("logs/" + tlog_file + ".tlog",dialect);
    gps_fused = readmsg(tlogReader,"MessageName","GLOBAL_POSITION_INT").Messages{1};
    heading = double(gps_fused.hdg)/100;


    lat = double(gps_fused.lat)./(1E7);
    lon = double(gps_fused.lon)./(1E7);
    alt = double(gps_fused.alt(77:2116))./1E3;
    t_gps = gps_fused.Time;
    

   
    
    % Load lidar data
    lidar_data = readtimetable("logs/" + lidar_file + ".csv");
    t_lidar = lidar_data.Time;
    range = lidar_data.Var1;
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
    lidar_range = nan*ones(1,length(t_meas));
    for i=1:length(t_meas)
        tdiff = milliseconds(t_meas(i) - t_lidar);
        range_idx = find(abs(tdiff) == min(abs(tdiff)));
        lidar_range(i) = range(range_idx(1));
  
    
    
    end
    
    good_idxs = ~(lidar_range == 65535);
    
    yaw_angle = mean(heading(t_start_idx:t_stop_idx));

end



