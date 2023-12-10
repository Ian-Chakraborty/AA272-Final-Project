close all
clear
clc
%%
% Creates subtitle file for overlaying lidar data text on video


[t_meas, lidar_range, p_ecef, lat, lon] = align_data("video", "flight7");
lidar_range(lidar_range < 100) = NaN;
lidar_range(lidar_range > 1900) = NaN;
delta_t_ms = milliseconds(t_meas - t_meas(1));
delta_t  = datetime(0,0,0,0,0,0,delta_t_ms);

delta_t.Format = 'mm:ss,SSS';

end_idx = 79;
for i = 1:end_idx

    if isnan(lidar_range(i))
        range = "NaN";
        caption = "RANGE: " + string(range) + '\n';
    else
        range = lidar_range(i)/100;
        caption = "RANGE: " + string(range) + " m" + '\n';
    end
    
    
    lines(i,1) = string(i) + '\n'  + "00:" + string(delta_t(i)) + " --> " + "00:" +string(delta_t(i+1)) + '\n' + caption;

end

fileID = fopen('captions2.srt','w');
for i=1:length(lines)

    fprintf(fileID,lines(i,1) + '\n');

end

fclose(fileID);