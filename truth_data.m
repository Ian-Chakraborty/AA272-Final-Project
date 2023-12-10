
function [lat, lon, ref_alt, x_ecef, y_ecef, z_ecef] = truth_data(file)

tlogReader = mavlinktlog("logs/" + file + ".tlog");

gps_fused = readmsg(tlogReader,"MessageName","GLOBAL_POSITION_INT").Messages{1};
lat = double(gps_fused.lat)./(1E7);
lon = double(gps_fused.lon)./(1E7);
alt = double(gps_fused.alt)./1E3;

alt = mean(alt);
lat = mean(lat);
lon = mean(lon);

lon_wrapped = lon;
lon_wrapped(lon_wrapped < 0) = lon_wrapped(lon_wrapped < 0) + 360; 
geoid_height_N = geoidheight(lat,lon_wrapped);
ref_alt = alt - geoid_height_N;

p_ECEF = lla2ecef([lat,lon,ref_alt]);

x_ecef  = p_ECEF(1); y_ecef = p_ECEF(2); z_ecef = p_ECEF(3);


end
