function theta = azimuth(lat1,lon1,lat2,lon2)
% Calculate bearing in degrees from (lat1,lon1) -> (lat2, lon2)
% lat1 = GEODETIC latitude of first point (degrees)
% lon1 = longitude of first point (degrees)
% lat2, lon2 = second point (degrees)
% theta is headings from True North in [0, 360]
% Original algorithm source:
% T. Vincenty, "Direct and Inverse Solutions of Geodesics on the Ellipsoid
% with Application of Nested Equations", Survey Review, vol. 23, no. 176,
% April 1975, pp 88-93
% https://in.mathworks.com/matlabcentral/fileexchange/5379-geodetic-distance-on-wgs84-earth-ellipsoid

% Input check:
if abs(lat1)>90 || abs(lat2)>90
    error('Input latitudes must be between -90 and 90 degrees, inclusive.')
end
% Supply WGS84 earth ellipsoid axis lengths in meters:
a = 6378137; % definitionally
b = 6356752.31424518; % computed from WGS84 earth flattening coefficient definition
% convert inputs in degrees to radians:
lat1 = lat1 * 0.0174532925199433;
lon1 = lon1 * 0.0174532925199433;
lat2 = lat2 * 0.0174532925199433;
lon2 = lon2 * 0.0174532925199433;
% correct for errors at exact poles by adjusting 0.6 millimeters:
if abs(pi/2-abs(lat1)) < 1e-10
    lat1 = sign(lat1)*(pi/2-(1e-10));
end
if abs(pi/2-abs(lat2)) < 1e-10
    lat2 = sign(lat2)*(pi/2-(1e-10));
end
f = (a-b)/a;
U1 = atan((1-f)*tan(lat1));
U2 = atan((1-f)*tan(lat2));
lon1 = mod(lon1,2*pi);
lon2 = mod(lon2,2*pi);
L = abs(lon2-lon1);
if L > pi
    L = 2*pi - L;
end
%%%lambda = L;
lambda = lon2 - lon1;
% From point #1 to point #2
a12 = atan2(cos(U2)*sin(lambda),cos(U1)*sin(U2)-sin(U1)*cos(U2)*cos(lambda));
%?a12 = atan2(cos(lat2)*sin(lambda),cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lambda));
%%%if a12 < 0
%%%    a12 = a12+2*pi;
%%%end

theta = mod(a12*180/pi, 360);