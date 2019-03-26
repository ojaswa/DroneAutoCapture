% DroneAutoCapture: Main routine for WaypointsRoof stage
% 
% Author: Ojaswa Sharma, <ojaswa@iiitd.ac.in>
%

%% Init
clc; clear; close all;

% GPToolbox
gp_subdirs = split(genpath('../Depends/gptoolbox/'),':');
addpath(strjoin(gp_subdirs(~contains(gp_subdirs,'.git')),':'));
% Common
addpath('../Include');

building_name = 'Academic';
coverage_type = 'roof';

% Constants
MAX_DJI_WAYPOINTS = 99; % Hardware limit
horizHalfFOV = 40.626127*pi/180;
vertHalfFOV = 32.758089*pi/180;
bufferDist = 8; % Capture from 8 meter above the roof
UTM_offset = [722338.000 3159808.000 398.000];
[g_altitudes_drone, g_altitudes_floors, groundMeshZ, buildingHeight, dummyPosition] = getBuildingParameters(building_name, coverage_type);
g_altitudes_mesh = groundMeshZ + g_altitudes_drone; % Altitudes at which to section the mesh

% Fix a dummy position where the drone goes first before hitting the first capture point
dummyPosition = dummyPosition - UTM_offset(1:2);


infile_roof = ['../Data/Plan boundaries/', building_name ,'/geoBF1.dxf']; % Plan with maximum coverage
%infile_obstacle = ['../Data/Obstacle data/', building_name ,'/obstacle_roof1_' g_altitudes_drone 'm.txt'];
outfile_waypoints = ['../Data/Drone path/', building_name, '/waypoints_roof1_' num2str(g_altitudes_drone) 'm.txt']; 
% Note: for roof, we generate only the obstacle polygons and not a obstacle
% free polygon since the later get erroneous because of no building at that
% height.
% Below here, we *must* visually check if obstacles occur inside the
% coverage area

%% Read boundary, and obstacles data from file
% Load boundary of roof
[c_Line,c_Poly,c_Cir,c_Arc,c_Poi] = f_LectDxf(infile_roof);
g_boundary =  c_Poly{1} - UTM_offset(1,1:2);
if(~isCCW(g_boundary)) % Reorient to CCW if not already.
    g_boundary = flipud(g_boundary);
end

% Read obstacles
% fid = fopen(infile_obstacle, 'r');
% tline = fgetl(fid);
% x = [];
% y = [];
% i=1;
% g_obstacles = {};
% while ischar(tline)
%     x = sscanf(tline, '%f,');
%     tline = fgetl(fid);
%     y = sscanf(tline, '%f,');
%     g_obstacles{i} = [x, y];
%     i=i+1;
%     tline = fgetl(fid);
% end
% fclose(fid);

%% Gnerate sample points inside boundig box
% Calculate OBB for boundary polygon
obb = minBoundingBox(g_boundary')';
minHCoverage = 2*bufferDist*tan(horizHalfFOV);
minVCoverage = 2*bufferDist*tan(vertHalfFOV);
overlap = 0.5;
deltaH = minHCoverage*(1 - overlap);
deltaV = minVCoverage*(1 - overlap);
[~, i] = min(sqrt(sum((obb - dummyPosition).^2, 2)));
iprev = (1 + mod(i-1-1, 4));
inext = (1 + mod(i-1+1, 4));
vec1 = obb(iprev,:) - obb(i,:);
vec2 = obb(inext,:) - obb(i,:);
dist1 = sqrt(dot(vec1,vec1));
dist2 = sqrt(dot(vec2, vec2));
vec1 = vec1/dist1;
vec2 = vec2/dist2;
%Do we need any affinity of V/H of the camera with these vectors? 
%i.e, should the camera be oriented preferentially along vec 1 or vec2?
% I think we really don;t care!
n_svec = numel(0:deltaH:dist1); % Get the minimum number of elements along an edge
n_tvec = numel(0:deltaV:dist2); 

if mod(dist1, deltaH) > 0 % Increment by one if dist1 doesn't fall on boundary
    n_svec = n_svec + 1;
end
if mod(dist2, deltaV) > 0
    n_tvec = n_tvec + 1;
end

if mod(n_svec,2) == 1 % Increment by one if not even. We want the drone to return to the same side where it started.
    n_svec = n_svec+1;
end

if mod(n_svec,2) == 1
    n_tvec = n_tvec+1;
end

svec = linspace(0, dist1, n_svec);
tvec = linspace(0, dist2, n_tvec);

[SS, TT] = meshgrid(svec, tvec);
TT(:,2:2:end) = flipud(TT(:,2:2:end));
p0 = obb(i,:);
pts = p0 + SS(:)*vec1  + TT(:)*vec2;

%% Clip points to lie within the area
% Generate clip polygon
offsetDist = 2; % 2m area around boundary
scale_to_int = 2^15; % A large scale
in_pts.x = int64(scale_to_int*g_boundary(:,1));
in_pts.y = int64(scale_to_int*g_boundary(:,2));
out_pts = clipper(in_pts, offsetDist*scale_to_int, 0);
g_inflatedBoundary = [out_pts(1).x/scale_to_int, out_pts(1).y/scale_to_int]; 
colnum_mat = repmat(1:size(SS, 2),size(SS, 1), 1);
in = inpolygon(pts(:,1), pts(:,2), g_inflatedBoundary(:,1), g_inflatedBoundary(:,2));
camPositions = pts(in, :);
pts_colnum = colnum_mat(in);
if size(camPositions, 1) >MAX_DJI_WAYPOINTS
    error('Number of generated waypoints is more than %d. Try reducing image overlap.\n', MAX_DJI_WAYPOINTS);
end

%% Computing heading rotations
nzRotIdx = []; %Indices of points that'll have non-zero head rotation
for i=1:size(SS, 2)
   q = find(pts_colnum == i);
   if ~isempty(q)
       idx = min(q);% First in that row
       nzRotIdx = [nzRotIdx; idx];
   end
end

headRotations = zeros(size(camPositions, 1),1);
% First point needs to be rotated w.r.t. the dummyPosition
dir = vec2;
motion = normalize(camPositions(1,:) - dummyPosition);
headRotations(1) = vectorAngle(motion, dir)*180/pi;
for i=2:numel(nzRotIdx)
   idx = nzRotIdx(i);
   dir = vec2;
   if mod(pts_colnum(idx),2) ==0 % For odd rows, reverse direction
       dir = -dir;
   end
   p = camPositions(idx,:);
   p_prev = camPositions(idx-1,:); % idx should always be \in [2, N-1], so this is safe.
   motion = normalize(p - p_prev);
   headRotations(idx) = vectorAngle(motion, dir)*180/pi;
end

q = find(headRotations > 180);
headRotations(q) = headRotations(q) - 360; % In the range [-180, 180]
%% Convert camera directions to yaw angles (from True North)
% Important: Replace '43 R' with you UTM zone.
[lat, lon] = utm2deg(dummyPosition(1) + UTM_offset(1), dummyPosition(2) + UTM_offset(2), '43 R');
dummyPositionWGS84 = [lat, lon];
[lat, lon] = utm2deg(camPositions(:, 1) + UTM_offset(1), camPositions(:, 2) + UTM_offset(2), repmat('43 R', size(camPositions, 1), 1));
latlonsegs = zeros(size(camPositions, 1), 4);
latlonsegs(1,1:2) = dummyPositionWGS84;
latlonsegs(2:end,1:2) = [lat(1:end-1), lon(1:end-1)];
latlonsegs(:,3:4) = [lat, lon]; 
camPositionsWGS84 = [lat, lon];

headings = -headRotations;%Convert to clockwise +ve
q = headings < 0;
headings(q) = 360 + headings(q);% Convert from [-180, 180] to [0, 360]

% Compute azimuths of current aircraft heading
azimuths = zeros(size(camPositions,1), 1);
for j=1:size(camPositions,1)
    azimuths(j) = azimuth(latlonsegs(j, 1), latlonsegs(j, 2), latlonsegs(j,3), latlonsegs(j, 4));
end

% Compute Yaw values for camera
yaw = mod(headings + azimuths, 360);
q = yaw > 180;
yaw(q)  = yaw(q) - 360; % Convert to [-180, 180] range
camYaw = yaw;
%% Writeout waypoints
% Coordinates in lat-long, angles in degrees, and distances are in meters
% First line specifies height of the mission in meters from ground level
% Format: bool:isdummy, bool:isrepeat, float:lat, float:lon, float:heading
% if isdummy is true (1), heading is not specified
% if isrepeat is true (1), lat, lon are not specified
fid = fopen(outfile_waypoints, 'w');
fprintf(fid, '%f\n', g_altitudes_drone);
% Write out the first dummy point (for known positioining of the drone.
fprintf(fid, '%d %d %.15f %.15f\n', true, false, dummyPositionWGS84(1), dummyPositionWGS84(2));
for j=1:size(camPositionsWGS84, 1)
    fprintf(fid, '%d %d %.15f %.15f %.3f\n', false, false, camPositionsWGS84(j,1), camPositionsWGS84(j,2), camYaw(j));       
end
fclose(fid);

%% Plot
[nlat, nlon] = PositionFromAzimuth(camPositionsWGS84(:,1), camPositionsWGS84(:,2), 0);
[plat, plon] = PositionFromAzimuth(camPositionsWGS84(:,1), camPositionsWGS84(:,2), camYaw);
[nX, nY, ~] = deg2utm(nlat, nlon);
[pX, pY, ~] = deg2utm(plat, plon);
[camX, camY, ~] = deg2utm(camPositionsWGS84(:,1), camPositionsWGS84(:,2));
nX = nX - UTM_offset(1); nY = nY - UTM_offset(2);
pX = pX - UTM_offset(1); pY = pY - UTM_offset(2);
camX = camX - UTM_offset(1); camY = camY - UTM_offset(2);
nnorms = [nX - camX, nY - camY];
ndist = sqrt(dot(nnorms, nnorms, 2));
nnorms = nnorms./ndist;
pnorms = [pX - camX, pY - camY];
pd = sqrt(dot(pnorms, pnorms, 2));
pnorms = pnorms./pd;

figure('units','normalized','outerposition',[0 0 1 1]); % A maximized figure window
hold on; axis equal
fill(g_inflatedBoundary(:,1), g_inflatedBoundary(:,2), [0.8, 0.8, 0.8]);
plot(g_boundary(:,1), g_boundary(:,2), 'b-');
% for i=1:size(g_obstacles,2)
%     loop = g_obstacles{i};
%     fill( loop(:,1), loop(:,2), [0.5, 0.5, 0.5]);
% end
obb_l = [obb; obb(1,:)];
plot(obb_l(:,1), obb_l(:,2), 'b:');
plot(dummyPosition(1), dummyPosition(2), 'k*')

plot(camX, camY, 'b-', 'LineWidth', 2);
text(camX, camY, num2str([1:size(camX, 1)]'));

plotNormals([camX, camY], nnorms, 'k');
plotNormals([camX, camY], pnorms, 'r');
%plot(pts_in(nzRotIdx,1), pts_in(nzRotIdx,2), 'r*');
%text(pts_in(:,1), pts_in(:,2), num2str(pts_colnum));
%text(pts_in(:,1), pts_in(:,2), num2str([1:numel(pts_colnum)]'));
filename = [outfile_waypoints 'roof1_' num2str(g_altitudes_drone) 'm.png'];
saveas(gcf, filename)
close;
