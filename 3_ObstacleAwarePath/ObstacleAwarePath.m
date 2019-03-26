% DroneAutoCapture: Main routine for ObstacleAwarePath stage
% 
% Author: Ojaswa Sharma, <ojaswa@iiitd.ac.in>
%

%% Init
clc; clear; close all;

% GPToolbox
gp_subdirs = split(genpath('../Depends/gptoolbox/'),':');
addpath(strjoin(gp_subdirs(~contains(gp_subdirs,'.git')),':'));
% Fast marching toolbox
addpath('../Depends/toolbox_fast_marching/data');
addpath('../Depends/toolbox_fast_marching');
addpath('../Depends/toolbox_fast_marching/toolbox');
% FM2 toolbox
addpath('../Depends/fm2/algorithms');
addpath('../Depends/fm2/fm2tools');
% Common stuff
addpath('../Include');

building_name = 'Academic';
coverage_type = 'walls';

bufferDist = 8; % 8 meter buffer around floor plan boundary
safeDist = 4; % 4 meter (2*GPS accuracy of 2m) safe distance for computing obstacle aware path 
UTM_offset = [722338.000 3159808.000 398.000];

[g_altitudes_drone, g_altitudes_floors, groundMeshZ, buildingHeight] = getBuildingParameters(building_name, coverage_type);
g_altitudes_mesh = groundMeshZ + g_altitudes_drone; % Altitudes at which to section the mesh

infile_obstacle = ['../Data/Obstacle data/', building_name, '/obstacle_'];
infile_boundary = ['../Data/Plan boundaries/', building_name, '/geoBF'];
outfile_dronesafepath = ['../Data/Drone path/', building_name, '/safepath_'];

%% Read obstacle data
g_obstacles = {};
for k=1:length(g_altitudes_mesh)
    filename = [infile_obstacle coverage_type  num2str(g_altitudes_floors(k)) '_' num2str(g_altitudes_drone(k)) 'm.txt'];
    fid = fopen(filename, 'r');
    tline = fgetl(fid);
    x = [];
    y = [];
    i=1;
    obstacle = {};
    while ischar(tline)
        x = sscanf(tline, '%f,');
        tline = fgetl(fid);
        y = sscanf(tline, '%f,');
        obstacle{i} = [x, y];
        i=i+1;
        tline = fgetl(fid);
    end
    fclose(fid);
    g_obstacles{k} = obstacle;
end

%% Read boundaries and generate offset path from building boundary
g_boundary = {};
g_offset = {};
scale_to_int = 2^15; % A large scale

for i = 1: numel(g_altitudes_mesh)
    infile = [infile_boundary num2str(g_altitudes_floors(i)) '.dxf'];
    
    % Load boundary of largest cross section
    [c_Line,c_Poly,c_Cir,c_Arc,c_Poi] = f_LectDxf(infile);
    boundary = c_Poly{1} - UTM_offset(1,1:2);

    in_pts.x = int64(scale_to_int*boundary(:,1));
    in_pts.y = int64(scale_to_int*boundary(:,2));
    orientation = clipper(in_pts);
    if orientation == 0 % Orientation is clockwise, reverse it.
        boundary = flipud(boundary);
        in_pts.x = flipud(in_pts.x);
        in_pts.y = flipud(in_pts.y);
    end
    out_pts = clipper(in_pts, bufferDist*scale_to_int, 0);
    g_buffer{i} = double([out_pts(1).x out_pts(1).y])/scale_to_int;
    g_boundary{i} = boundary;
end

%% create a offset around obstacles 'safeDist' and clip the drone Path against it.
% Store unsafe paths as end-point indices into the g_buffer
% NOTE: THIS BLOCK MODIFIES g_buffer VARIABLE!
in_pts = [];
g_inflatedObstacles = {};
g_clippedPathIndices = {};
for i=1:numel(g_altitudes_mesh)
    % Inflate obstacles by safeDist
   obstacle = g_obstacles{i};
   in_pts = [];
   for j=1:size(obstacle, 2)
       loop = obstacle{j};
       in_pts(j).x = int64(scale_to_int*loop(:,1));
       in_pts(j).y = int64(scale_to_int*loop(:,2));
   end
   out_pts = clipper(in_pts, safeDist*scale_to_int, 0);
   inflatedObs = {};
   for j=1:size(out_pts, 2)
       loop = [out_pts(j).x/scale_to_int, out_pts(j).y/scale_to_int];
       inflatedObs{j} = loop;
   end
   g_inflatedObstacles{i} = inflatedObs;
   
   % Intersect bufferPath with inflated obstacles
   buffer = g_buffer{i};
   polyObs = inflatedObs{1};
   for j=2:size(inflatedObs, 2)
       polyObs = [polyObs; nan nan; inflatedObs{j}];
   end
   
   tic; pathxObs = InterX([buffer; buffer(1,:)]', polyObs'); toc;
   if isempty(pathxObs)
       g_clippedPathIndices{i} = []; % no inersection with obstacles. Path is clear
       continue;
   end
   [buffer2, xidx] = mergeIntersectionsInPath(buffer, pathxObs');
   xidx = sort(xidx); % sort the intersection point indices to make arcs
   
   % Test if the first segment is inside or outside obstacles
   if(xidx(2) - xidx(1) > 1) % This arc is non-linear
       testpoint = buffer2(round(mean(xidx(1:2))),:); % A point on the segment
   else % This arc is a linear edge
       testpoint = mean(buffer2(xidx(1:2),:)); % A point on the segment
   end
   testin = inpolygon(testpoint(1), testpoint(2), polyObs(:,1), polyObs(:,2));
   if ~testin % test failed. shift indices by 1
       xidx = circshift(xidx, -1);
   end
   
   % Store path endpoints that lie in obstacle zone. Also update buffer path
   g_clippedPathIndices{i} = reshape(xidx, 2,[])';
   g_buffer{i} = buffer2;
end

%% Create obstacle maps
% Get bbox and create placeholder for maps
fprintf('Creating obstacle maps ... ');
mini = [Inf; Inf];
maxi = [-Inf; -Inf];
for i=1:numel(g_altitudes_mesh)
    mini = min(mini, min(reshape(cell2mat(cellfun(@min, g_obstacles{i}, 'UniformOutput', false)), 2, []), [], 2));
    maxi = max(maxi, max(reshape(cell2mat(cellfun(@max, g_obstacles{i}, 'UniformOutput', false)), 2, []), [], 2));
    mini = min(mini, min(g_buffer{i})' - safeDist);% Buffer *must* be part of the bbox
    maxi = max(maxi, max(g_buffer{i})' + safeDist);% Buffer *must* be part of the bbox
end
maxres = 500;
delta = max(maxi - mini)/(maxres-1);
xvec = mini(1):delta:maxi(1);
yvec = mini(2):delta:maxi(2);
[XX, YY] = meshgrid(xvec, yvec);

gpuXX = gpuArray(XX);
gpuYY = gpuArray(YY);
g_obstacleMaps = zeros([size(XX), numel(g_altitudes_mesh)], 'logical');
% Populate maps
for i=1:numel(g_altitudes_mesh)
    tic
    obstacles = g_obstacles{i};
    poly = obstacles{1};
    for j=2:size(obstacles, 2)
        poly = [poly; nan nan; obstacles{j}];
    end
    gpuInpoly = inpolygon(gpuXX, gpuYY, poly(:,1), poly(:,2));toc;
    time_elapsed = toc;
    fprintf('([%d] in %f sec. ', i, time_elapsed);
    g_obstacleMaps(:,:,i) = ~gather(gpuInpoly);
end
fprintf('\n');
%% Create safe path between clipped curve end-points that avoid obstacles
% A. Valero, J.V. G?mez, S. Garrido and L. Moreno, The Path to Efficiency: 
% Fast Marching Method for Safer, More Efficient Mobile Robot Trajectories, 
% IEEE Robotics and Automation Magazine, Vol. 20, No. 4, 2013.
[XI, YI] = meshgrid(1:numel(xvec), 1:numel(yvec));
II = sub2ind(size(XI), YI, XI); % YI - row indices, XI - col indices
%h = [];
% Generate safe paths
g_safePaths = {};
for i=1:numel(g_altitudes_mesh)
    buffer = g_buffer{i}; % drone trajectory
    clipEndIdx = g_clippedPathIndices{i};
    %h(2*i-1) = figure; hold on; axis equal; set(gca,'YDir','normal');
    h(2*i) = figure; hold on; axis equal;
    obstacle = g_obstacles{i};
    for j=1:size(obstacle, 2)
        loop  = obstacle{j};
        fill(loop(:,1), loop(:,2), 'c', 'EdgeColor', 'None');
    end
    plot(buffer(:,1), buffer(:,2), 'b-');
    plot(buffer(clipEndIdx(:),1), buffer(clipEndIdx(:),2), 'ko');
    %figure(h(2*i-1));
    %imagesc(g_obstacleMaps(:,:,i));
    safePaths = {};
    for j=1:size(clipEndIdx, 1)
        clipEndCoord = buffer(clipEndIdx(j, :),:); % Actual coordinates
        clipEndCoordRaster = interp2(XX, YY, II, clipEndCoord(:,1), clipEndCoord(:,2), 'nearest'); % Pixel coordinate
        [ii, jj] = ind2sub(size(II), clipEndCoordRaster);
        startPt = [jj(1); ii(1)]; % [x; y]
        stopPt = [jj(2); ii(2)];
        [F, T, path, vels, times] = FM2(g_obstacleMaps(:,:,i), 0.1, startPt, stopPt);
        pathCoords = pixelToMap([1 1], fliplr(size(II)), mini', maxi', path');
        pathCoordsSmooth = snapPathToEnds(clipEndCoord, pathCoords);
        safePaths{j} = pathCoordsSmooth;
        %figure(h(2*i-1)); plot(path(1,:), path(2,:), 'r-');
        figure(h(2*i)); plot(pathCoordsSmooth(:,1), pathCoordsSmooth(:,2), 'r-');
    end
    g_safePaths{i} = safePaths;
end

%% Replace unsafe paths in the original buffer path with safe paths and save to disk
%Splice
g_safeBuffer = {};
for i=1:numel(g_altitudes_mesh)
    buffer = g_buffer{i};
    safeBuffer = [];
    clipEndIdx = g_clippedPathIndices{i};
    if ~isempty(clipEndIdx)
        keepIdx = [circshift(clipEndIdx(:,2), 1) clipEndIdx(:,1)];
        safePaths = g_safePaths{i};
        for j=1:size(keepIdx, 1)
            if keepIdx(j,1) < keepIdx(j,2)
                safeBuffer = [safeBuffer; buffer(keepIdx(j,1):keepIdx(j,2), :); safePaths{j}(2:end-1, :)];
            else
                safeBuffer = [safeBuffer; buffer([keepIdx(j,1):end, 1:keepIdx(j,2)], :); safePaths{j}(2:end-1, :)];
            end
        end
    else
        safeBuffer = buffer;
    end
    g_safeBuffer{i} = safeBuffer;
    figure; hold on; axis equal;
    plot(safeBuffer(:,1), safeBuffer(:,2), 'b-');
end

%Write out safe paths - in UTM subtracted coordinates.
for i=1:numel(g_altitudes_mesh) 
    filename = [outfile_dronesafepath coverage_type num2str(g_altitudes_floors(i)) '_' num2str(g_altitudes_drone(i)) 'm.txt'];
    safeBuffer = g_safeBuffer{i};
    fid = fopen(filename, 'w');
    fprintf(fid, '%f', safeBuffer(1,1));
    fprintf(fid, ',%f', safeBuffer(2:end,1));
    fprintf(fid, '\n');
    fprintf(fid, '%f', safeBuffer(1,2));
    fprintf(fid, ',%f', safeBuffer(2:end,2));
    fprintf(fid, '\n');    
    fclose(fid);
end    
