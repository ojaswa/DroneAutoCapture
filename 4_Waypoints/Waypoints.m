% DroneAutoCapture: Main routine for Waypoints stage
% 
% Author: Ojaswa Sharma, <ojaswa@iiitd.ac.in>
%

%% Init
clc; clear; close all;

% GPToolboxx
gp_subdirs = split(genpath('../Depends/gptoolbox/'),':');
addpath(strjoin(gp_subdirs(~contains(gp_subdirs,'.git')),':'));
% Commonn
addpath('../Include');

building_name = 'Academic';
coverage_type = 'walls';

% Constants
horizHalfFOV = 40.626127*pi/180;
bufferDist = 8; % 8 meter buffer around floor plan boundary. 
% Note: bufferDist is actually variable now with obstacle free path. Fix it?
UTM_offset = [722338.000 3159808.000 398.000];
[g_altitudes_drone, g_altitudes_floors, groundMeshZ, buildingHeight, dummyPosition] = getBuildingParameters(building_name, coverage_type);
g_altitudes_mesh = groundMeshZ + g_altitudes_drone; % Altitudes at which to section the mesh

% Fix a dummy position where the drone goes first before hitting the first capture point
dummyPosition = dummyPosition - UTM_offset(1:2); 

infile_boundary = ['../Data/Plan boundaries/', building_name, '/geoBF'];
infile_obstacle = ['../Data/Obstacle data/', building_name, '/obstacle_'];
infile_dronesafepath = ['../Data/Drone path/', building_name, '/safepath_'];
outfile_waypoints = ['../Data/Drone path/', building_name, '/waypoints_'];

%% Read boundary, obstacles, and drone path data from file
g_boundary = {};
for i = 1: numel(g_altitudes_mesh)
    filename = [infile_boundary num2str(g_altitudes_floors(i)) '.dxf'];
    % Load boundary of largest cross section
    [c_Line,c_Poly,c_Cir,c_Arc,c_Poi] = f_LectDxf(filename);
    l_boundary =  c_Poly{1} - UTM_offset(1,1:2);
    if(~isCCW(l_boundary)) % Reorient to CCW if not already.
        l_boundary = flipud(l_boundary);
    end
    g_boundary{i} = l_boundary;
end

g_obstacles = {};
for k=1:length(g_altitudes_mesh)
    filename = [infile_obstacle coverage_type num2str(g_altitudes_floors(k)) '_' num2str(g_altitudes_drone(k)) 'm.txt'];
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

g_safeBuffer = {};
for i=1:numel(g_altitudes_mesh) 
    filename = [infile_dronesafepath coverage_type num2str(g_altitudes_floors(i)) '_' num2str(g_altitudes_drone(i)) 'm.txt'];
    fid = fopen(filename, 'r');
    x = sscanf(fgetl(fid), '%f,');
    y = sscanf(fgetl(fid), '%f,');
    fclose(fid);
    l_safeBuffer = [x, y];
    if(~isCCW(l_safeBuffer)) % Reorient to CCW if not already.
        l_safeBuffer = flipud(l_safeBuffer);
    end    
    g_safeBuffer{i} = l_safeBuffer;
end

%% Solve optimization
g_waypoints = {};
for i=1:numel(g_altitudes_mesh)
    [camPositions, camNormals, camPositionRepeats] = optimizeWaypoints(g_boundary{i}(1:end-1,:), g_safeBuffer{i}, bufferDist, horizHalfFOV);
    waypoints.camPositions = camPositions;
    waypoints.camNormals = camNormals;
    waypoints.camPositionRepeats = camPositionRepeats;
    g_waypoints{i} = waypoints;
end

%% Insert dummy points in-between waypoints to stay the drone on safe path 
% (i.e., avoid obstacle while transiting from one waypoint to another)
% Reorder waypoints such that the first waypoint in the list is closest to the dummy Position
for i=1:numel(g_altitudes_mesh)
    camPositions = g_waypoints{i}.camPositions;
    camNormals = g_waypoints{i}.camNormals; % This is absolute direction
    camPositionRepeats = g_waypoints{i}.camPositionRepeats;
    vec2dummy = camPositions - dummyPosition;
    dist2dummy = sqrt(dot(vec2dummy, vec2dummy, 2));
    [~, minIdx] = min(dist2dummy); 
    minIdx = min(minIdx); % minIdx could potentially be a vector, choose the minimum index.
    n = size(camPositions, 1);
    reindex = [];
    reindex(circshift(1:n, minIdx - 1)) = 1:n; % A permutation that'll place minIdx at index 1
    g_waypoints{i}.camPositions = camPositions(reindex,:);
    g_waypoints{i}.camNormals = camNormals(reindex,:);
    g_waypoints{i}.camPositionRepeats = camPositionRepeats(reindex,:);
end

% Inflate obstacles by safeDist
g_inflatedObstacles = {};
g_clippedPathIndices = {};
safeDist = 2.0; % Ensure drone never goes closer than 2 meters from an obstacle
scale_to_int = 2^15; % A large scale

for i=1:numel(g_altitudes_mesh)
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
end

% Check intersections
g_crashEdges = {};
for i=1:numel(g_altitudes_mesh)
    camPositions = g_waypoints{i}.camPositions;
    allEdges = [camPositions, circshift(camPositions, -1)];
    inflatedObstacles = g_inflatedObstacles{i};
    crashEdges = [];
    for j=1:size(inflatedObstacles, 2) % Test with each obstacle polygone in the set
        poly = inflatedObstacles{j};
        %[~, xinds] = intersectEdgePolygon(allEdges, poly);
        inter = arrayfun(@(idx) intersectEdgePolygon(allEdges(idx,:), poly), 1:size(allEdges,1), 'UniformOutput', false);
        crashEdges = [crashEdges, find(~cellfun(@isempty, inter))];
    end
    g_crashEdges{i} = sort(crashEdges);
end

% Grab part of safeBuffer lying in-between the edges, and compute the coarsest
% approximation of this sub-path that doesn't intersect with inflated obstacles
% And add to original set of waypoints
g_simplifiedIndices = {};
g_waypointsModified = {};
simplyfyThreshold = 0.5; % 0.5 m should be safe since the safePath is safeDist (2.0 m) away from obstacles.
for i=1:numel(g_altitudes_mesh)
    if isempty(g_crashEdges{i}) 
        g_simplifiedIndices{i} = {};
        g_waypointsModified{i} = g_waypoints{i};
        g_waypointsModified{i}.camPositionDummy = zeros(size(g_waypoints{i}.camPositions, 1), 1, 'logical');
        continue;
    end
    crashEdges = g_crashEdges{i};
    camPositions = g_waypoints{i}.camPositions;
    camPositionsModified = camPositions;
    safeBuffer = g_safeBuffer{i};
    simplifiedIndices = {};
    appendingFracIdx = (1:size(camPositions,1))';
    newCamPositions = camPositions;
    newCamNormals = g_waypoints{i}.camNormals;
    newCamPositionRepeats = g_waypoints{i}.camPositionRepeats;
    newCamPositionDummy = zeros(size(newCamPositions, 1), 1, 'logical'); %Is the waypoint dummy?
    for j=1:numel(crashEdges)
        nextID = 1 + mod(crashEdges(j) - 1 + 1, size(camPositions, 1));
        p0 = camPositions(crashEdges(j),:);
        p1 = camPositions(nextID,:);
        [~, p0Idx] = min(sqrt(sum((safeBuffer - p0).^2, 2)));
        [~, p1Idx] = min(sqrt(sum((safeBuffer - p1).^2, 2)));
        curve = safeBuffer(p0Idx:p1Idx, :);
        [~, simplifiedIdx] = dpsimplify(curve, simplyfyThreshold);
        simplifiedIndices{j} = simplifiedIdx - 1 + p0Idx;
        appendingFracIdx = [appendingFracIdx; crashEdges(j) + linspace(0.1, 0.9, numel(simplifiedIdx) - 2)'];
        newCamPositions = [newCamPositions; safeBuffer(simplifiedIndices{j}(2:end-1), :)];
        newCamNormals = [newCamNormals; zeros(numel(simplifiedIdx), 2)];
        newCamPositionRepeats = [newCamPositionRepeats; zeros(numel(simplifiedIdx), 1, 'logical')];
        newCamPositionDummy = [newCamPositionDummy; ones(numel(simplifiedIdx), 1, 'logical')];
    end
    g_simplifiedIndices{i} = simplifiedIndices;
    [~, appendingSortedIdx] = sort(appendingFracIdx);
    % Permute the new arrays to correct order.
    g_waypointsModified{i}.camPositions = newCamPositions(appendingSortedIdx, :);
    g_waypointsModified{i}.camNormals = newCamNormals(appendingSortedIdx, :);
    g_waypointsModified{i}.camPositionRepeats = newCamPositionRepeats(appendingSortedIdx);
    g_waypointsModified{i}.camPositionDummy = newCamPositionDummy(appendingSortedIdx);
end

%% Compute head rotations at waypoints (w.r.t. the previous heading vector)
% For repeated points, the rotation is w.r.t. to previous drone heading
% (which was a cam normal direction for previous point)
for i=1:numel(g_altitudes_mesh)
   camPositions = g_waypointsModified{i}.camPositions; 
   camNormals = g_waypointsModified{i}.camNormals; % This is absolute direction
   camDummy = g_waypointsModified{i}.camPositionDummy;
   % Convert normals into rotation from current heading of the drone
   camRepeats = g_waypointsModified{i}.camPositionRepeats;
   prevPos = circshift(camPositions, 1);
   prevPos(1,:) = dummyPosition; % This is the very first position (dummy) of the drone.
   posVec = camPositions - prevPos;
   distVec = sqrt(sum(posVec.^2, 2));
   posVec = posVec./distVec; % Normalize
   posVec(camRepeats, :) = camNormals([camRepeats(2:end); false], :); %If position is repeated, set the vector to previous normal vector
   headRotations = vectorAngle(posVec, camNormals)*180/pi;
   q = headRotations >=180;
   headRotations(q) = headRotations(q) - 360; % Negative angles if obtuse
   headRotations(camDummy) = 0; % This has to be zero by function
   g_waypointsModified{i}.headRotations = headRotations;
end

%% Convert camera directions to yaw angles (from True North)
[lat, lon] = utm2deg(dummyPosition(1) + UTM_offset(1), dummyPosition(2) + UTM_offset(2), '43 R');
dummyPositionWGS84 = [lat, lon];
for i=1:numel(g_altitudes_mesh)
    % Convert camPositions to lat, lon
    camPositions = g_waypointsModified{i}.camPositions;
    camRepeats = g_waypointsModified{i}.camPositionRepeats;
    sumRepeats = cumRepeats(camRepeats);
    latlonsegs = zeros(size(camPositions, 1), 4);
    latlonsegs(1,1:2) = dummyPositionWGS84;
    % Important: Replace '43 R' with you UTM zone.
    [lat, lon] = utm2deg(camPositions(:, 1) + UTM_offset(1), camPositions(:, 2) + UTM_offset(2), repmat('43 R', size(camPositions, 1), 1));
    latlonsegs(2:end,1:2) = [lat(1:end-1), lon(1:end-1)];% This needs to be corrected for repeats
    latlonsegs(:,3:4) = [lat, lon]; 
    g_waypointsModified{i}.camPositionsWGS84 = [lat, lon];
    
    % Convert head rotations (which is w.r.t. current heading) to yaw values (w.r.t. True North)
    headRotations = g_waypointsModified{i}.headRotations;
    headings = -headRotations; %Convert to clockwise +ve
    q = headings < 0;
    headings(q) = 360 + headings(q);% Convert from [-180, 180] to [0, 360]
    % Convert repeat angles into absolute angles w.r.t the path previous path heading
    headings0 = headings;
    q = find(sumRepeats > 0);
    for j=1:numel(q)
        headings(q(j)) = mod(sum(   headings0(   (q(j)-sumRepeats(q(j))):q(j)   ) ), 360);
        idx = q(j)-sumRepeats(q(j))-1;
        if idx > 0
            latlonsegs(q(j),1:2) = latlonsegs(idx, 3:4); % for a repeat point, store the correct first latlon
        else
           latlonsegs(q(j),1:2) = dummyPositionWGS84; % This happens when the second camPosition is duplicate of first
        end
    end
    
    azimuths = zeros(size(camPositions,1), 1);
    for j=1:size(camPositions,1)
        azimuths(j) = azimuth(latlonsegs(j, 1), latlonsegs(j, 2), latlonsegs(j,3), latlonsegs(j, 4));
    end
    yaw = mod(headings + azimuths, 360);
    q = yaw > 180;
    yaw(q)  = yaw(q) - 360; % Convert to [-180, 180] range
    yaw(g_waypointsModified{i}.camPositionDummy) = 0; % We don't need yaw values for dummy positions
    g_waypointsModified{i}.camYaw = yaw;
end
%% Save results to file
% Coordinates in lat-long, angles in degrees, and distances are in meters
% First line specifies height of the mission in meters from ground level
% Format: bool:isdummy, bool:isrepeat, float:lat, float:lon, float:heading
% if isdummy is true (1), heading is not specified
% if isrepeat is true (1), lat, lon are not specified
for i=1:numel(g_altitudes_mesh)
    filename = [outfile_waypoints 'floor' num2str(g_altitudes_floors(i)) '_' num2str(g_altitudes_drone(i)) 'm.txt'];
    fid = fopen(filename, 'w');
    fprintf(fid, '%f\n', g_altitudes_drone(i));
    camPositionsWGS84 = g_waypointsModified{i}.camPositionsWGS84;
    camYaw = g_waypointsModified{i}.camYaw;
    %headRotations = g_waypointsModified{i}.headRotations;
    camRepeats = g_waypointsModified{i}.camPositionRepeats;
    camDummy = g_waypointsModified{i}.camPositionDummy;
    % Write out the first dummy point (for known positioining of the drone.
    fprintf(fid, '%d %d %.15f %.15f\n', true, false, dummyPositionWGS84(1), dummyPositionWGS84(2));
    for j=1:size(camPositionsWGS84, 1)
        isdummy = camDummy(j);
        isrepeat = camRepeats(j);
        if isdummy
            fprintf(fid, '%d %d %.15f %.15f\n', isdummy, isrepeat, camPositionsWGS84(j,1), camPositionsWGS84(j,2));
        elseif isrepeat
            fprintf(fid, '%d %d %.3f\n', isdummy, isrepeat, camYaw(j));
        else
            fprintf(fid, '%d %d %.15f %.15f %.3f\n', isdummy, isrepeat, camPositionsWGS84(j,1), camPositionsWGS84(j,2), camYaw(j));
        end
    end
    fclose(fid);
end
%% Plot
% for i=1:numel(g_altitudes_mesh)
%     camPositions2 = g_waypointsModified{i}.camPositions;
%     camNormals = g_waypoints{i}.camNormals;
%     camPositions = g_waypoints{i}.camPositions;
%     boundary = [g_boundary{i}; g_boundary{i}(1,:)];
%     safeBuffer = g_safeBuffer{i};
%     figure; hold on; axis equal;
%     inflatedObstacles = g_inflatedObstacles{i};
%     for j=1:size(inflatedObstacles, 2)
%         loop  = inflatedObstacles{j};
%         fill(loop(:,1), loop(:,2), [0.9, 0.9, 0.9], 'EdgeColor', 'None');
%     end
%     obstacles = g_obstacles{i};
%     for j=1:size(obstacles, 2)
%         loop  = obstacles{j};
%         fill(loop(:,1), loop(:,2), [0.5, 0.5, 0.5], 'EdgeColor', 'None');
%     end
%     
%     plot(boundary(:,1), boundary(:,2), 'k-');
%     plot(safeBuffer(:,1), safeBuffer(:,2), 'c-');
%     plot(camPositions2(:,1), camPositions2(:,2), 'b-', 'LineWidth', 2);
%     %plot(camPositions(:,1), camPositions(:,2), 'r-');
%     %plot(camPositions(:,1), camPositions(:,2), 'b.');
% 
%     plotNormals(camPositions, camNormals, 'k');
%     plot(camPositions(1, 1), camPositions(1,2), 'ok');
%     text(camPositions2(:,1), camPositions2(:,2), num2str([1:size(camPositions2, 1)]'));
%     dline = [dummyPosition; camPositions(1,:)];
%     plot(dline(:,1), dline(:,2), 'r-');
%     
% %     for j=1:numel(g_crashEdges{i})
% %         line = camPositions(g_crashEdges{i}(j):g_crashEdges{i}(j) +1,:);
% %        plot(line(:,1), line(:,2), 'k', 'LineWidth', 3); 
% %     end
% %     for j=1:size(g_simplifiedIndices{i}, 2)
% %        simp = safeBuffer(g_simplifiedIndices{i}{j}, :);
% %        plot(simp(:,1), simp(:,2), 'k-')
% %     end
% end
%% Plot2
for i=1:numel(g_altitudes_mesh)
    camPositionsWGS84 = g_waypointsModified{i}.camPositionsWGS84;
    camYaw = g_waypointsModified{i}.camYaw;
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
    
    boundary = [g_boundary{i}; g_boundary{i}(1,:)];
    safeBuffer = g_safeBuffer{i};
    
    figure('units','normalized','outerposition',[0 0 1 1]); % A maximized figure window
    hold on; axis equal;
    inflatedObstacles = g_inflatedObstacles{i};
    for j=1:size(inflatedObstacles, 2)
        loop  = inflatedObstacles{j};
        fill(loop(:,1), loop(:,2), [0.9, 0.9, 0.9], 'EdgeColor', 'None');
    end
    obstacles = g_obstacles{i};
    for j=1:size(obstacles, 2)
        loop  = obstacles{j};
        fill(loop(:,1), loop(:,2), [0.5, 0.5, 0.5], 'EdgeColor', 'None');
    end
    
    plot(boundary(:,1), boundary(:,2), 'k-');
    plot(safeBuffer(:,1), safeBuffer(:,2), 'c-');
    plot(camX, camY, 'b-', 'LineWidth', 2);
    text(camX, camY, num2str([1:size(camX, 1)]'));
    
    plotNormals([camX, camY], nnorms, 'k');
    plotNormals([camX, camY], pnorms, 'r');
    filename = [outfile_waypoints 'floor' num2str(g_altitudes_floors(i)) '_' num2str(g_altitudes_drone(i)) 'm.png'];
    saveas(gcf, filename)
    close;
end
