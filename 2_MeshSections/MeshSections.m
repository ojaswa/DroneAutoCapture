% DroneAutoCapture: Main routine for MeshSections stage
% 
% Author: Ojaswa Sharma, <ojaswa@iiitd.ac.in>
%

%% Init
clc; clear; close all;

%GPToolbox
gp_subdirs = split(genpath('../Depends/gptoolbox/'),':');
addpath(strjoin(gp_subdirs(~contains(gp_subdirs,'.git')),':'));
addpath('../Include');

building_name = 'Academic';
coverage_type = 'walls';


vertHalfFOV = 32.758089*pi/180;
bufferDist = 8; % 8 meter buffer around floor plan boundary
vertCameraHalfCoverage = bufferDist*tan(vertHalfFOV);

[g_altitudes_drone, g_altitudes_floors, groundMeshZ, buildingHeight] = getBuildingParameters(building_name, coverage_type);
g_altitudes_mesh = groundMeshZ + g_altitudes_drone; % Altitudes at which to section the mesh

infile_mesh = ['../Data/Obstacle data/', building_name, '/mesh_20m.obj'];
outfile_prefix = ['../Data/Obstacle data/', building_name, '/obstacle_'];

%% Read mesh
[V, ~, ~, F, ~, ~] = readOBJFast(infile_mesh);
g_mesh.V = V(:, 1:3);
g_mesh.F = F;
clear V F;

%% plot mesh with cutting planes
figure; axis equal; hold on;
patch('Faces', g_mesh.F, 'Vertices', g_mesh.V, 'FaceColor', 'r', 'EdgeColor', 'None');
mesh_center = mean(g_mesh.V(:,1:2));
for i=1:1 %numel(g_altitudes_mesh)
   pl = createPlane( [mesh_center g_altitudes_mesh(i)], [0 0 1]);
   drawPlane3d(pl);
end
view(3); camlight
%% Create cross sections
meshBBox = boundingBox3d(g_mesh.V);
g_sections = {};

verticalBuffer = 1; %1 meter above and below any altitude
horizontalBuffer = 0.5; % Expand obstacle poly by 0.5 m

for i=1:numel(g_altitudes_mesh)
    g_sections{i} = generateObstaclePoly(g_mesh, g_altitudes_mesh(i), verticalBuffer, horizontalBuffer);
end

%% Generate offset buffer from floor plan --only for visualization
UTM_offset = [722338.000 3159808.000 398.000];
for i = 1: numel(g_altitudes_mesh)
    infile_boundary = ['../Data/Plan boundaries/', building_name, '/geoBF', num2str(g_altitudes_floors(i)), '.dxf'];
    
    % Load boundary of largest cross section
    [c_Line,c_Poly,c_Cir,c_Arc,c_Poi] = f_LectDxf(infile_boundary);
    g_boundary = c_Poly{1} - UTM_offset(1,1:2);
    
    limitDist = 20;
    scale_to_int = 2^15; % A large scale
    in_pts.x = int64(scale_to_int*g_boundary(:,1));
    in_pts.y = int64(scale_to_int*g_boundary(:,2));
    orientation = clipper(in_pts);
    if orientation == 0 % Orientation is clockwise, reverse it.
        g_boundary = flipud(g_boundary);
        in_pts.x = flipud(in_pts.x);
        in_pts.y = flipud(in_pts.y);
    end
    out_pts = clipper(in_pts, bufferDist*scale_to_int, 0);
    g_buffer = double([out_pts(1).x out_pts(1).y])/scale_to_int;
    out_pts = clipper(in_pts, limitDist*scale_to_int, 0);
    g_limit = double([out_pts(1).x out_pts(1).y])/scale_to_int;
    
    % plot
    figure; hold on; axis equal;
    section = g_sections{i};
    for j=1:size(section, 2)
        loop = section{j};
        loop = [loop; loop(1,:)];
        fill(loop(:,1), loop(:,2), 'r-', 'EdgeColor', 'None');
    end
    plot(g_buffer(:,1), g_buffer(:,2), 'b-')
    plot(g_boundary(:,1), g_boundary(:,2), 'k-')
    plot(g_limit(:,1), g_limit(:,2), 'b--')
    title(['Floor ' num2str(g_altitudes_floors(i))]);
end

%% Write out the obstacle polygons
fprintf('writing obstacle files for %s: ', coverage_type);
for k=1:length(g_altitudes_mesh)
    fprintf('%d ',g_altitudes_floors(k));
    filename = [outfile_prefix coverage_type num2str(g_altitudes_floors(k)) '_' num2str(g_altitudes_drone(k)) 'm.txt'];
    fid = fopen(filename, 'w');
    % Write two lines per loop (one of x coordinate, another for y
    section = g_sections{k};
    for i=1:size(section, 2)
        loop = section{i};
        fprintf(fid, '%f', loop(1,1));
        fprintf(fid, ',%f', loop(2:end,1));
        fprintf(fid, '\n');
        fprintf(fid, '%f', loop(1,2));
        fprintf(fid, ',%f', loop(2:end,2));
        fprintf(fid, '\n');
    end
    fclose(fid);
end
fprintf('done.\n');
