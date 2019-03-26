% DroneAutoCapture: Main routine for MeshCut stage
% 
% Author: Ojaswa Sharma, <ojaswa@iiitd.ac.in>
%

%% Init
clc; clear; close all;
gp_subdirs = split(genpath('../Depends/gptoolbox/'),':');
addpath(strjoin(gp_subdirs(~contains(gp_subdirs,'.git')),':'));
addpath('../Include');

% Parameters:
building_name = 'Academic';
coverage_type = 'walls';

infile_boundary = ['../Data/Plan boundaries/', building_name, '/geoBF1.dxf'];
infile_nadir_mesh = ['../Data/Campus mesh/nadir_mesh.obj'];
outfile_mesh_prefix = ['../Data/Obstacle data/', building_name, '/mesh_'];

%% Read boundary  and create offset curve
UTM_offset = [722338.000 3159808.000 398.000]; % Use appropriate UTM offset for the  nadir mesh
% Load boundary of largest cross section
[c_Line,c_Poly,c_Cir,c_Arc,c_Poi] = f_LectDxf(infile_boundary);
g_boundary = c_Poly{1} - UTM_offset(1,1:2);

%% Create buffer polygon
bufferDist = 20; % 20 meter buffer
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

%% Load campus mesh --WARNING: nadir mesh could be very large to load!
[V, TC, N, F, FTC, FN] = readOBJFast(infile_nadir_mesh);
g_campus_mesh.V = V(:, 1:3);
g_campus_mesh.TC = TC;
g_campus_mesh.N = N;
g_campus_mesh.F = F;
g_campus_mesh.FTC = FTC;
g_campus_mesh.FN = FN;
clear V TC N F FTC FN;
campusBBox = boundingBox3d(g_campus_mesh.V);

%% Extrude buffer polygon to a mesh
extraFraction = 0.05; % expand the z-extent by 5% on each side
extraDist = (campusBBox(6) - campusBBox(5))*extraFraction;
minz = campusBBox(5) - extraDist;
maxz = campusBBox(6) + extraDist;
g_buffer_mesh = extrudeBoundary(g_buffer, minz, maxz);

%% Cut campus mesh with buffer mesh
tic
%[V, F, J] = mesh_boolean_winding_number(g_campus_mesh.V, g_campus_mesh.F, g_buffer_mesh.V, g_buffer_mesh.F, 'intersect'); % Not good!
[V, F, J] = mesh_boolean(g_campus_mesh.V, g_campus_mesh.F, g_buffer_mesh.V, g_buffer_mesh.F, 'intersect', 'BooleanLib', 'cork');
toc
%% Save cutout mesh
writeOBJ([outfile_mesh_prefix num2str(bufferDist) 'm.obj'], V, F); % Not writing texture coords!

%% Plot mesh
figure; hold on; axis equal;
patch('Vertices', V, 'Faces', F, 'FaceColor', 'r', 'EdgeColor', 'none');
camlight
