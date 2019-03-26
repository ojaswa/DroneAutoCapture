function obstaclePoly = generateObstaclePoly(mesh, altitude, verticalBuffer, varargin)
% generateObstaclePoly generates a cross section of a mesh at a certain
% altitude. The section is a union of two cross sections above and below
% the altitude by  the verticalBuffer amount. Optionally offset the cross
% section by horizontal distance
% Inputs: 
% Mesh: mesh with struct fields V and F
% altitude: height in meters
% verticalBuffer: distance in meters
% Optional: 
%   horizontalBuffer: distance in meters
% Outputs:
% opoly struct array of contours

do_horizontal_offset = false;
horizontalBuffer = 0;
if ~isempty(varargin)
    do_horizontal_offset = true;
    horizontalBuffer = varargin{1};
end
tic
fprintf('generateObstaclePoly: sectioning... ');
%% Cut the mesh with a cuboid 
% The cuboid encloses mesh from all sides except the one that is kept at the altitude.
meshBBox = boundingBox3d(mesh.V);
% section
cutBox = getCutBox(meshBBox, altitude, verticalBuffer);
[V, F, ~] = mesh_boolean(mesh.V, mesh.F, cutBox.V, cutBox.F, 'intersect', 'BooleanLib', 'cork');

% Filter triangles w.r.t. Z-coordinate
confidence = 2.5e-5;
z_lo  = altitude - verticalBuffer;
z_hi = altitude + verticalBuffer;
faces_lo = abs(sum([V(F(:,1),3), V(F(:,2),3), V(F(:,3),3)] - z_lo, 2)) <= confidence;
faces_hi = abs(sum([V(F(:,1),3), V(F(:,2),3), V(F(:,3),3)] - z_hi, 2)) <= confidence;
F_lo = F(faces_lo, :);
F_hi = F(faces_hi,:);

[SF, ~, SV] = split_nonmanifold(F_lo, 'V', V);% The cross section turns out to be non-manifold, fix that.
lsec.V = SV;
lsec.F = SF;
[SF, ~, SV] = split_nonmanifold(F_hi, 'V', V);% The cross section turns out to be non-manifold, fix that.
usec.V = SV;
usec.F = SF;

%% Get boundaries from triangulated cross sections
fprintf('boundaries... ');
[uB, uL] = ordered_outline(usec.F);% Get ordered boundary
[lB, lL] = ordered_outline(lsec.F);% Get ordered boundary

%% Perform union --use Clipper, gpToolbox doesn't work well here
fprintf('union... ');
scale=2^15;% scale factor from double to int64 (arbitrary)
upoly = [];
lpoly = [];
for i=1:size(uL,2)-1
    loop = uB(uL(i):uL(i+1)-1);
    verts = usec.V(loop, 1:2);
    upoly(i).x = int64(scale*verts(:,1));
    upoly(i).y = int64(scale*verts(:,2));
end

for i=1:size(lL,2)-1
    loop = lB(lL(i):lL(i+1)-1);
    verts = lsec.V(loop, 1:2);
    lpoly(i).x = int64(scale*verts(:,1));
    lpoly(i).y = int64(scale*verts(:,2));
end

opoly = clipper(upoly, lpoly, 3); % Perform a union

% Optionally perform offsetting
if do_horizontal_offset
    fprintf('offsetting... ')
    for i=1:length(opoly)    % convert double output back to int64 for more input
        opoly(i).x=int64(opoly(i).x);
        opoly(i).y=int64(opoly(i).y);
    end
    opoly = clipper(opoly, horizontalBuffer*scale, 0);
end

%% Send output back
obstaclePoly = {};
for i=1:length(opoly)
   loop = [opoly(i).x/scale, opoly(i).y/scale];
   obstaclePoly{i} = loop;
end

timeElapsed = toc;
fprintf('done in %f sec.\n', timeElapsed);
