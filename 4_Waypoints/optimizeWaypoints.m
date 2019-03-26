function [camPositions, camNormals, camPositionRepeats] = optimizeWaypoints(boundary, buffer, dWall, cameraHalfAngle)
% Assumes last point is connected to the first. Therefore the last point
% in array should NOT be repeated at the end.
% camPositions - camera positions
% camNormals - camera normals
% camPositionRepeats - a vector with true at indices where position is repeated  (same as previous)
MAX_DJI_WAYPOINTS = 99; % Hardware limit

%% Subdivide boundary into smaller patches of similar sizes
patchSize = 1; % 1 meter
patchCenters = [];
patchWidths = [];
patchNormals = [];
%patchPolygon = [];
%patchPolygonNormals = [];
boundary_loop = [boundary; boundary(1,:)]; % close it
for i=1:(size(boundary_loop, 1) - 1)
    p0 = boundary_loop(i,:);
    p1 = boundary_loop(i+1,:);
    vec = p1 - p0;
    len = sqrt(dot(vec, vec));
    vec  = vec / len; % normalize;
    perp = [-vec(2), vec(1)];
    tvec = [0:patchSize:len];
    if mod(len, patchSize) > 0
       tvec = [tvec, len]; 
    end
    pcenters = p0 + (tvec(1:end-1)' + tvec(2:end)')*vec/2;
    pwidths = tvec(2:end)' - tvec(1:end-1)';
    pnormals = repmat(perp, numel(pwidths), 1);
    %ppoly = p0 + tvec(1:end-1)'*vec;
    %ppolynorms = repmat(perp, numel(tvec) - 1, 1);
    patchCenters = [patchCenters; pcenters];
    patchWidths = [patchWidths; pwidths];
    patchNormals = [patchNormals; pnormals];
    %patchPolygon = [patchPolygon; ppoly];
    %patchPolygonNormals = [patchPolygonNormals; ppolynorms];
end

%% Subdivide buffer into a dense pointset (of candidate camera positions)
pointsetDensity = 1; % Desired point density, 1m or less
vec_buffer = circshift(buffer, -1) - buffer;
dist_buffer = sqrt(dot(vec_buffer, vec_buffer, 2));
maxDensity = min(0.05, mode(dist_buffer, 1)); % Most frequent value or 0.05 m, whichever is minimum
vec_buffer = vec_buffer./dist_buffer;
npts = size(buffer, 1);
% Compute points
pointsetPositionsMax = [];
for i=1:npts
   if dist_buffer(i) <= maxDensity 
       pointsetPositionsMax = [pointsetPositionsMax; buffer(i,:)];
       continue; 
   end
   dist = dist_buffer(i);
   tvec = 0:maxDensity:dist;
   if mod(len, patchSize) == 0 % if last point happens to be in tvec (rare!), remove it.
       tvec(end) = [];
   end
   pos = buffer(i,:) + tvec'*vec_buffer(i,:);
   pointsetPositionsMax = [pointsetPositionsMax; pos];
end
% Compute normals -- this is more accurate than gradient()
vec_buffer = circshift(pointsetPositionsMax, -1) - pointsetPositionsMax; % A segment vector in buffer
dist_buffer = sqrt(dot(vec_buffer, vec_buffer, 2));
vec_buffer = vec_buffer./dist_buffer; % Normalize
nrm_buffer = [-vec_buffer(:,2), vec_buffer(:,1)]; % Orthogonal vector
nrm_buffer_avg = circshift(nrm_buffer, -1) + nrm_buffer; % Averaged normal (this is like a centered derivative approach)
pointsetNormalsMax = nrm_buffer_avg./sqrt(dot(nrm_buffer_avg, nrm_buffer_avg, 2));% Normalize and store

% Resample the pointset at given density
totalLength = sum(dist_buffer);

% Remove duplicates from dense sampling
[cumdist_buffer, idx] = unique(cumsum(dist_buffer));
clear dist_buffer vec_buffer nrm_buffer nrm_buffer_avg;
totalLength = cumdist_buffer(end);
pointsetPositionsMax = pointsetPositionsMax(idx,:);
pointsetNormalsMax = pointsetNormalsMax(idx,:);

tvec = 0:pointsetDensity:totalLength; % Desired point density
ind = interp1(cumdist_buffer, 1:length(cumdist_buffer), tvec, 'nearest', 'extrap');
pointsetPositions = pointsetPositionsMax(ind, :);
pointsetNormals = pointsetNormalsMax(ind, :);

%% Build visibility list for all patches from all points in pointsetPositions
npatches = size(patchCenters, 1);
npointset = size(pointsetPositions, 1);
patchVisibilityMatrix = -Inf*ones(npatches, npointset);
for i=1:npatches
   pcenter = patchCenters(i,:);
   pwidth = patchWidths(i,:);
   pnormal = patchNormals(i,:);
   pvec = [pnormal(2), -pnormal(1)];
   p0 = pcenter - pvec*pwidth/2;
   p1 = pcenter + pvec*pwidth/2;
   % Consider only front facing line-of-sights
   frontFacingIdx0 = find(dot(p0 - pointsetPositions, repmat(pnormal, npointset, 1), 2) > 0);
   frontFacingIdx1 = find(dot(p1 - pointsetPositions, repmat(pnormal, npointset, 1), 2) > 0);
   frontFacingIdx = intersect(frontFacingIdx0, frontFacingIdx1);
   for j=1:size(frontFacingIdx, 1)
       ps = pointsetPositions(frontFacingIdx(j), :); % pointset position
       edge0 = contractEdge([p0 ps], 1e-8);
       edge1 = contractEdge([p1 ps], 1e-8);
       xbuffer0 = intersectEdgePolygon(edge0, buffer);
       if isempty(xbuffer0)
           xboundary0 = intersectEdgePolygon(edge0, boundary); % Self intersection
           if isempty(xboundary0)
               xbuffer1 = intersectEdgePolygon(edge1, buffer);
               if isempty(xbuffer1)
                   xboundary1 = intersectEdgePolygon(edge1, boundary); % Self intersection
                   if isempty(xboundary1)
                       % Entire patch is clearly visible from point ps
                       los_vec = pcenter - ps; % line-of-sight vector
                       los_dist = sqrt(dot(los_vec, los_vec));
                       los_vec = los_vec./los_dist;
                       patchVisibilityMatrix(i, frontFacingIdx(j)) = dot(los_vec, pnormal);
                   end
               end
           end
       end
   end
end

%% Use Genetic algorithm to find optimum
% Parameters of our optimization
boundaryLength = sum(patchWidths);
cameraBase = 2*dWall*tan(cameraHalfAngle);
ncams = min(3*ceil(boundaryLength/cameraBase), MAX_DJI_WAYPOINTS);
cameraMaxDeviation = pi/16; % Max deviation of camera normal from buffer normal
ps_vec = circshift(pointsetPositions, -1) - pointsetPositions;
pointsetDist = sqrt(dot(ps_vec, ps_vec, 2));
pointsetCumDist = cumsum(pointsetDist);
bufferLength = sum(pointsetDist);
nCamerasViewingPatch = 3; % we want each patch to be viewed by at least 'n' cameras

% Bounds
lb = zeros(2*ncams, 1);
ub = ones(2*ncams, 1);
% No other constrains
A = [];
b = [];
Aeq = [];
beq = [];
nonlcon = [];
% Set options
data.boundary = boundary;
data.buffer = buffer;
data.pointsetCumDist = pointsetCumDist;
data.pointsetPositions = pointsetPositions;
data.pointsetNormals = pointsetNormals;
data.patchCenters = patchCenters;
data.patchWidths = patchWidths;
data.patchNormals = patchNormals;
data.patchVisibilityMatrix = patchVisibilityMatrix;
data.cameraHalfAngle = cameraHalfAngle;
data.bufferLength = bufferLength;
data.cameraMaxDeviation = cameraMaxDeviation;
data.nCamerasViewingPatch = nCamerasViewingPatch;

GA_PlotFn = @(options,state,flag) GA_plot(options,state,flag, data);
GA_ObjFn = @(x)GA_objective(x, data);

options = optimoptions(@ga, 'PlotFcn', GA_PlotFn,...
    'MaxGenerations',500,'PopulationSize',200, ...
    'MaxStallGenerations',200,'UseVectorized',false);
% Perform optimization
tic
nvars = ncams*2;
[x, fval] = ga(GA_ObjFn, nvars, A, b, Aeq, beq, lb, ub, nonlcon, options);
toc
% Get camera position and normals
tvec = x(1:ncams)'*bufferLength; % This is not ordered, so needs to be sorted!
avec = (x(ncams+1:end)'*2 - 1)*cameraMaxDeviation; % angle from buffer normal, ccw positive.
[tvec, idx] = sort(tvec);
avec = avec(idx);

cam_ind = interp1(pointsetCumDist, 1:length(pointsetCumDist), tvec, 'nearest', 'extrap');
[~, ~, repeats] = unique(cam_ind);
camPositions = pointsetPositions(cam_ind, :);
ps_norm = pointsetNormals(cam_ind,:);

camNormals = zeros(ncams, 2);
for i=1:ncams
   camNormals(i,:) = rotateVector(ps_norm(i,:), avec(i)); 
end
camPositionRepeats = ~logical([1; diff(repeats)]);
