function J = GA_objective(x, optimData)
pointsetCumDist = optimData.pointsetCumDist;
pointsetPositions = optimData.pointsetPositions;
pointsetNormals = optimData.pointsetNormals;
patchCenters = optimData.patchCenters;
patchWidths = optimData.patchWidths;
patchVisibilityMatrix = optimData.patchVisibilityMatrix;
cameraHalfAngle = optimData.cameraHalfAngle;
bufferLength = optimData.bufferLength;
cameraMaxDeviation = optimData.cameraMaxDeviation;
nCamerasViewingPatch = optimData.nCamerasViewingPatch;

% Get actual positions and normals from parameters
ncams = size(x,2)/2;
tvec = x(1:ncams)'*bufferLength; % Camera positions along the obstacle free path
avec = (x(ncams+1:end)'*2 - 1)*cameraMaxDeviation; % Camera deviation from normal to drone path
cam_ind = interp1(pointsetCumDist, 1:length(pointsetCumDist), tvec, 'nearest', 'extrap');
cam_pos = pointsetPositions(cam_ind, :);
cam_norm = pointsetNormals(cam_ind,:);
cam_angle_range = [avec - cameraHalfAngle, avec + cameraHalfAngle];

% For each camera and given normal, calculate patch indices covered
mergedList = [];
for i=1:ncams
    patch_ind = find(~isinf(patchVisibilityMatrix(:, cam_ind(i)))); % Visible patches from ith camera
    p0 = cam_pos(i,:);
    n0 = cam_norm(i,:);
    c2p_vec = patchCenters(patch_ind,:) - p0;
    c2p_dist = sqrt(dot(c2p_vec, c2p_vec, 2));
    c2p_nrm = c2p_vec./c2p_dist;
    c2p_angle = atan2d(n0(1)*c2p_nrm(:,2)-n0(2)*c2p_nrm(:,1),n0(1)*c2p_nrm(:,1)+n0(2)*c2p_nrm(:,2))*pi/180;
    c2p_covered = and(c2p_angle >= cam_angle_range(i,1), c2p_angle <= cam_angle_range(i,2));
    mergedList = [mergedList, patch_ind(c2p_covered)'];
end
[patchesFreq, patchesCovered] = hist(mergedList, unique(mergedList));

% Add up lengths of uncovered patches
npatches = size(patchCenters, 1);
allPatchDeficit = zeros(npatches, 1); % Coverage deficit for each patch
allPatchDeficit(patchesCovered) = patchesFreq;
allPatchDeficit(allPatchDeficit >= nCamerasViewingPatch) = nCamerasViewingPatch; % saturate to upper limit
allPatchDeficit = nCamerasViewingPatch - allPatchDeficit; % Convert to deficit (of # of views for every patch)
J = sum(patchWidths.*allPatchDeficit);