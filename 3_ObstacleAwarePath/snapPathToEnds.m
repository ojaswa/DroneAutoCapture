function snappedPath = snapPathToEnds(endPts, path)
% Snap path to endPoints given by endIdx which are indices to two points in closed contour

%Trim a few points in path from both sides -- sometimes they are very spiky at ends
path([1,2,end-1,end],:) = [];

INFLEUNCE_RADIUS = 20; % points in path

% Move points to join main contour
snappedPath = path;
% First part
vecP = endPts(1,:) - path(1, :);
distP = sqrt(sum(vecP.*vecP));
vecP = vecP/distP;
dists = distP*(1 + cos(pi*linspace(0, 1, INFLEUNCE_RADIUS)))/2; % Sinusoidal decay
snappedPath(1:INFLEUNCE_RADIUS, :) = snappedPath(1:INFLEUNCE_RADIUS, :) + dists'*vecP;
% Last part
vecP = endPts(2,:) - path(end, :);
distP = sqrt(sum(vecP.*vecP));
vecP = vecP/distP;
dists = fliplr(distP*(1 + cos(pi*linspace(0, 1, INFLEUNCE_RADIUS)))/2); % Sinusoidal decay
snappedPath((end-INFLEUNCE_RADIUS+1):end, :) = snappedPath((end-INFLEUNCE_RADIUS+1):end, :) + dists'*vecP;
