function [path2, xidx] = mergeIntersectionsInPath(path, xpoints)
% Merge intersection points xpoints (m x 2) in path (n x 2)
% Return modified path2 ((n + m) x 2) and indices to merged xpoints in path2
edges = [path, circshift(path, -1)];
dist = distancePointEdge(xpoints, edges);
[~, minidx] = min(dist, [], 2);
minidx2 = 1 + mod(minidx , size(path,1)); % Index to the other point on the segment
mindist = sqrt(sum((xpoints - path(minidx, :)).^2, 2))./sqrt(sum((path(minidx, :) - path(minidx2, :)).^2,2)); % fraction of dist
fracidx = minidx + mindist;

% Append fracidx to the range of indices, append xpoints to path and sort
% all based on indices.
path2 = [path; xpoints];
idx2 = [transpose(1:size(path,1));fracidx];

[~, idxorder] = sort(idx2);
path2 = path2(idxorder,:);
revorder(idxorder) = 1:size(path2,1);
xidx = transpose(revorder( (size(path,1)+1):end));

