function state = GA_plot(options, state, flag, plotData)
%x, optimValues, state, plotData)
boundary = plotData.boundary;
buffer = plotData.buffer;
pointsetCumDist = plotData.pointsetCumDist;
pointsetPositions = plotData.pointsetPositions;
pointsetNormals = plotData.pointsetNormals;
bufferLength = plotData.bufferLength;
cameraMaxDeviation = plotData.cameraMaxDeviation;
patchCenters = plotData.patchCenters;
patchWidths = plotData.patchWidths;
patchNormals = plotData.patchNormals;

plot([boundary(:,1); boundary(1,1)], [boundary(:,2); boundary(1,2)], 'b-');
hold on; axis equal; axis tight;
plot(buffer(:,1), buffer(:,2), 'c-');

[uncovered, best] = min(state.Score);
x = state.Population(best,:)';

ncams = numel(x)/2;
tvec = x(1:ncams)*bufferLength;
avec = (x(ncams+1:end)*2 - 1)*cameraMaxDeviation;
ind = interp1(pointsetCumDist, 1:length(pointsetCumDist), tvec, 'nearest', 'extrap');
camPos = pointsetPositions(ind, :);
camNorm = zeros(ncams, 2);
for i=1:ncams
   camNorm(i,:) = rotateVector(pointsetNormals(ind(i), :), avec(i));
end
plot(camPos(:,1), camPos(:,2), 'ko');
plotNormals(camPos, camNorm, 'm-');

% Plot uncovered patches
uncoveredIDs = GA_uncovered(x, plotData);
for i=1:numel(uncoveredIDs)
    uid = uncoveredIDs(i);
   cen = patchCenters(uid,:);
   vec = patchNormals(uid,:)*[0 1; -1 0];
   wid = patchWidths(uid);
   l = cen + [-vec; vec]*wid/2;
   plot(l(:,1), l(:,2), 'r-', 'LineWidth', 2);
end
title(['Uncovered length: ' num2str(uncovered), ', Iter: ' num2str(state.Generation)]);
hold off