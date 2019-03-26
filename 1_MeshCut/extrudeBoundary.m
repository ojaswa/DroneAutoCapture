function extrudedMesh = extrudeBoundary(boundary, minz, maxz)
P = boundary(1:(end-1),:);
E = [1:size(P,1);2:size(P,1) 1]';
[V, F] = wedding_cake({P}, {E}, [maxz - minz]);
V(:,3) = V(:,3) + minz;
extrudedMesh.V = V;
extrudedMesh.F = F;
%patch('Faces', F, 'Vertices', V, 'FaceVertexCData', V(:,3), 'FaceColor', 'interp'); axis equal;