function plotNormals(points, normals, style)
for i=1:size(points, 1)
   l = points(i,:) + [[0 0]; normals(i,:)*2];
   plot(l(:,1), l(:,2), style);
end