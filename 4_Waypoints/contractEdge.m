function edgeOut = contractEdge(edgeIn, eps)
% Contract edge by distance eps from both sides and return the modified
% edge
vec = edgeIn(3:4) - edgeIn(1:2);
dist = sqrt(dot(vec, vec));
vec = vec/dist;
edgeOut = [ edgeIn(3:4) - (dist - eps)* vec, edgeIn(1:2) + (dist - eps)*vec];