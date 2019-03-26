function cutBox = getCutBox(bbox, height, buffer)
p0 = bbox([1, 3, 5]);
p1 = bbox([2, 4, 6]);

% Expand bbox by some fraction on all sides
expandFraction = 0.1;
expandDist = expandFraction*(p1 - p0);
p0 = p0 - expandDist;
p1 = p1 + expandDist;

% Replace zmax with height
p1(3) = height + buffer;
p0(3) = height - buffer;

% Create box
V = [p0(1), p0(2) p0(3);% p0
     p0(1), p1(2) p0(3);
     p1(1), p1(2) p0(3);
     p1(1), p0(2) p0(3);
     p0(1), p0(2) p1(3);
     p0(1), p1(2) p1(3);
     p1(1), p1(2) p1(3);% p1
     p1(1), p0(2) p1(3)];
F = [1 3 2; 1 4 3;
    5 6 7; 5 7 8;
    2 3 6; 3 7 6;
    3 4 7; 4 8 7;
    4 5 8; 1 5 4;
    1 2 6; 1 6 5];
cutBox.V = V;
cutBox.F = F;