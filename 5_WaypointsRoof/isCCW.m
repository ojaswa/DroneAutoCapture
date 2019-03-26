function res = isCCW(poly)
% Returns true if given polygon orientation is Counter-clockwise, else
% false
res = false;
if size(poly, 1) < 3
    error('Given polygon is invalid.');
end
sum = 0;
for i=1:(size(poly, 1)-1)
    p0 = poly(i,:);
    p1 = poly(i+1,:);
    sum  = sum + (p1(1) - p0(1))*(p1(2) + p0(2));    
end
if sum < 0
    res = true;
end