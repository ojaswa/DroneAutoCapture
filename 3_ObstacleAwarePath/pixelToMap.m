function pathmap = pixelToMap(minpix, maxpix, minmap, maxmap, pathpix)
% Map pathpix to pathMap via a bilinear function
pathmap = minmap + (pathpix  - minpix).*(maxmap - minmap)./(maxpix - minpix);