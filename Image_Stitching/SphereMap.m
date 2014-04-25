% I=imread('result4.jpg'); 
% [x,y,z] = sphere(9);
% warp(x,y,z,I);
% imshow('ashaks')


imgRGB = imread('result4.jpg');
[imgInd,map] = rgb2ind(imgRGB,256);
[imgIndRows,imgIndCols] = size(imgInd);
[X,Y,Z] = sphere(imgIndRows,imgIndCols);
surface(X,Y,Z,flipud(imgInd),...
    'FaceColor','texturemap',...
    'EdgeColor','none',...
    'CDataMapping','direct');
colormap(map);
view(-35,45);


image = imread('r_undistorted_perspective.jpg');
clf
%imshow(image);
r = @(x) sqrt(x(:,1).^2 + x(:,2).^2);
w = @(x) atan2(x(:,2), x(:,1));

f = @(x) [r(x).^2 .* cos(w(x)), r(x).^2 .* sin(w(x))];
g = @(x, unused) f(x);

tform3 = maketform('custom', 2, 2, [], g, []);
image3 = imtransform(face, tform3, 'UData', [-1 1], 'VData', [-1 1], ...
    'XData', [-1 1], 'YData', [-1 1]);
imshow(image3)
