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
