function points = fill_in_using_spline(points)
% points = fill_in_using_spline(points)
%
% Use a spline to fill in the missing points (those with value <=0),
% as returned by label_sequence.m
%

T = size(points,2);
bad = find(points(1,:) <= 0 | points(2,:) <= 0);
good = setdiff(1:T, bad);

% spline does interpolation
points = spline(good,points(:,good),1:T);

% csaps smooths the result a little, without strictly interpolating
% the non-zero points
% [curve,p] = csaps(good, points(:,good), 0.2);
% points(:,bad) = ppval(curve,bad);
