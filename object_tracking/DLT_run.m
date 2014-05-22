
clear all
dataPath = 'D:\data_seq\';
% dataPath = 'F:\dropbox\Tracking\data\';
%title = 'Car4';
title = 'Debris';

switch (title)
case 'davidin';  p = [158 106 62 78 0];
    opt = struct('numsample',1000, 'affsig',[4, 4,.005,.00,.001,.00]);
case 'trellis';  p = [200 100 45 49 0];
    opt = struct('numsample',1000, 'affsig',[4,4,.00, 0.00, 0.00, 0.0]);
case 'Car4';  p = [123 94 107 87 0];
    opt = struct('numsample',1000, 'affsig',[4,4,.02,.0,.001,.00]);
case 'car11';  p = [88 139 30 25 0];
    opt = struct('numsample',1000,'affsig',[4,4,.005,.0,.001,.00]);
case 'animal'; p = [350 40 100 70 0];
    opt = struct('numsample',1000,'affsig',[12, 12,.005, .0, .001, 0.00]);
case 'shaking';  p = [250 170 60 70 0];%
    opt = struct('numsample',1000, 'affsig',[4,4,.005,.00,.001,.00]);
case 'singer1';  p = [100 200 100 300 0];
    opt = struct('numsample',1000, 'affsig',[4,4,.01,.00,.001,.0000]);
case 'bolt';  p = [292 107 25 60 0];
    opt = struct('numsample',1000, 'affsig',[4,4,.005,.000,.001,.000]);
case 'woman';  p = [222 165 35 95 0.0];
    opt = struct('numsample',1000, 'affsig',[4,4,.005,.000,.001,.000]);               
case 'bird2';  p = [116 254 68 72 0.0];
    opt = struct('numsample',1000, 'affsig',[4,4,.005,.000,.001,.000]); 
case 'surfer';  p = [286 152 32 35 0.0];
    opt = struct('numsample',1000,'affsig',[8,8,.01,.000,.001,.000]);   
case 'Debris';  p = [280 340 110 120 0.0];
    opt = struct('numsample',1000,'affsig',[8,8,.01,.000,.001,.000]);     
otherwise;  error(['unknown title ' title]);
end

% The number of previous frames used as positive samples.
opt.maxbasis = 10;
opt.updateThres = 0.8;
% Indicate whether to use GPU in computation.
global useGpu;
useGpu = true;
opt.condenssig = 0.01;
opt.tmplsize = [32, 32];
opt.normalWidth = 320;
opt.normalHeight = 240;
seq.init_rect = [p(1) - p(3) / 2, p(2) - p(4) / 2, p(3), p(4), p(5)];

% Load data
disp('Loading data...');
fullPath = [dataPath, title, '\img\'];
d = dir([fullPath, '*.jpg']);
if size(d, 1) == 0
    d = dir([fullPath, '*.png']);
end
if size(d, 1) == 0
    d = dir([fullPath, '*.bmp']);
end
im = imread([fullPath, d(1).name]);
data = zeros(size(im, 1), size(im, 2), size(d, 1));
seq.s_frames = cell(size(d, 1), 1);
for i = 1 : size(d, 1)
    seq.s_frames{i} = [fullPath, d(i).name];
end
seq.opt = opt;
results = run_DLT(seq, '', false);
save([title '_res'], 'results');
