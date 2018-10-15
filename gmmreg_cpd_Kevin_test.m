%gmmreg_cpd_Kevin_test
%%%loading worm data and test with cpd-gmm registration method (kernel method)~~
%% loading data
cd /tigress/LEIFER/communalCode
%path(pathdef)

load('/tigress/LEIFER/PanNeuronal/20180523/BrainScanner20180523_150239/pointStatsRef.mat')
PS_ref = PS_ref;
load('/tigress/LEIFER/PanNeuronal/20180523/BrainScanner20180523_150239/PointsStats2.mat')
P1 = pointStats2(1);

model = PS_ref(5).straightPoints;  %model from reference
scene = P1.straightPoints;  %scene from one frame time

cd /home/kschen/github/gmmreg_cpd

%% configuration for gmm-reg
%%%original settings
config = gmmreg_load_config('./fish_full.ini');
config.motion = 'tps';%'grbf'%;  %%%kernel method for non-rigid !!!
% config.init_param = zeros(25,3);

MSEs = [];

for ii = 1:30  %%%repitition test
    
%%%load data
load('/tigress/LEIFER/PanNeuronal/20180523/BrainScanner20180523_150239/pointStatsRef.mat')
PS_ref = PS_ref;
model = PS_ref(ii).straightPoints;  %model from reference

sig = 3;
config.beta = 1;
config.lambda = 1;
config.model = model;
config.scene = model + sig*randn(size(model));  %scene(:,:);%

%%%compute initual sigma square
temp = 0;
for i = 1:length(config.model)
%     for j = 1:length(config.scene)
%         temp = temp + norm(config.model(i,:) - config.scene(j,:));
    temp = temp + norm(config.model(i,:) - config.scene(i,:));
%     end
end
config.init_sigma = (temp/i);%sig;%(temp/(i*j*size(config.model,3)))^0.5;

%%%make own control points!
nx = 4;  ny = 4;  nz = 4;
allp = [model; scene];
temp1 = min(allp);
x1 = temp1(1); y1 = temp1(2); z1 = temp1(3);
temp2 = max(allp);
x2 = temp2(1); y2 = temp2(2); z2 = temp2(3);

px = 0; %(x2 - x1) / 4
py = 0; %(y2 - y1) / 4
pz = 0; %(z2 - z1) / 4
xs = linspace(x1 - px, x2 + px, nx);
ys = linspace(y1 - py, y2 + py, ny);
zs = linspace(z1 - pz, z2 + pz, nz);

[xs, ys, zs] = meshgrid(xs, ys, zs);
ctrl_pts = [xs(:), ys(:), zs(:)];
config.ctrl_pts = ctrl_pts;
config.init_param = zeros(size(ctrl_pts,1),3);

[fp,fm] = gmmreg_cpd_Kevin(config);
DisplayPoints(fm,config.scene,3);

%% error
MSE = mean(sum((fm-config.scene)'.^2));

MSEs = [MSEs MSE];

end
