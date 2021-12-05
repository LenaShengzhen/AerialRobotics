% if demoActivate exist and equal true, not clean all variable.
% runsim.m called by the demo,  not clean all variable
% runsim.m called by itself,  clean all variable
if (~exist('demoActivate')) || (demoActivate == false)
    close all;
    clear all;
    clc;
end

addpath(genpath('./'));


%% map generation
map = GridMap();
init_data
map.flag_obstacles();


%% path planning
nquad = length(start);
for qn = 1:nquad    
    disp('JPS time is :');
    tic
    path{qn} = JPS_3D(map, start{qn}, stop{qn});
    path{qn} = [start{qn}; path{qn}(1:end,:)];
    path{qn}(end + 1,:) = stop{qn};
    toc
end

%% delete the points of no-use
path{2} = simplify_path(map, path{1});

%% Generating Convex Polytopes
obps = PointCloudMap(map.blocks, map.margin);   % blocks of Metric Map change to point cloud

decomps{2} = []
disp('JPS -> SFC time is :');
tic
decomps{1} = SFC_3D(path{2}, obps, map.boundary); %  call SFC
toc

disp('JPS -> StarConvexMethod time is : ');
tic
[A, b] = SCMFromPath(path{2}, obps, map.boundary); 
toc


path{path_id}


%% draw path and blocks
if nquad == 1
    plot_path(path, map, decomps); 
else
    % you could modify your plot_path to handle cell input for multiple robots
end

% draw_ObcPoints
% makeGifAndJpg(1);     %figure(1): Graph without trajectory

%% Trajectory planning
[t_time, ts_par, x_par] = TrajectoryPlanning(path{path_id}, decomps{SFC_id}, time_allocation);


%% Trajectory tracking
disp('Generating Trajectory ...');
trajectory_generator([], [], path{path_id}, t_time, ts_par, x_par);
trajectory = test_trajectory(start, stop, path, true); % with visualization
disp('Blue line is Trajectory planning.');
disp('Red line is Trajectory tracking.');

%% Gif
% makeGifAndJpg(3);     %figure(3): Graph with trajectory
