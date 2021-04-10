start = {[]};
stop  = {[]};

if (~exist('demoActivate')) || (demoActivate == false)
    choose_map  = 2;
end
demoActivate    = false;
if (~exist('path_id'))
    path_id     = 2;            % Path planning: 2 = JPS, 4 = JPS-PF [This feature has not been uploaded yet]
end
SFC_id      = path_id/2;        % Path planning: 2 = path_id(4)'s SFC, 1 = path_id(2)'s SFC
speed       = 1;
acc         = 0;

time_allocation.type = 'trapzoidSpeed';
%time_allocation.type = 'averageSpeed';

switch choose_map
   case 1   %% 3 blocks
      map.load_map('0Maps/ellipsoid.txt', 0.2, 0.2, 0.1);
      start = {[1.5 0.2 0.2]};
      stop  = {[1.5 2.0 1.9]};
      speed = 1;
      acc   = 2; 
   case 2
      map.load_map('0Maps/3dCorner.txt', 0.1, 0.1, 0.1);
      start = {[2.6  1.0 1.0]};
      stop  = {[2.5  3.0 2.6]};
      acc   = 2;        % acceleration 
      if path_id == 4
          speed = 1.0767; % for path_id = 4, time = 4.3682, snap = 350.3038, fly use  4.1s
      else
          speed = 0.8; % for path_id = 2, time = 4.3681, snap = 895.7681, fly use  4.15s
      end
   case 3   %% Z_3blocks
      map.load_map('0Maps/Z_3blocks.txt', 0.2, 0.2, 0.1);
      start = {[-1 1.5 2]};
      stop  = {[11 1.5 1]};
      acc   = 2;
      if path_id == 4
          speed = 1.4;      %for path_id = 4, time = 12.9343, snap = 58.376, fly use  12.45s
      else
          speed = 1.10423;  %for path_id = 2, time = 12.9343, snap = 300.3111, fly use  13.9s  
      end

    otherwise 
end

% Must be after acc assignment above
time_allocation.avg_speed   = speed;
time_allocation.acc         = acc;
