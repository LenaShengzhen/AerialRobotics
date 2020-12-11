function [path] = JPS_3D(map, start, goal)
    
    map_start = map.points_to_idx(start);
    map_goal = map.points_to_idx(goal);
    
    map_visit = map.occ_map;
    
    fn = [0];  % fn = gn(EuclideanDistance) + hn(ManhattanDistance)
    all_direction = make_all_dir();
    nodeStart = struct( ...
        'pos', map_start, ...   % index of the point on the grid-map
        'fa', 1, ...        % fa records the index of the parent node in the closelist
        'id', 1, ...        % id records the index of node in closelist
        'dir', all_direction);
    
    openlist = [nodeStart];
    closelist = [nodeStart];
    map_visit(map_start(1), map_start(2), map_start(3)) = 1;
     
    findgoal = 0;
    while isempty(openlist) == 0
        
        [~, temid] = min(fn);
        node = openlist(temid);
       
        fn(temid) = [];
        openlist(temid) = [];
        fa = node.id;
        
        [e1, ~] = size(node.dir);
        for j = 1 : e1
            nodeNew = jump(map, map_goal, node.pos, node.dir(j,:));
            if (length(nodeNew.pos) == 0 )
                continue;
            end
            if (map_visit(nodeNew.pos(1), nodeNew.pos(2), nodeNew.pos(3)) == 0)
                nodeNew.fa = fa;
                nodeNew.id = length(closelist) + 1;  
                ds = ManhattanDistance(map_goal, nodeNew.pos) + EuclideanDistance(node.pos, nodeNew.pos);
                fn = [fn ds];
                openlist = [openlist; nodeNew];
                closelist = [closelist; nodeNew];
                map_visit(nodeNew.pos(1), nodeNew.pos(2), nodeNew.pos(3)) = 1;
                
                % find the goal
                if (nodeNew.pos == map_goal)
                    findgoal = 1;
                    break;
                end
            end
        end
        % find the goal
        if (findgoal == 1)
           break;
        end
    end
    
    
    path = [];
    k = length(closelist);
    while k > 1
        renode = closelist(k);
        float_pos = map.idx_to_points(renode.pos);     
        path = [float_pos; path];
        k = renode.fa;
    end
end

function dis = EuclideanDistance(pos1, pos2)
    dis = norm(pos1 - pos2, 2);
end

function dis = ManhattanDistance(pos1, pos2)
    dis = norm(pos1 - pos2, 1);
end

function ret = obCheck(map, pos)
    x = pos(1); y = pos(2); z = pos(3);
    ret = map.occ_map(x, y, z);
end

function all_direction = make_all_dir()
    x = [1 1 1 -1 -1 -1 0 0];
    b = unique(nchoosek(x,3),'rows');   
    A = []; 
    for i = 1 : 9                        
	    A = [A; perms(b(i,:))];         
    end
    all_direction = unique(A,'rows');   
end

function newnode = jump(map, goal, startpos, d)
    
    newnode = struct( ...
        'pos', [], ...
        'fa', 1, ...
        'id', 1, ...
        'dir', []);
    
    
    dir_Dimension = sum(d.^2);
    
    if (dir_Dimension == 0)
        return;
    end
    
    newpos = step(startpos, d);
    x = newpos(1); y = newpos(2); z = newpos(3);
    % ourside the grid on the newpos
    if outmap(map, newpos) == 1
        return;
    % meet the obstacle on the newpos 
    elseif map.occ_map(x, y, z) == 1
        return;
    elseif newpos == goal
        newnode.pos = newpos;
        return;
    end
    
   
    if dir_Dimension == 2 
        dside = dir2Dto1D(d);    
       
        if (obCheck(map, startpos+dside(1,:)) == 1) && (obCheck(map, startpos+dside(2,:)) == 1)
            return;
        end
    end
    
   
    dirs = hasForceNeighbour(map, newpos, d);
    if ~isempty(dirs)
        newnode.pos = newpos;
        newnode.dir = dirs;
        return;
    end
    
    % go 3d diagonal
        % sum(d.^2) = d(1)^2 + d(2)^2 + d(3)^2
    if(dir_Dimension == 3)
        % one 3d-diagonal change three 2d-diagonal + three straght line
        node1 = jump(map, goal, newpos, [0 d(2) d(3)]);
        node2 = jump(map, goal, newpos, [d(1) 0 d(3)]);
        node3 = jump(map, goal, newpos, [d(1) d(2) 0]);
        % three straght line
        node4 = jump(map, goal, newpos, [d(1) 0 0]);
        node5 = jump(map, goal, newpos, [0 d(2) 0]);
        node6 = jump(map, goal, newpos, [0 0 d(3)]);
        % find a ForceNegihbour
        if(length(node1.pos) + length(node2.pos) + length(node3.pos) + length(node4.pos) + length(node5.pos) + length(node6.pos) > 0)
            newnode.pos = newpos;
            newnode.dir = [d];
            
            newnode.dir = addDir(node1.pos, [0 d(2) d(3)], newnode.dir);
            newnode.dir = addDir(node2.pos, [d(1) 0 d(3)], newnode.dir);
            newnode.dir = addDir(node3.pos, [d(1) d(2) 0], newnode.dir);
            newnode.dir = addDir(node4.pos, [d(1) 0 0], newnode.dir);
            newnode.dir = addDir(node5.pos, [0 d(2) 0], newnode.dir);
            newnode.dir = addDir(node6.pos, [0 0 d(3)], newnode.dir);
            return;
        end
    end
    
    % go 2d diagonal
    if(dir_Dimension == 2)
        node4 = jump(map, goal, newpos, [d(1) 0 0]);
        node5 = jump(map, goal, newpos, [0 d(2) 0]);
        node6 = jump(map, goal, newpos, [0 0 d(3)]);
        % find a ForceNegihbour
        if(length(node4.pos) + length(node5.pos) + length(node6.pos) > 0)
            newnode.pos = newpos;
            newnode.dir = [d];
            % Add the direction of find ForceNegihbour
            newnode.dir = addDir(node4.pos, [d(1) 0 0], newnode.dir);
            newnode.dir = addDir(node5.pos, [0 d(2) 0], newnode.dir);
            newnode.dir = addDir(node6.pos, [0 0 d(3)], newnode.dir);
            return;
        end
    end
    
    % [go straght] or [go diagonal not find ForceNegihbour]
    newnode = jump(map, goal, newpos, d);
    return;
end

% if pos ~= Null, dirs = [dirs; d];
function dirs = addDir(pos, d, dirs)
    if(length(pos) > 0)
        dirs = [dirs; d];
    end
end


function dside = dir2Dto1D(d)
    dside = [[0 0 0] ; [0 0 0]];
    temi = 1;
    if(d(1) ~= 0) 
        dside(temi,:) = [d(1) 0 0];
        temi = temi + 1;
    end
    if(d(2) ~= 0) 
        dside(temi,:) = [0 d(2) 0];
        temi = temi + 1;
    end        
    if(d(3) ~= 0) 
        dside(temi,:) = [0 0 d(3)];
    end     
end


function ret = outmap(map, pos) 
    ret = false;
    x = pos(1); y = pos(2); z = pos(3);
    if(x < 1 || x > map.maxIndex(1) || y < 1 || y > map.maxIndex(2) || z < 1 || z > map.maxIndex(3))
        ret = true;
    end
end   


function ret = isforceNeighbourExist(map, pos, d)
    ret = false;
    x = pos(1); y = pos(2); z = pos(3);
    posNeighbour = pos + d;
    if outmap(map, pos) == 1 || ...   †
        map.occ_map(x, y, z) == 0 || ... 
        outmap(map, posNeighbour) == 1 || ... †
        map.occ_map(posNeighbour(1), posNeighbour(2), posNeighbour(3)) == 1 
        return;     
    end
    ret = true;
end

function dirs = hasForceNeighbour(map, pos, d)
    dirs = [];
    
    if outmap(map, pos + d) == 1 
        return;
    end
    
    % [go straght]  ----------------------------------------------
    if(sum(d.^2) == 1)
       
        if obCheck(map, pos + d) == 1
            return;
        end
        
        d1 = [abs(d(3)) abs(d(1)) abs(d(2))];
        d2 = [abs(d(2)) abs(d(3)) abs(d(1))];
        
        dob = [d1; -d1; d2; -d2; d1+d2; -d1-d2; d1-d2; -d1+d2];
        for i = 1 : 8
            if isforceNeighbourExist(map, pos + dob(i,:), d) == 1
                dirs = [dirs; dob(i,:) + d];
            end
        end
        
    % go 2D-diagonal ----------------------------------------------
    elseif  (sum(d.^2) == 2)
        dside = dir2Dto1D(d);
        
        for i = 1 : 2
            if isforceNeighbourExist(map, pos - dside(i,:), d - dside(i,:) ) == 1
                dirs = [dirs; d - 2*dside(i,:)];
            end
        end
        
        d1 = [abs(d(1))-1 abs(d(2))-1 abs(d(3))-1];
        if isforceNeighbourExist(map, pos + d1, d) == 1
            dirs = [dirs; d + d1];
        end
        if isforceNeighbourExist(map, pos - d1, d) == 1
            dirs = [dirs; d - d1];
        end
        
    % go 3D-diagonal  ----------------------------------------------
    else  % (sum(d.^2) == 3)
        
        dirs = hasForceNeighbour(map, pos, [0 d(2) d(3)]);
        dirs = [dirs; hasForceNeighbour(map, pos, [d(1) 0 d(3)])];
        dirs = [dirs; hasForceNeighbour(map, pos, [d(1) d(2) 0])];
        dirs = unique(dirs,'rows');        
        
        [e1, ~] = size(dirs);
        for i = 1 : e1
            if(dirs(i,:) == d) 
                dirs(i,:) = [];
            end
        end
    end
end

function newpos = step(pos, d)
    newpos = pos + d;
end
