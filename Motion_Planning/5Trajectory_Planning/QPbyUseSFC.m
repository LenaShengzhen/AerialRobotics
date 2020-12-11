function X = QPbyUseSFC(waypts, ts, decomp)

%% condition
traj.n_order = 7;
traj.n_poly = size(waypts, 1) - 1;
traj.p0 = waypts(1,:);
traj.pe = waypts(end,:);
traj.v0 = [0,0,0];
traj.ve = [0,0,0];
traj.a0 = [0,0,0];
traj.ae = [0,0,0];


%% trajectory plan
[minSnapValue, px, py, pz] = minimum_snap_three_axis_SFC(traj, ts, decomp);

disp(['minSnapValue is : ',num2str(minSnapValue)]);

X = [px py pz];
end

% v0 = [1*3]
function [minValue, px, py, pz] = minimum_snap_three_axis_SFC(traj, ts, decomp)
    % Dimension
    dim = 3;
    n_order = traj.n_order;
	p0 = traj.p0;
	pe = traj.pe;
    v0 = traj.v0;
    ve = traj.ve;
    a0 = traj.a0;
    ae = traj.ae;
	
	n_poly = traj.n_poly;
	n_coef = n_order+1;
	
	%% compute Q,  Q is a Symmetric matrix
    % Q_1 =   [0    0   0]
    %         [0    f   f]
    %         [0    f   f]          % f =  integral of ts(i) ~ ts(i+1) 
    % Q_x =   [Q_1  0     0    0]
    %         [0    Q_2   0    0]
    %         [0    0   Q_...  0]
    %         [0    0     0  Q_n]   % n = n_poly
    % Q_all = [Q_x  0     0]
    %         [0    Q_y   0]
    %         [0    0   Q_z]        % Q_x == Q_y == Q_z
    Q_x = [];
	for i=1:n_poly
	    Q_x = blkdiag(Q_x,calc_Q(n_order,4,ts(i),ts(i+1)));
    end
    zeroM = zeros(size(Q_x));
    Q_all = [Q_x zeroM zeroM; zeroM Q_x zeroM; zeroM zeroM Q_x];
    xlen = size(Q_all,1);
	f = zeros(xlen,1);      %% min (1/2p^TQp + f^Tp)
	
    %% compute Aeq x = beq
    % beacuse Aeq_x == Aeq_y == Aeq_z
    % Aeq = [Aeq_x  0      0]
    %       [0    Aeq_y    0]
    %       [0      0  Aeq_z] 
	neq = 8;  %% (8 equations)
    eqNum_x = (7*(n_poly-1)+neq);
	Aeq_x = zeros(eqNum_x, n_coef*n_poly);
	beq = zeros(dim*eqNum_x, 1);
	
	% start/terminal pva constraints  (8 equations)
	Aeq_x(1:4,1:n_coef) = [calc_dc(ts(1),n_order,0);
	                     calc_dc(ts(1),n_order,1);
	                     calc_dc(ts(1),n_order,2);
	                     calc_dc(ts(1),n_order,3)];
	Aeq_x(5:8,n_coef*(n_poly-1)+1:n_coef*n_poly) = ...
	                    [calc_dc(ts(end),n_order,0);
	                     calc_dc(ts(end),n_order,1);
	                     calc_dc(ts(end),n_order,2);
	                     calc_dc(ts(end),n_order,3)];
	beq(1:8,1)                              = [p0(1),v0(1),a0(1),0,pe(1),ve(1),ae(1),0]';
    beq((eqNum_x   + 1):(eqNum_x   + 8),1)  = [p0(2),v0(2),a0(2),0,pe(2),ve(2),ae(2),0]';
    beq((eqNum_x*2 + 1):(eqNum_x*2 + 8),1)  = [p0(3),v0(3),a0(3),0,pe(3),ve(3),ae(3),0]';
	
	% continuous constraints  ((n_poly-1)*7 equations)
	for i=1:n_poly-1
		t_derc_p = calc_dc(ts(i+1),n_order,0);
        t_derc_v = calc_dc(ts(i+1),n_order,1);
        t_derc_a = calc_dc(ts(i+1),n_order,2);
        t_derc_j = calc_dc(ts(i+1),n_order,3);  % jerk
        t_derc_4 = calc_dc(ts(i+1),n_order,4);
        t_derc_5 = calc_dc(ts(i+1),n_order,5);
        t_derc_6 = calc_dc(ts(i+1),n_order,6);
        Aeq_x(neq+1,n_coef*(i-1)+1:n_coef*(i+1))=[t_derc_p,-t_derc_p];
        Aeq_x(neq+2,n_coef*(i-1)+1:n_coef*(i+1))=[t_derc_v,-t_derc_v];
        Aeq_x(neq+3,n_coef*(i-1)+1:n_coef*(i+1))=[t_derc_a,-t_derc_a];
        Aeq_x(neq+4,n_coef*(i-1)+1:n_coef*(i+1))=[t_derc_j,-t_derc_j];
        Aeq_x(neq+5,n_coef*(i-1)+1:n_coef*(i+1))=[t_derc_4,-t_derc_4];
        Aeq_x(neq+6,n_coef*(i-1)+1:n_coef*(i+1))=[t_derc_5,-t_derc_5];
        Aeq_x(neq+7,n_coef*(i-1)+1:n_coef*(i+1))=[t_derc_6,-t_derc_6];
        neq = neq + 7;
    end
	
  
    zeroM = zeros(size(Aeq_x)); 
    Aeq = [Aeq_x zeroM zeroM; zeroM Aeq_x zeroM; zeroM zeroM Aeq_x];
    
    %% compute Ax <= b
    A = [];
    b = [];
    ieq = 1;
    xll = xlen / dim;  % 1/3 of xlen,  = n_coef * (n_poly-1)
    for i = 1 : n_poly
        planesCell = decomp.lines_{i}.polyhedron_.polys_;
        [~, lenj] = size(planesCell);
        for j = 1 : lenj
            n = planesCell{j}.n_;
            p = planesCell{j}.p_;
            nx = zeros(1, xlen);
            % add (Denominator + 1) points on the poly's Ax<b 
            Denominator = 2;  % Denominator > 0
            for k = 0 : Denominator
                tvec_mid = calc_dc(ts(i) +  k*(ts(i+1) - ts(i))/Denominator, n_order, 0);   % [1*8]vector
                nx(1, 8*(i-1) + 1 : 8*(i-1) + 8)                = tvec_mid.*n(1);
                nx(1, 8*(i-1) + 1 + xll : 8*(i-1) + 8 + xll)    = tvec_mid.*n(2);
                nx(1, 8*(i-1) + 1 + 2*xll : 8*(i-1) + 8 + 2*xll)= tvec_mid.*n(3);
                A(ieq,:) = nx;
                b(ieq,:) = dot(n, p);
                % dot(plane.n, p1 - plane.p) < 0 
                % => dot(plane.n, p1) - dot(plane.n, plane.p) < 0 
                % => dot(plane.n, p1) < dot(plane.n, plane.p) 
                %% Ax < b
                ieq = ieq + 1;
            end
        end
    end
   
    
%     % Modify the number of iterations
    options = optimoptions('fmincon');
    options.MaxIterations = 2000;   % Defaults = 1000
    lb = []; ub = []; x0 = []; 
    p = quadprog(Q_all,f,A,b,Aeq,beq, lb,ub,x0, options);

% 	p = quadprog(Q_all,f,A,b,Aeq,beq);
    
    minValue = p'*Q_all*p;
	
    px = p(1:xll,1);
    py = p(xll + 1:2*xll,1);
    pz = p(xll*2 + 1:3*xll,1);
end

