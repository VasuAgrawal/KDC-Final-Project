function [joint_traj] = DLS_Traj(th_start,xi,qlast,targ_pos)
%th is a column vector of thetas 1 through n
%xi is a matrix of twists [xi_1 xi_2.....xi_n]
%qlast is the last q vector (q5 if n = 5) (column vector)
%targ_pos is an [x; y; z] coordinate of a target position

%ee is for end effector
error = .05;
itLim = 1000;
tstep = .01;
Lmda = .1; %may need to be tuned

I = eye(6);

n = length(th_start);
joint_traj = [th_start];
gslee_0 = [eye(3) qlast; [0 0 0] 1];

count = 0;
th = th_start;

gslee_th = eye(4);
for i = 1:n
    gslee_th = gslee_th * expm(wedge(xi(:,i))*th(i));
end
gslee_th = gslee_th * gslee_0;

curr_pos = gslee_th(1:3,4);

e = targ_pos-curr_pos; %if we impliment orientations include quaternion subtraction too

while and(not(sum(abs(e)>error))==0,(count<itLim))    
    count = count+1;
    
    J = zeros(6,n);
    gslee_th = eye(4);
    for i = 1:n
        row = eye(4);
        for j = i:n
            row = row * expm(wedge(xi(:,j))*th(j));
        end
        J(:,i) = adj(row) \ xi(:,i);
        gslee_th = gslee_th * expm(wedge(xi(:,i))*th(i));
    end
    gslee_th = gslee_th * gslee_0;

    e = [e; [0 0 0]']; %since we are not computing quaternion/orientation error
    
    deltaTh = J'*inv(J*J'+(Lmda^2)*I)*e;
    th = th+deltaTh*tstep;
    
    joint_traj = [joint_traj th];
    
    curr_pos = gslee_th(1:3,4);

    e = targ_pos-curr_pos; 
end
end


