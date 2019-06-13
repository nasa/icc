%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% Compute a semi-analytical derivative of the delivered science with      %
% respect to the initial location of the orbits. Experiments and scripts. %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2019 by California Institute of Technology.  ALL RIGHTS RESERVED. %
% United  States  Government  sponsorship  acknowledged.   Any commercial use %
% must   be  negotiated  with  the  Office  of  Technology  Transfer  at  the %
% California Institute of Technology.                                         %
%                                                                             %
% This software may be subject to  U.S. export control laws  and regulations. %
% By accepting this document,  the user agrees to comply  with all applicable %
% U.S. export laws and regulations.  User  has the responsibility  to  obtain %
% export  licenses,  or  other  export  authority  as may be required  before %
% exporting  such  information  to  foreign  countries or providing access to %
% foreign persons.                                                            %
%                                                                             %
% This  software  is a copy  and  may not be current.  The latest  version is %
% maintained by and may be obtained from the Mobility  and  Robotics  Sytstem %
% Section (347) at the Jet  Propulsion  Laboratory.   Suggestions and patches %
% are welcome and should be sent to the software's maintainer.                %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
a_num = 50*1e3;
e_num = 0.1;
i_anomaly_num = -pi/4;
Omega_num = 0;
omega_num = 0;
theta_num = 0;
verbose = true;

disp('Setting up problem')
%Setup an interesting problem
relay_parameters = [a_num, e_num, i_anomaly_num, Omega_num, omega_num, theta_num];
[sc_location,sc_vel]=op2rv(...
relay_parameters(1),relay_parameters(2),relay_parameters(3),...
relay_parameters(4),relay_parameters(5),relay_parameters(6),...
GM);
relay_initial_condition = [sc_location; sc_vel];

[goal, spacecraft, ErosGravity, time, GM] = comm_optimization_problem(relay_initial_condition, [0:2000:86400]);
[flows, effective_science, delivered_science, bandwidths_and_memories, dual_bandwidth_and_memory] = communication_optimizer(spacecraft);
% 
% % Call the differentiator to initialize
% [dxda, dxde, dxdi, dxdOmega, dxdomega, dxdtheta] = diff_op2rv(a_num, e_num, i_anomaly_num, Omega_num, omega_num, theta_num, GM, verbose);

% Get the problem size
N=length(spacecraft.orbits);
if N == 0
    disp('ERROR: no orbits')
else
    K = size(spacecraft.orbits{1},2);
end

reference_bandwidth = 250000;
reference_distance = 100000;
max_bandwidth = 100*1e6;

%% Derivative of delivered science with respect to bandwidth:
%  given by the duals of the network flow LP
dk_dbandwidth = reshape(dual_bandwidth_and_memory,1,K*N*N);


% Checking reshaping is correct
dk_dbandwidth2 = zeros(1,K*N*N);
finddkdbw = @(k,n1,n2) k + K*(n1-1) + K*N*(n2-1);
for k=1:K
    for n1=1:N
        for n2=1:N
            dk_dbandwidth2(finddkdbw(k,n1,n2)) = dual_bandwidth_and_memory(k,n1,n2);
        end
    end
end

assert(sum(abs(dk_dbandwidth2-dk_dbandwidth)) == 0 )
%% Derivative of bandwidth with respect to spacecraft locations:
%  computed according to simple bandwidth model and norm
dbandwidth_dlocation_full = zeros(K,N,N,K,N,3);
for k=1:K
    for i=1:N
        for j=1:N
            x1 = spacecraft.orbits{i}(:,k);
            x2 = spacecraft.orbits{j}(:,k);
            % Derivatives wrt all three directions are the same
            dbandwidth_dlocation_full(k,i,j,k,i,:) = diff_quadratic_comm_model_x1(x1, x2, reference_distance,reference_bandwidth,max_bandwidth);
            dbandwidth_dlocation_full(k,i,j,k,j,:) = diff_quadratic_comm_model_x2(x1, x2, reference_distance,reference_bandwidth,max_bandwidth);
        end
    end
end

dbandwidth_dlocation = reshape(dbandwidth_dlocation_full,[K*N*N,K*N*3]);

% Checking reshaping is correct
dbandwidth_dlocation2 = zeros(K*N*N,K*N*3);
finddbwdl_col = @(k,n,dim) k + K*(n-1) + K*N*(dim-1);
for k=1:K
    for n1=1:N
        for n2=1:N
            for kr=1:K
                for nr=1:N
                    for dim=1:3
                        dbandwidth_dlocation2(finddkdbw(k, n1, n2), finddbwdl_col(kr, nr, dim)) = dbandwidth_dlocation_full(k, n1, n2, kr, nr, dim);
                    end
                end
            end
        end
    end
end
assert(sum(sum(abs(dbandwidth_dlocation2-dbandwidth_dlocation))) == 0 )
%% Derivative of location with respect to initial orbital elements
% %  In this version, we propagate osculating Keplerian orbits.
% 
% dlocation_dop_full = zeros(K,N,3,6);
% disp('DTHETA IS WRONG!')
% for k=1:K
%     for i=1:N
%         % Should we differentiate wrt assumed or osculating orbit? WHat
%         % about theta?
%         [a_num, e_num, i_anomaly_num, Omega_num, omega_num, theta_num] = rv2op(spacecraft.orbits{i}(:,k), spacecraft.velocities{i}(:,k),GM);
%         [dxda, dxde, dxdi, dxdOmega, dxdomega, dxdtheta] = diff_op2rv(a_num, e_num, i_anomaly_num, Omega_num, omega_num, theta_num,GM);
%         if sum(isnan([dxda, dxde, dxdi, dxdOmega, dxdomega, dxdtheta]))>0
%             disp("TROUBLE")
%             i
%             k
%             spacecraft.orbits{i}(:,k)
%             spacecraft.velocities{i}(:,k)
%             [a_num, e_num, i_anomaly_num, Omega_num, omega_num, theta_num]
%         end
%         dlocation_dop_full(k,i,:,:) = [dxda, dxde, dxdi, dxdOmega, dxdomega, dxdtheta];
%     end
% end
% 
% dlocation_dop = reshape(dlocation_dop_full,[K*N*3,6]);
% 
% dk_dop = dk_dbandwidth*dbandwidth_dlocation*dlocation_dop;

%% Now, let's use SBDT to compute the state transition matrix, which gives
%  us exactly the partial of the current location wrt initial conditions.

stm_sc1 = spacecraft.state_transition_matrix{1};
stm_sc2 = spacecraft.state_transition_matrix{2};
stm_re  = spacecraft.state_transition_matrix{3};
stm_ca  = spacecraft.state_transition_matrix{4};

% stm(new_state, initial_state, time) =
% d(new_state/d(initial_state) @ time

dlocation_dic_full = zeros(K,N,3,6);
for sc = 1:length(spacecraft.state_transition_matrix)
    for t = 1:K
        for new_state = 1:1:3
            dlocation_dic_full(t,sc,new_state,:) = spacecraft.state_transition_matrix{sc}(new_state, :, t);
        end
    end
end

dlocation_dic = reshape(dlocation_dic_full,[K*N*3,6]);

% Checking reshaping is correct
dlocation_d2 = zeros(K*N*3, 6);
for k=1:K
    for n=1:N
        for dim=1:3
            for ic = 1:6
                dlocation_dop2(finddbwdl_col(k,n,dim), ic) = dlocation_dic_full(k,n,dim, ic);
            end
        end
    end
end
assert(sum(sum(abs(dlocation_dop2-dlocation_dic))) == 0 )

%% Putting it all together
dk_dic = dk_dbandwidth*dbandwidth_dlocation*dlocation_dic;

%% The reasoning:
% For every time step
% For every sc1
% Partial ka-ching/partial sc1: sum(links) Partial ka-ching/partial
% bandwidth*partial_bandwidth/partial x

% For every orbital parameter
% Partial x/partial op: symb
% Partial ka-ching/partial op: sum(times) sum(links) Partial ka-ching/partial
% bandwidth*partial_bandwidth/partial x * partial_x/partial op

%%
start_dist = 1000;
end_dist = 9999;
stride = 10;
steps = floor((end_dist-start_dist)/stride+1);

x1s = [start_dist:stride:end_dist; zeros(1,steps); zeros(1,steps)];
%x2s = [x1s(2,:); x1s(1,:); x1s(3,:)];
x1s = [start_dist:stride:end_dist];
x2s = zeros(size(x1s));
bws = zeros(length(x1s),1);
dbw_dx1s = zeros(size(x1s));
dbw_dx2s = zeros(size(x2s));

for it = 1:length(x1s)
    x1 = x1s(:, it);
    x2 = x2s(:, it);
    bw = quadratic_comm_model(x1, x2, reference_distance,reference_bandwidth,max_bandwidth);
    dbw_dx1 = diff_quadratic_comm_model_x1(x1, x2, reference_distance,reference_bandwidth,max_bandwidth);
    dbw_dx2 = diff_quadratic_comm_model_x2(x1, x2, reference_distance,reference_bandwidth,max_bandwidth);
    
    bws(it) = bw;
    dbw_dx1s(:, it) = dbw_dx1;
    dbw_dx2s(:, it) = dbw_dx2;
end

figure()
subplot(3,1,1)
plot(x1s, bws)
subplot(3,1,2)
hold all
%plot(x1s, sqrt(diag(dbw_dx1s'*dbw_dx1s)), ':')
%plot(x1s, sqrt(diag(dbw_dx2s'*dbw_dx2s)), '-.')
plot(x1s, dbw_dx1s, ':')
plot(x1s, dbw_dx2s, '-.')
legend('db/dx1', 'db/dx2')

subplot(3,1,3)
approx_dx = zeros(size(x1s));
for ix =2:length(x1s)
    approx_dx(ix) = (bws(ix) - bws(ix-1))/(x1s(ix) - x1s(ix-1));
end
plot(x1s, dbw_dx1s, ':')
hold all
plot(x1s, approx_dx, '-.')
legend('db/dx1', 'db/dx1 (approx)')

norm(approx_dx-dbw_dx1s)

%% Functions
function [bandwidth] = quadratic_comm_model(x1, x2, reference_distance,reference_bandwidth,max_bandwidth)
    bandwidth = min(max_bandwidth, reference_bandwidth*(reference_distance/norm(x2-x1,2))^2);
end

function [db_dx1] = diff_quadratic_comm_model_x1(x1, x2, reference_distance,reference_bandwidth,max_bandwidth)
if quadratic_comm_model(x1, x2, reference_distance,reference_bandwidth,max_bandwidth)>=max_bandwidth
    db_dx1 = zeros(size(x1));
else
    db_dx1 = reference_bandwidth*reference_distance^2*(-2/norm(x2-x1,2)^4) * (x1-x2);
end
end

function [db_dx2] = diff_quadratic_comm_model_x2(x1, x2, reference_distance,reference_bandwidth,max_bandwidth)
db_dx2 = - diff_quadratic_comm_model_x1(x1, x2, reference_distance,reference_bandwidth,max_bandwidth);
end

function [dxda, dxde, dxdi, dxdOmega, dxdomega, dxdtheta] = diff_op2rv(a_num, e_num, i_anomaly_num, Omega_num, omega_num, theta_num, GM, verbose)
persistent dxda_sym dxde_sym dxdi_sym dxdOmega_sym dxdomega_sym dxdtheta_sym;
% Will create an empty vector for some reason - so check for that
persistent variables_ready 

if nargin<8
   verbose = false; 
end

if (isempty(variables_ready))
    if verbose == true
        disp("Computing symbolic differentiation")
    end
    if nargin<7
        if verbose == true
            disp('Assuming we are on Earth')
        end
        GM=3.986*1e14;
    end
    
    syms a e i_anomaly Omega omega theta
    syms par(a, e, i_anomaly, Omega, omega, theta)
    syms r(a, e, i_anomaly, Omega, omega, theta)
    syms rvec(a, e, i_anomaly, Omega, omega, theta)
    syms vr(a, e, i_anomaly, Omega, omega, theta) vt(a, e, i_anomaly, Omega, omega, theta)
    syms vvec(a, e, i_anomaly, Omega, omega, theta)
    syms T(a, e, i_anomaly, Omega, omega, theta)
    syms rvec(a, e, i_anomaly, Omega, omega, theta) vvec(a, e, i_anomaly, Omega, omega, theta)
    
    par(a, e, i_anomaly, Omega, omega, theta) = a*(1-e^2);
    r(a, e, i_anomaly, Omega, omega, theta) = par/(1+e*cos(theta));
    rvec(a, e, i_anomaly, Omega, omega, theta)=[r*cos(theta); r*sin(theta); 0];
    vr(a, e, i_anomaly, Omega, omega, theta)=sqrt(GM/(a*(1-e^2)))*e*sin(theta);
    vt(a, e, i_anomaly, Omega, omega, theta)=sqrt(GM/(a*(1-e^2)))*(1+e*cos(theta));
    vvec(a, e, i_anomaly, Omega, omega, theta)=[vr*cos(theta)-vt*sin(theta); vr*sin(theta)+vt*cos(theta);0];
    T(a, e, i_anomaly, Omega, omega, theta)=rotmat(omega,3)*rotmat(i_anomaly,1)*rotmat(Omega,3);
    
    rvec(a, e, i_anomaly, Omega, omega, theta)=T'*rvec;
    % vvec(a, e, i_anomaly, Omega, omega, theta)=T'*vvec;
    
    
    dxda_sym = matlabFunction(diff(rvec,a));
    dxde_sym = matlabFunction(diff(rvec,e));
    dxdi_sym = matlabFunction(diff(rvec,i_anomaly));
    dxdOmega_sym = matlabFunction(diff(rvec,Omega));
    dxdomega_sym = matlabFunction(diff(rvec,omega));
    dxdtheta_sym = matlabFunction(diff(rvec,theta));
    % Consider expressing theta = f(M), M = theta0+t/T*2pi, and differentiate wrt theta0 - it makes way more sense. See https://en.wikipedia.org/wiki/True_anomaly
    variables_ready = 1;
else
    if verbose == true
        disp("Using cached variables")
    end
end

dxda = double(dxda_sym(a_num, e_num, i_anomaly_num, Omega_num, omega_num, theta_num));
dxde = double(dxde_sym(a_num, e_num, i_anomaly_num, Omega_num, omega_num, theta_num));
dxdi = double(dxdi_sym(a_num, e_num, i_anomaly_num, Omega_num, omega_num, theta_num));
dxdOmega = double(dxdOmega_sym(a_num, e_num, i_anomaly_num, Omega_num, omega_num, theta_num));
dxdomega = double(dxdomega_sym(a_num, e_num, i_anomaly_num, Omega_num, omega_num, theta_num));
dxdtheta = double(dxdtheta_sym(a_num, e_num, i_anomaly_num, Omega_num, omega_num, theta_num));
end