function [Pset_f] = particleFilter(Pset_i,map,radius,z,g_fun,h_fun,R,Q,usebeacon)
% PARTICLEFILTER: generic particle filter to perform one round of sampling
% and resampling to output a new set of particles
% 
%   [PSET_F] = PARTICLEFILTER(PSET_I,MAP,Z,G_FUN,H_FUN,R,Q) returns 
%   a new set of particles resutling from one step of the particle filter
% 
%   INPUTS
%       Pset_i      initial set of particles [Nx3]
%       map         M-by-4 matrix containing the coordinates of walls in 
%                   the environment: [x1, y1, x2, y2]
%       radius      radius of robot
%       z           sensor measurement for localization
%       g_fun       function handle to estimate predicted pose given
%                   initial pose and control, g(u_t, mu_(t-1)) 
%       h_fun       function handle to estimate predicted sensor
%                   measurement given a predicted pose and map, h(mu_p, map)
%       R           process noise covariance matrix
%       Q           measurement noise covariance matrix
%       usebeacon   flag indicating if we are using beacon or sonar data
% 
%   OUTPUTS
%       Pset_f      final set of particles [Nx3]
%
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Final Competition
%   Pu, Kenneth (kp295)

%==============================================================================
% INITIALIZE VARIABLES
%==============================================================================
% Get size of initial particle set
N = size(Pset_i,1);

% Get map dimensions
x_min = min(min(map(:,1)),min(map(:,3)));
x_max = max(max(map(:,1)),max(map(:,3)));
y_min = min(min(map(:,2)),min(map(:,4)));
y_max = max(max(map(:,2)),max(map(:,4)));
xRange = [x_min,x_max];
yRange = [y_min,y_max];

% Reduce size of z and Q accordingly if measurement includes NaN's
z_ind = find(~isnan(z));
zu = z(z_ind);
Qu = Q(z_ind,z_ind);

%==============================================================================
% IMPORTANCE SAMPLING
%==============================================================================
% 1. Find updated positions of each particle given input control and process
% noise covariance
Pset_t = zeros(N,3);  % Initialize array to hold predicted particles positions
for p = 1:N
    Pset_t(p,:) = mvnrnd(feval(g_fun,Pset_i(p,1:end-1)),R);
end
% 2. Find weights of each particle given actual and expected measurements
P_w = zeros(N,1);     % Initialize array to hold particle weights
for p = 1:N
    if (ptIsFree(Pset_t(p,1:2),map,radius,xRange,yRange))
        % if in map, weight is probability of getting measurement
        % reduce size of h accordingly if measurement includes NaN's
        h = feval(h_fun,Pset_t(p,:)',map);
        hu = h(z_ind);
        P_w(p) = mvnpdf(zu,hu,Qu);
    else
        % otherwise weight = 0
        P_w(p) = 0;
    end
end
% 3. Normalize the particle weights
P_w = P_w./sum(P_w);
%==============================================================================
% SELECTION / RESAMPLING
%==============================================================================
% 4. Resample particles according to particle weights
Pset_f = resampleStratified(Pset_t,P_w);

end

%% ============================================================================
% resampleMultinomial
%==============================================================================
%   Helper function to resample particles using multinomial resampling
%
%   INPUTS 
%       pset_t  N-by-3 array containing predicted particle positions
%       weights Normalized particle weights
% 
%   OUTPUTS 
%       pset_f  N-by-4 array containing resampled particles and weights
function pset_f = resampleMultinomial(pset_t,weights)
    % Get number of particles to resample
    N = size(pset_t,1);
    % Initialize array to hold new particle set
    pset_f = zeros(N,4);  
    for p = 1:N
        idx = find(rand<=cumsum(weights),1);
        pset_f(p,:) = [pset_t(idx,:) 1/N];
    end
end

%% ============================================================================
% resampleStratified
%==============================================================================
%   Helper function to resample particles using stratified resampling
%
%   INPUTS 
%       pset_t  N-by-3 array containing predicted particle positions
%       weights Normalized particle weights
% 
%   OUTPUTS 
%       pset_f  N-by-4 array containing resampled particles and weights
function pset_f = resampleStratified(pset_t,weights)
    % Get number of particles to resample
    N = size(pset_t,1);
    % Initialize array to hold new particle set
    pset_f = zeros(N,4);  

    % Pre-partition (0,1] interval into N disjoint sets
    T = ones(1,N+1);
    for i=1:N
        T(i) = rand(1,1)/N + (i-1)/N;
    end

    % Sample particles by weight using the created sets
    i=1;
    j=1;
    Q = cumsum(weights);
    while i<=N
        if T(i)<Q(j)
            pset_f(i,:) = [pset_t(j,:) 1/N];
            i = i+1;
        else
            j=j+1;
        end
    end

end