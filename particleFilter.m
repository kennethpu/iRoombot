function [Pset_f] = particleFilter(Pset_i,map,radius,zs,zb,g_fun,hs_fun,hb_fun,R,Qs,Qb)
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
%       zs          sonar sensor measurement for localization
%       zb          beacon sensor measurement for localization
%       g_fun       function handle to estimate predicted pose given
%                   initial pose and control, g(u_t, mu_(t-1)) 
%       hs_fun      function handle to estimate predicted sonar sensor
%                   measurement given a predicted pose and map, hs(mu_p, map)
%       hb_fun      function handle to estimate predicted beacon sensor
%                   measurement given a predicted pose and map, h(mu_p, map)
%       R           process noise covariance matrix
%       Qs          sonar measurement noise covariance matrix
%       Qb          beacon measurement noise covariance matrix
% 
%   OUTPUTS
%       Pset_f      final set of particles [Nx3]
%
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Final Competition
%   Pu, Kenneth (kp295)

%% ============================================================================
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

%% ============================================================================
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
        
        % Reset h,z,and Q vectors
        hu = [];
        zu = [];
        Qu = [];
        
        % Get predicted beacon measurement values
        hb = feval(hb_fun,Pset_t(p,:)',map);
        % Attempt to use only high confidence measurements
        % If a beacon is seen in zb and not in hb add the beacon to hb with
        % purposefully exaggerated distances 
        for i=1:size(zb,1)
            zu = [zu,zb(i,2:3)];
            if ~isempty(hb)
                [mem,idx] = ismember(zb(i,1),hb(:,1));
                if (mem)
                    hu = [hu,hb(idx,2:3)];
                    Qu = [Qu,Qb];
                else
                    [mem,idx] = ismember(-zb(i,1),hb(:,1)); 
                    if (mem)
                        hu = [hu,hb(idx,2:3)];
                        Qu = [Qu,Qb.*3];
                    else
                        hu = [hu,99,99];
                        Qu = [Qu,Qb];
                    end
                end
            else
                hu = [hu,99,99];
                Qu = [Qu,Qb];          
            end
            
        end
        
        % If a beacon is in 
        if ~isempty(hb)
            for i=find(hb(:,1)>0)'
                if ~isempty(zb)
                    [mem,idx] = ismember(hb(i,1),zb(:,1));
                    if (~mem)
                        hu = [hu,hb(i,2:3)];
                        zu = [zu,99,99];
                        Qu = [Qu,Qb];
                    end
                else
                    hu = [hu,hb(i,2:3)];
                    zu = [zu,99,99];
                    Qu = [Qu,Qb];
                end
            end
        end
        
        % Get predicted sonar measurement values
        hs = feval(hs_fun,Pset_t(p,:)',map);
        % Attempt to use only high confidence measurements
        % (no NaNs, predicted measurements do not go through optional walls)
        z_ind = find(~isnan(zs)&hs>0);
        if ~isempty(z_ind)
            zu = [zu,zs(z_ind)];
            Qu = [Qu,Qs(z_ind)];
            hu = [hu,hs(z_ind)];
        elseif isempty(zu) && ~isempty(find(~isnan(zs), 1))
            % Otherwise if sonar measurements are valid attempt to use
            % predicted measurements with increased measurement noise
            z_ind = find(~isnan(zs));
            zu = zs(z_ind);
            Qu = Qs(z_ind).*3;
            hu = abs(hs(z_ind));  
        end
            
%         zu
%         hu
%         Qu
        if isempty(zu)
            % If we get here all measurements were invalid
            % (sketchy...) set weight to 0. (HOPEFULLY RARE)
            P_w(p) = 0;  
        else
            P_w(p) = mvnpdf(zu,hu,diag(Qu));   
        end
        
%         % Get predicted sonar measurement values
%         hs = feval(hs_fun,Pset_t(p,:)',map);
%         % Attempt to use only high confidence measurements
%         % (no NaNs, predicted measurements do not go through optional walls)
%         z_ind = find(~isnan(zs)&hs>0);
%         if ~isempty(z_ind)
%             zu = zs(z_ind);
%             Qu = Qs(z_ind);
%             hu = hs(z_ind);
%             P_w(p) = mvnpdf(zu,hu,diag(Qu));
%         elseif ~isempty(find(~isnan(zs), 1))
%             % Otherwise if sonar measurements are valid attempt to use
%             % predicted measurements with increased measurement noise
%             z_ind = find(~isnan(z));
%             zu = zs(z_ind);
%             Qu = Qs(z_ind).*3;
%             hu = abs(hs(z_ind));
%             P_w(p) = mvnpdf(zu,hu,diag(Qu));
%         else
%             % If we get here all sonar measurements were invalid
%             % (sketchy...) set weight to 0. (HOPEFULLY RARE)
%             P_w(p) = 0;           
%         end
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
    Q(size(weights,1)) = 9999; 
    while i<=N
        if T(i)<Q(j)
            pset_f(i,:) = [pset_t(j,:) 1/N];
            i = i+1;
        else
            j=j+1;
        end
    end

end