function [V,E] = treeMIRRT(map,waypoints,ECwaypoints,step,n,radius,seed)
% TREEMIRRT: Outputs a roadmap covering Q_free of a given environment by
% growing trees from each waypoint and connecting them until only one tree
% remains
% 
%   TREEMIRRT(MAP,WAYPOINTS,STEP,N,RADIUS) returns 
%   a roadmap covering Q_free of a given environment by growing trees from 
%   each waypoint and connecting them until only one tree remains
% 
%   INPUTS  
%       map         N-by-4 matrix containing the coordinates of walls in the 
%                   environment: [x1, y1, x2, y2]
%       waypoints   N-by-2 matrix containing the coordinates of waypoints in
%                   the environment: [x, y]
%       ECwaypoints N-by-2 matrix containing the coordinates of extra credit 
%                   waypoints in the environment: [x, y]
%       step        step size
%       n           stopping criteria (number of loops to run before giving up)
%       radius      radius of robot
%
%   OUTPUTS 
%       V           Set of nodes in roadmap
%       E           Set of edges in roadmap
%
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Final Competition
%   Pu, Kenneth (kp295)

%% ============================================================================
% INITIALIZE VARIABLES
%==============================================================================
% Get map dimensions
x_min = min(min(map(:,1)),min(map(:,3)));
x_max = max(max(map(:,1)),max(map(:,3)));
y_min = min(min(map(:,2)),min(map(:,4)));
y_max = max(max(map(:,2)),max(map(:,4)));
xRange = [x_min,x_max];
yRange = [y_min,y_max];

% Node and edge sets
Vset = {};
Eset = {};
   
% Add waypoints to node set, initialize corresponding empty edge list
for i=1:size(waypoints,1)
    Vset{i} = waypoints(i,:);
    Eset{i} = [];
end

% Current size of Vset (so we can append to Vset)
sumidx = size(Vset,2);

% Add EC waypoint to node set, initialize corresponding empty edge list
for i=1:size(ECwaypoints,1)
    Vset{sumidx+i} = ECwaypoints(i,:);
    Eset{sumidx+i} = [];
end

% Current size of Vset (indicates waypoints)
sumidx = size(Vset,2);

% Deterministically sample valid points in the map to assist in connecting waypoints
S = deterministicDist(map,seed,radius,xRange,yRange);

% Add seed points to node set, initialize corresponding empty edge list
for i=1:size(S,1)
    Vset{sumidx+i} = S(i,:);
    Eset{sumidx+i} = [];
    plot(S(i,1),S(i,2),'bo');
end

% Attempt to connect all the nodes in the node set
[Vset,Eset,done] = tryConnectTrees(Vset,Eset,map,radius,sumidx);

% If all the waypoints are already connected, we are done!
if (done)
    idx = find(~cellfun(@isempty,Vset),1);
    V = Vset{idx};
    E = Eset{idx};
else
    % Otherwise grow each tree and merge whenever possible until all
    % waypoints are connected in one tree
    for i=1:n
        for t=find(~cellfun(@isempty,Vset))
            % If current tree ever becomes empty (due to merging) move on to
            % next nonempty tree in Vset
            if ~size(Vset{t},1)
                continue;
            end
            % Sample random point in Q_free
            q_rand = sampleRand(map,xRange,yRange,radius);
            % Find closest q in Vset{t} to q_rand
            q_near = findNear(q_rand,Vset{t});
            % Create point on line (q_near,q_rand) that is step size away from q_near
            q_new = createNew(q_near,q_rand,step);
            % If q_new is free and the edge (q_near,q_new) is free, add
            % them to Vset{t} and Eset{t} respectively
            if (ptIsFree(q_new,map,radius,xRange,yRange) && edgeIsFree([q_near,q_new],map,radius))
                Vset{t} = [Vset{t};q_new];
                Eset{t} = [Eset{t};q_near,q_new];

                % Plot q_new and edge (q_near,q_new) 
                plot(q_new(1),q_new(2),'bo');
                plot([q_near(1),q_new(1)],[q_near(2),q_new(2)],'b');
                
                % Attempt to connect trees
                [Vset,Eset,done] = tryConnectTrees(Vset,Eset,map,radius,sumidx);
                
                % If all the waypoints are connected, we are done!
                if (done)
                    idx = find(~cellfun(@isempty,Vset),1);
                    V = Vset{idx};
                    E = Eset{idx};
                    break;
                end
                pause(0.02);
            end
        end
        if (done)
            break;
        end
    end
    if (~done)
        % If we were not able to merge all the trees return the first nonempty 
        % tree we find (HOPEFULLY RARE CASE)
        idx = find(~cellfun(@isempty,Vset),1);
        V = Vset{idx};
        E = Eset{idx};
    end
        
end

end

%% ============================================================================
% mergeTrees
%==============================================================================
%   Helper function that "merges" two selected trees. The merge is done by
%   taking all the nodes and edges contained in one tree and appending them
%   to the corresponding node and edge arrays for the other tree. As a
%   result one tree will contain the nodes and edges of BOTH, while the
%   other tree will be EMPTY. In this way the two trees have been
%   effectively merged
%
%   INPUTS 
%       map     N-by-4 matrix containing the coordinates of walls in the 
%               environment: [x1, y1, x2, y2]
%       Vset    M-length cell array holding the nodes contained in each tree
%       Eset    M-length cell array holding the edges contained in each tree
%       a       Index in Vset for tree A
%       b       Index in Vset for tree B
%       edges   L-by-4 matrix containing all the valid edges between tree A
%               and tree B
%       wpts    The total number of waypoints (including ECwaypoints)
% 
%   OUTPUTS 
%       Vret    Updated M-length cell array holding the nodes contained in 
%               each tree
%       Eret    Updated M-length cell array holding the edges contained in 
%               each tree
%       keep    Index in Vset for tree we keep
%       done    Boolean flag indicating whether all waypoints are connected
function [Vret,Eret,keep,done] = mergeTrees(Vset,Eset,a,b,edges,wpts)
    % Initialize flag indicating completion (initially FALSE)
    done = 0;    
    
    % If tree B contains a waypoint, keep tree B. This helps us keep track
    % of whether or not a tree contains waypoints
    if (b <= wpts)
        Vset{b} = [Vset{b};Vset{a}];
        Vset{a} = [];
        Eset{b} = [Eset{b};Eset{a};edges];
        Eset{a} = [];
        keep = b;
    else
        % Otherwise by default keep tree A
        Vset{a} = [Vset{a};Vset{b}];
        Vset{b} = [];
        Eset{a} = [Eset{a};Eset{b};edges];
        Eset{b} = [];
        keep = a;
    end
    % Check if we are done. There are two possible done criteria:
    %   1) Amongst trees containing waypoints (index <= wpts), there is 
    %       only one non-empty tree. This means all waypoints have been 
    %       connected. At this point we can terminate.
    %   2) There is only one non-empty tree. This means all trees have 
    %       been connected and we can terminate.
    % Criteria 1 is required because occasionally we wind up with a tree
    % that is impossible to connect to the rest of the graph. We are
    % guaranteed that this will never occur with waypoints.
    if (size(find(find(~cellfun(@isempty,Vset))<=wpts),2)==1) ...
            || (size(find(~cellfun(@isempty,Vset)),2)==1)
        done = 1;
    end
    % Return updated Vset and Eset so our changes are not lost
    Vret = Vset;
    Eret = Eset;
end

%% ============================================================================
% tryConnectTrees
%==============================================================================
%   Helper function that takes as input multiple trees and attempts to
%   merge them into a single tree
%
%   INPUTS 
%       map     N-by-4 matrix containing the coordinates of walls in the 
%               environment: [x1, y1, x2, y2]
%       Vset    M-length cell array holding the nodes contained in each tree
%       Eset    M-length cell array holding the edges contained in each tree
%       radius  radius of robot
%       wpts    The total number of waypoints (including ECwaypoints)
% 
%   OUTPUTS 
%       Vret    Updated M-length cell array holding the nodes contained in 
%               each tree
%       Eret    Updated M-length cell array holding the edges contained in 
%               each tree
%       done    Boolean flag indicating whether all waypoints are connected
function [Vret,Eret,done] = tryConnectTrees(Vset,Eset,map,radius,wpts)

    % Flag indicating whether tree is successfully constructed
    done = 0; 

    % Iterate through each nonempty tree in Vset -> tree A
    for a=find(~cellfun(@isempty,Vset)) 
        % Iterate through each node in tree A
        for i=1:size(Vset{a},1)
            if ~size(Vset{a},1)
                % If tree A ever becomes empty (due to merging) move on to
                % next nonempty tree in Vset
                continue;
            end
            % Iterate through each nonempty tree in Vset -> tree B
            for b=find(~cellfun(@isempty,Vset))                
                if ~size(Vset{a},1)
                    % If tree A ever becomes empty (due to merging) stop
                    % comparing against tree B
                    break;
                end
                % Check that tree A and tree B refer to different trees
                if (a~=b)
                    % Check if any of the edges between tree A and tree B are valid
                    for j=1:size(Vset{b},1)
                        edge = [Vset{a}(i,:),Vset{b}(j,:)];
                        if (edgeIsFree(edge,map,radius))
                            % If a valid edge exists, merge tree A and tree B
                            plot([edge(1),edge(3)],[edge(2),edge(4)],'b');
                            [Vset,Eset,keep,done] = mergeTrees(Vset,Eset,a,b,edge,wpts);
                            
                            % Attempt to connect all nodes of the new tree
                            for va=1:size(Vset{keep},1)
                                for vb=1:size(Vset{keep},1)
                                    if va~=vb
                                        edge = [Vset{keep}(va,:),Vset{keep}(vb,:);];
                                        if (~ismember(edge,Eset{keep},'rows') && edgeIsFree(edge,map,radius))
                                            Eset{keep} = [Eset{keep};edge];
                                            plot([edge(1),edge(3)],[edge(2),edge(4)],'b');
                                        end
                                    end
                                end
                            end 
                            
                            % Stop searching through tree B
                            break;
                        end
                    end
                end
                % Break out of loop once all waypoints are connected to the
                % tree
                if (done)
                    break;
                end
            end        
            if (done)
                break;
            end
        end       
        if (done)
            break;
        end
    end
    % Return updated Vset and Eset so our changes are not lost
    Vret = Vset;
    Eret = Eset;
end

%% ============================================================================
% deterministicDist
%==============================================================================
%   Helper function to sample up to n points using a deterministic
%   low-dispersion sampling
%
%   INPUTS 
%       map    N-by-4 matrix containing the coordinates of walls in the 
%              environment: [x1, y1, x2, y2]
%       n      Number of vertices to include in roadmap
%       radius radius of robot
%       xRange 1-by-2 vector representing interval of possible X values
%       yRange 1-by-2 vector representing interval of possible Y values
% 
%   OUTPUTS 
%       V      N-by-2 list of all nodes in graph
function V = deterministicDist(map,n,radius,xRange,yRange)
    % Initialize variables
    V = [];
    
    % Divide map into grids. The number of grids is the largest square
    % number that is less than n. ex) n = 12 -> 9 grids
    numStep = floor(sqrt(n));
    x_min = xRange(1);
    x_max = xRange(2);
    x_step = (x_max-x_min)/numStep;
    y_min = yRange(1);
    y_max = yRange(2);
    y_step = (y_max-y_min)/numStep;
    
    % Sample the center of each previously defined grid. If the pt is free,
    % add to V
    for j=0:(numStep-1)
        for i=0:(numStep-1)
            pt = [x_min+(i+0.5)*x_step,y_min+(j+0.5)*y_step];
            if (ptIsFree(pt,map,radius,xRange,yRange))
                plot(pt(1),pt(2),'bo');
                V = [V;pt];
            end
        end
    end
end

%% ============================================================================
% sampleRand
%==============================================================================
%   Helper function to find a random free point within the given x/y ranges
%
%   INPUTS 
%       map    N-by-16 matrix containing the coordinates of (up to 8) vertices 
%              corresponding to obstacles in the environment
%               OR
%              N-by-4 matrix containing the coordinates of walls in the 
%              environment: [x1, y1, x2, y2]
%       xRange 1-by-2 vector representing interval of possible X values
%       yRange 1-by-2 vector representing interval of possible Y values
% 
%   OUTPUTS 
%       q_rand 1-by-2 array containing x/y coordinates random free point in
%              map
function q_rand = sampleRand(map,xRange,yRange,radius)
    % Sample initial random point in map
    q_rand = unifrnd([xRange(1),yRange(1)],[xRange(2),yRange(2)]);
    
    % Sample new points until we find one that is free
    while (~ptIsFree(q_rand,map,radius,xRange,yRange))
        q_rand = unifrnd([xRange(1),yRange(1)],[xRange(2),yRange(2)]);
    end
end

%% ============================================================================
% findNear
%==============================================================================
%   Helper function to find the closest point in a list to a target point
%
%   INPUTS 
%       pt  1-by-2 array containing x/y coordinates of test point
%       V   N-by-2 array containing a list of x/y coordinates of candidate
%           points
% 
%   OUTPUTS 
%       cpt 1-by-2 array containing x/y coordinates of closest point in V
%           to test point
function cpt = findNear(pt,V)
    % Initialize variables
    min_dist = 9999;
    cpt = [];
    
    % Iterate through points in V to find closest point to pt
    for i=1:size(V,1)
        dist = pdist([pt;V(i,:)],'euclidean');
        if dist < min_dist
            min_dist = dist;
            cpt = V(i,:);
        end
    end
end

%% ============================================================================
% createNew
%==============================================================================
%   Helper function to find a new point that is a given step size away from
%   a given point in the direction of a target point
%
%   INPUTS 
%       pt1  1-by-2 array containing x/y coordinates of first point
%       pt2  1-by-2 array containing x/y coordinates of target point
%       step step size
% 
%   OUTPUTS 
%       npt 1-by-2 array containing x/y coordinates of new point
function npt = createNew(pt1,pt2,step)
    % Calculate normalized directional vector
    v = pt2 - pt1;
    v = v/norm(v);
    
    % Find a new point that is a step size away from given point in the
    % direction of target point
    npt = pt1 + step.*v;
end