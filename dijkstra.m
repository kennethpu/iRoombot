function [path,goal,gfound] = dijkstra(V,E,start,goals)
% DIJKSTRA: Takes as input a graph represented by a set of nodes V and
% edges E, a start position, and a list of goal positions, then returns 
% the shortest path between start node and the closest goal node. Uses 
% Dijkstra's algorithm
% 
%   [PATH,GOAL,GFOUND] = DIJKSTRA(V,E,START,GOALS) returns
%   the shortest path between a start and goal node given a set of nodes V 
%   and edges E
% 
%   INPUTS
%       V      Set of nodes in graph
%       E      Set of edges in graph
%       start  1-by-2 array containing x/y coordinates of start node
%       goals  N-by-2 array containing x/y coordinates of goal nodes
% 
%   OUTPUTS
%       path   N-by-2 array containing a series of points representing the
%              shortest path connecting initial and closest goal points
%       goal   1-by-2 array containing x/y coordinates of closest goal node
%       gfound An integer denoting the number of goals found
%
%   Cornell University
%   MAE 4180/5180: Autonomous Mobile Robots
%   Final Competition
%   Pu, Kenneth (kp295) 

%% ============================================================================
% INITIALIZE VARIABLES
%==============================================================================
V_cost = ones(size(V,1),1).*9999;   % Array of node costs (initially HI)
V_prev = zeros(size(V,1),1);        % List of node parent indices (initially 0)

% Integer to keep track of the number of goals found
gfound = 0;

% Set cost (distance from start) of start to 0
V_cost = setCost(start,V,V_cost,0);

% All nodes are initially in Q
Q = V;  % List of nodes to be optimized

%% ============================================================================
% RUN DJIKSTRAS
%==============================================================================
% Loop while there are still nodes to be optimized
while size(Q,1)
    % Get node u in Q with smallest cost
    qidx = minNode(Q,V,V_cost);
    u = Q(qidx,:);
    
    % Remove node u from Q
    Q(qidx,:) = [];
    
    % Break out of loop if u's cost is MAX. No more nodes are reachable
    % from current node
    if (getCost(u,V,V_cost) == 9999)
        break;
    end
    
    % Get list of neighbor node indices
    n_ids = getNeighbors(u,V,E);
    
    % Iterate through all neighbors of u
    for i=1:size(n_ids,1)
        % Get neighbor node n
        n = V(n_ids(i),:);
        
        % Calculate alternative cost of n as cost of u + distance between u
        % and n
        u_cost = getCost(u,V,V_cost);
        n_dist = pdist([u;n],'euclidean');
        alt = u_cost + n_dist;
        
        % If calculated alternative cost is cheaper than stored cost for n,
        % replace stored cost for n and set u as parent node for n
        if alt < getCost(n,V,V_cost)
            V_cost = setCost(n,V,V_cost,alt);
            V_prev = setPrev(n,V,V_prev,u);
        end
    end
    
    if ismember(u,goals,'rows')
        gfound = gfound+1;
        if gfound == size(goals,1)
            break;
        end
    end
end

%% ============================================================================
% FIND CLOSEST GOAL
%==============================================================================
% Initialize array to hold goal nodes and their corresponding costs
goal_costs = zeros(size(goals,1),size(goals,2)+1);
for i=1:size(goals,1)
    % Get costs for each goal and store in goal_costs
    goal_costs(i,:) = [goals(i,:),getCost(goals(i,:),V,V_cost)];
end
% Sort goal_costs by cost
goal_costs = sortrows(goal_costs,3);
% Return the cheapest goal
goal = goal_costs(1,1:2);

%% ============================================================================
% RECONSTRUCT MINIMUM PATH TO CLOSEST GOAL
%==============================================================================
% Start from goal point
u = goal;
% plot(u(1),u(2),'go','LineWidth',2);

% Add goal point to path
path = u;

% Step backward from goal point adding parent nodes until parent node index
% is no longer valid
while getPrev(u,V,V_prev)
    prev_idx = getPrev(u,V,V_prev);
    u = V(prev_idx,:);
%     plot(u(1),u(2),'go','LineWidth',2);
%     plot([u(1),path(1,1)],[u(2),path(1,2)],'g','LineWidth',2);
    path = [u;path];
end
end

%% ============================================================================
% minNode
%==============================================================================
%   Helper function to get the index of the node with smallest cost in a 
%   list of nodes
%
%   INPUTS 
%       Q      list of nodes to search over
%       V      list of nodes in graph
%       V_cost list of costs associated with V
% 
%   OUTPUTS 
%       nidx   index of node in Q with smallest cost
function nidx = minNode(Q,V,V_cost)
    % Initialize variables
    nidx = 1;
    min_cost = 9999;
    % Iterate through all nodes in Q
    for i=1:size(Q,1)
        node = Q(i,:);
        cost = getCost(node,V,V_cost);
        % If node cost is less than minimum, save minimum cost and node index
        if (cost<min_cost)
            nidx = i;
            min_cost = cost;
        end
    end
end

%% ============================================================================
% getCost
%==============================================================================
%   Helper function to get the cost of a node
%
%   INPUTS 
%       n      a given node
%       V      list of nodes in graph
%       V_cost list of costs associated with V
% 
%   OUTPUTS 
%       cost   cost associated with a node
function cost = getCost(n,V,V_cost)
    % Find index corresponding to n in V
    [~,idx] = ismember(n,V,'rows');
    % Look up corresponding cost in V_cost
    cost = V_cost(idx);
end

%% ============================================================================
% setCost
%==============================================================================
%   Helper function to set the cost of a node
%
%   INPUTS 
%       n      a given node
%       V      list of nodes in graph
%       V_cost list of costs associated with V
%       cost   cost associated with a node
%
%   OUTPUTS 
%       ret    updated list of costs associated with V
function ret = setCost(n,V,V_cost,cost)
    % Find index corresponding to n in V
    [~,idx] = ismember(n,V,'rows');
    % Set corresponding cost in V_cost to our target cost
    V_cost(idx) = cost;
    % Return updated list so our changes are not lost
    ret = V_cost;
end

%% ============================================================================
% getPrev
%==============================================================================
%   Helper function to get the index of parent node of a given node
%
%   INPUTS 
%       n        a given node
%       V        list of nodes in graph
%       V_prev   list of parents associated with V
% 
%   OUTPUTS 
%       prev_idx index of parent node of a given node
function prev_idx = getPrev(n,V,V_prev)
    % Find index corresponding to n in V
    [~,idx] = ismember(n,V,'rows');
    % Look up corresponding parent index in V_prev
    prev_idx = V_prev(idx);
end

%% ============================================================================
% setPrev
%==============================================================================
%   Helper function to set the parent node of a given node
%
%   INPUTS 
%       n      a given node
%       V      list of nodes in graph
%       V_prev list of parents associated with V
%       prev   parent node of a given node
%
%   OUTPUTS 
%       ret    updated list of parents associated with V
function ret = setPrev(n,V,V_prev,prev)
    % Find index corresponding to n in V
    [~,idx] = ismember(n,V,'rows');
    % Find index corresponding to prev in V
    [~,prev_idx] = ismember(prev,V,'rows');
    % Set corresponding prev_idx in V_prev to our target prev_idx
    V_prev(idx) = prev_idx;
    % Return updated list so our changes are not lost
    ret = V_prev;
end

%% ============================================================================
% getNeighbors
%==============================================================================
%   Helper function to get the indices of all neighboring nodes of a given
%   node
%
%   INPUTS 
%       n      a given node
%       V      list of nodes in graph
%       E      list of edges in graph
% 
%   OUTPUTS 
%       ids    N-by-1 array of indices of nodes in V neighboring node n
function ids = getNeighbors(n,V,E)
    % Initialize empty list
    ids = [];
    
    % Iterate through all edges
    for e=1:size(E,1)
        % Get endpoints of edges
        v1 = E(e,1:2);
        v2 = E(e,3:4);
        % If target node corresponds to first endpoint, save second
        % endpoint as a neighbor
        if (all(v1==n))
            [~,idx] = ismember(v2,V,'rows');
            ids = [ids;idx];
        % If target node corresponds to second endpoint, save first
        % endpoint as a neighbor
        elseif (all(v2==n))
            [~,idx] = ismember(v1,V,'rows');
            ids = [ids;idx];
        end 
    end
end
