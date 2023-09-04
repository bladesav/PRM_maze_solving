% ======
% ROB521_assignment1.m
% ======
%
% This assignment will introduce you to the idea of motion planning for  
% holonomic robots that can move in any direction and change direction of 
% motion instantaneously.  Although unrealistic, it can work quite well for
% complex large scale planning.  You will generate mazes to plan through 
% and employ the PRM algorithm presented in lecture as well as any 
% variations you can invent in the later sections.
% 
% There are three questions to complete (5 marks each):
%
%    Question 1: implement the PRM algorithm to construct a graph
%    connecting start to finish nodes.
%    Question 2: find the shortest path over the graph by implementing the
%    Dijkstra's or A* algorithm.
%    Question 3: identify sampling, connection or collision checking 
%    strategies that can reduce runtime for mazes.
%
% Fill in the required sections of this script with your code, run it to
% generate the requested plots, then paste the plots into a short report
% that includes a few comments about what you've observed.  Append your
% version of this script to the report.  Hand in the report as a PDF file.
%
% requires: basic Matlab, 
%
% S L Waslander, January 2022
%
clear; close all; clc;

% set random seed for repeatability if desired
% rng(1);

% ==========================
% Maze Generation
% ==========================
%
% The maze function returns a map object with all of the edges in the maze.
% Each row of the map structure draws a single line of the maze.  The
% function returns the lines with coordinates [x1 y1 x2 y2].
% Bottom left corner of maze is [0.5 0.5], 
% Top right corner is [col+0.5 row+0.5]
%

row = 5; % Maze rows
col = 7; % Maze columns
map = maze(row,col); % Creates the maze
start = [0.5, 1.0]; % Start at the bottom left
finish = [col+0.5, row]; % Finish at the top right

h = figure(1);clf; hold on;
plot(start(1), start(2),'go')
plot(finish(1), finish(2),'rx')
show_maze(map,row,col,h); % Draws the maze
drawnow;

% ======================================================
% Question 1: construct a PRM connecting start and finish
% ======================================================
%
% Using 500 samples, construct a PRM graph whose milestones stay at least 
% 0.1 units away from all walls, using the MinDist2Edges function provided for 
% collision detection.  Use a nearest neighbour connection strategy and the 
% CheckCollision function provided for collision checking, and find an 
% appropriate number of connections to ensure a connection from start to 
% finish with high probability.


% variables to store PRM components
nS = 500; % number of samples to test
nn = 10; % number of nearest neighbors to include for each milestone
milestones = [start; finish];  % each row is a point [x y] in feasible space
edges = [];  % each row is should be an edge of the form [x1 y1 x2 y2]

disp("Time to create PRM graph")
tic;
% ------insert your PRM generation code here-------

% Pick random samples
x_val = round((col + 0.5).*rand(nS,1), 2);
y_val = round((row + 0.5).*rand(nS,1), 2);
q = [x_val, y_val];

min_dst = MinDist2Edges(q, map);

q = q(min_dst > 0.1, :);

milestones = [milestones; q];

for i = 1 : size(milestones, 1)
    curr_point = milestones(i, :);
    dist = sqrt((milestones(:, 1) - curr_point(1)).^2 + (milestones(:, 2) - curr_point(2)).^2);
    [dist_sort, idx] = sort(dist);
    num_edges = 0;
    start_idx = 2;
    
    while num_edges < nn && start_idx <= size(milestones, 1)
        neighbour = milestones(idx(start_idx), :);
        start_idx = start_idx + 1;
        [coll, edge] = CheckCollision(curr_point, neighbour, map);
        if coll
            continue
        else
            num_edges = num_edges + 1;
            new_edge = [curr_point, neighbour];
            edges = [edges; new_edge];
        end
    end
end 

milestones = unique(milestones, 'rows', 'stable');

% ------end of your PRM generation code -------
toc;

figure(1);
plot(milestones(:,1),milestones(:,2),'m.');
if (~isempty(edges))
    line(edges(:,1:2:3)', edges(:,2:2:4)','Color','magenta') % line uses [x1 x2 y1 y2]
end
str = sprintf('Q1 - %d X %d Maze PRM', row, col);
title(str);
drawnow;

print -dpng assignment1_q1.png

% =================================================================
% Question 2: Find the shortest path over the PRM graph
% =================================================================
%
% Using an optimal graph search method (Dijkstra's or A*) , find the 
% shortest path across the graph generated.  Please code your own 
% implementation instead of using any built in functions.

disp('Time to find shortest path');
tic;

% Variable to store shortest path
spath = []; % shortest path, stored as a milestone row index sequence
queue = 1:1:size(milestones,1);
tmp_queue = 1:1:size(milestones,1);
dist = Inf(size(milestones, 1), 1);
prev = zeros(size(dist));
dist(1) = 0;

% ------insert your shortest path finding algorithm here-------
while queue
    
    % Take head off queue
    u = queue(1);
    
    if u == 2
        break
    end
    
    % Find neighbours
    v_s_1 = edges(edges(:,3)==milestones(u,1) & edges(:,4)==milestones(u,2),:); 
    v_s_1 = v_s_1(:, 1:2);
    v_s_2 = edges(edges(:,1)==milestones(u,1) & edges(:,2)==milestones(u,2),:); 
    v_s_2 = v_s_2(:, 3:4);
    v_s = [v_s_1; v_s_2]; 
    
    v_s = unique(v_s, 'rows');
    
    % For each neighbour
    for i = 1:size(v_s, 1)
        
        % Convert to index in milestones
        v_curr = v_s(i, :);
        v = tmp_queue(milestones(:,1)==v_curr(1) & milestones(:,2)==v_curr(2));
        
        % Check if in queue
        v_in_q = ismember(v, queue);
       
        if v_in_q
            arc_weight = sqrt((milestones(u, 1) - milestones(v,1))^2 + (milestones(u, 2) - milestones(v, 2))^2);
            alt = dist(u) + arc_weight;
            if alt < dist(v, 1)
                dist(v, 1) = alt;
                prev(v, 1) = u;
            end
        else
            continue
        end
        
    end 
    
    queue = queue(2:end);
    
    % Reorder queue according to cost
    tmp_cost = dist(queue);
    [tmp_cost, sort_idx] = sort(tmp_cost);
    queue = queue(sort_idx);
    
end

% Find shortest path by traversing through parent array
u = 2;
while u
    spath = [spath, u];
    u = prev(u);
end

% ------end of shortest path finding algorithm------- 
toc;    

% plot the shortest path
figure(1);
for i=1:length(spath)-1
    plot(milestones(spath(i:i+1),1),milestones(spath(i:i+1),2), 'go-', 'LineWidth',3);
end
str = sprintf('Q2 - %d X %d Maze Shortest Path', row, col);
title(str);
drawnow;

print -dpng assingment1_q2.png


% ================================================================
% Question 3: find a faster way
% ================================================================
%
% Modify your milestone generation, edge connection, collision detection 
% and/or shortest path methods to reduce runtime.  What is the largest maze 
% for which you can find a shortest path from start to goal in under 20 
% seconds on your computer? (Anything larger than 40x40 will suffice for 
% full marks)

row = 70;
col = 70;
map = maze(row,col);
start = [0.5, 1.0];
finish = [col+0.5, row];
milestones = [start];  % each row is a point [x y] in feasible space
edges = [];  % each row is should be an edge of the form [x1 y1 x2 y2]

h = figure(2);clf; hold on;
plot(start(1), start(2),'go')
plot(finish(1), finish(2),'rx')
show_maze(map,row,col,h); % Draws the maze
drawnow;

fprintf("Attempting large %d X %d maze... \n", row, col);
tic;     

% ------insert your optimized algorithm here------

milestones = [start]; 
edges = [];

% Get points along each grid point

for i=1:col
    for j =1:row
        milestones = [milestones; i, j];
    end
end

milestones = [milestones; finish]; 
milestones = unique(milestones, 'rows', 'stable');

% Get PRM edges 

tree = KDTreeSearcher(milestones);
neighbours = knnsearch(tree, milestones, 'K', 3);
neighbours = neighbours(:, 2:end);

for i = 1:size(neighbours, 2)
    
    inds = neighbours(:, i);
    raw_edges = [milestones, milestones(inds, :)];
    
    for j = 1:size(raw_edges, 1)
        [coll, edge] = CheckCollision(raw_edges(j, 1:2), raw_edges(j, 3:4), map);
        if ~coll
            edges = [edges; raw_edges(j, :)];
        end
    end
    
end

% Variable to store shortest path

spath = []; % shortest path, stored as a milestone row index sequence
queue = 1:1:size(milestones,1);
tmp_queue = 1:1:size(milestones,1);
dist = Inf(size(milestones, 1), 1);
heur = Inf(size(milestones, 1), 1);
prev = zeros(size(dist));
dist(1) = 0;

arcweights = sqrt((edges(:, 2) - edges(:, 4)).^2 + (edges(:, 1) - edges(:, 3)).^2);
edges = [edges, arcweights];

while queue
    
    % Take head off queue
    u = queue(1);
    
    if u == size(milestones, 1)
        break
    end
    
    % Find neighbours
    v_s_1 = edges(edges(:,3)==milestones(u,1) & edges(:,4)==milestones(u,2),:); 
    v_s_2 = edges(edges(:,1)==milestones(u,1) & edges(:,2)==milestones(u,2),:); 
    v_s = [v_s_1(:, 1:2); v_s_2(:, 3:4)];
    weights = [v_s_1(:, 5); v_s_2(:, 5)];
    [v_s,ivs,none] = unique(v_s, 'rows');
    weights = weights(ivs, 1);
    
    % For each neighbour
    for i = 1:size(v_s, 1)
        
        % Convert to index in milestones
        v_curr = v_s(i, :);
        v = tmp_queue(milestones(:,1)==v_curr(1) & milestones(:,2)==v_curr(2));
        
        % Check if in queue
        v_in_q = ismember(v, queue);
       
        if v_in_q
            arc_weight = weights(i);
            alt = dist(u) + arc_weight;
            if alt < dist(v, 1)
                dist(v, 1) = alt;
                prev(v, 1) = u;
                
                % Get heuristic 
                fin_dist = (finish(1) - milestones(v,1))^2 + (finish(2) - milestones(v, 2))^2;
                heur(v, 1) = alt + fin_dist;
            end
        else
            continue
        end
        
    end 
    
    queue = queue(2:end);
    
    % Reorder queue according to cost
    tmp_cost = heur(queue);
    [tmp_cost, sort_idx] = sort(tmp_cost);
    queue = queue(sort_idx);
    
end

% Find shortest path by traversing through parent array
u = size(milestones, 1);
while u
    spath = [spath, u];
    u = prev(u);
end

% ------end of your optimized algorithm-------
dt = toc

figure(2); hold on;
plot(milestones(:,1),milestones(:,2),'m.');
if (~isempty(edges))
    line(edges(:,1:2:3)', edges(:,2:2:4)','Color','magenta')
end
if (~isempty(spath))
    for i=1:length(spath)-1
        plot(milestones(spath(i:i+1),1),milestones(spath(i:i+1),2), 'go-', 'LineWidth',3);
    end
end
str = sprintf('Q3 - %d X %d Maze solved in %f seconds', row, col, dt);
title(str);
drawnow;
print -dpng assignment1_q3.png
