%
% TestScript for Assignment 1
%

%% Define a small map
map = false(20);

% Add an obstacle
map (1:5, 6) = true;
map (10, 1:10) = true;
map (16:20, 18) = true;


start_coords = [20, 20];
dest_coords  = [4, 6];

%%
close all;
%[route, numExpanded] = DijkstraGrid (map, start_coords, dest_coords);
% Uncomment following line to run Astar
[route, numExpanded] = AStarGrid (map, start_coords, dest_coords);

%HINT: With default start and destination coordinates defined above, numExpanded for Dijkstras should be 76, numExpanded for Astar should be 23.
