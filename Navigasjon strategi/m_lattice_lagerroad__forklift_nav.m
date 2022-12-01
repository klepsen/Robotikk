% Simulates the Lattice planner algoritm 
%if call fails, try "load road_lagerhus" in cmd window before running function):

% Load a predefined occupancy grid of a house
init = [20 20 0];
goal = [80 50 pi];
load road_lagerhus
about road_lagerhus

lp = Lattice(road_lagerhus,'grid',15,'root',[50 50 0]);   % Construct the navigation object
lp.plan('cost',[1 100 100])                 % Plan lattice, increase cost of turning and inflate obstacles
figure;                         % New figure
lp.plot();                      % Plot lattice
disp('Original path: ')
P = lp.query(init,goal);            % Compute path between init and goal, and report
%cost 
lp.plot();                      % Overlay path on vertices