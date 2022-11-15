function out=m_lattice_road_nav(init,goal)
% Simulates the Lattice planner algoritm
%% Example of function call 
%% (if call fails, try "load road" in cmd window before running function):
disp('lp=m_lattice_road_nav([30 45 0], [50 20 0]')
%% Initializing
% Load a predefined occupancy grid of a house
init = [20 20 0];
goal = [95 65 0];
load road_lagerhus
about road_lagerhus
%% Navigation using lattice planner
lp = Lattice(road_lagerhus,'grid',15,'root',[50 50 0]);   % Construct the navigation object
lp.plan('cost',[1 100 100], 'inflate', 5)                 % Plan lattice, increase cost of turning and inflate obstacles
figure;                         % New figure
lp.plot();                      % Plot lattice
disp('Original path: ')
lp.query(init,goal);            % Compute path between init and goal, and report
%cost 
lp.plot();                      % Overlay path on vertices