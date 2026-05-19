% This tool outputs a problem instance to be visualized. It is designed to aid in debugging.
% To use, add the problem instance that may contain o/1, i/1, a/3, g/2, and c/1.
%         Then, call clingo viz.lp | viz
% Requirement: the asprilo visualizer (https://asprilo.github.io/visualizer).
% Legend
% - Square - robot
% - Circle - goal
% - White background - inside node
% - Purple background - outside node
% - Green background - reserved node for incoming robot
% - Grey background - obstacle

%% Add the problem instance (map + agents) here
i((16,38)).i((10,39)).i((10,37)).i((9,38)).i((9,37)).i((9,39)).i((15,37)).i((10,40)).i((9,35)).i((10,35)).i((11,38)).i((10,33)).i((10,38)).i((9,33)).i((11,36)).i((14,36)).i((16,34)).i((16,35)).i((16,36)).i((15,34)).i((12,36)).i((14,35)).i((11,37)).i((13,36)).i((13,33)).i((15,38)).i((14,33)).i((13,38)).i((13,35)).i((12,37)).i((13,37)).i((14,37)).i((11,33)).i((11,39)).i((10,34)).i((14,34)).i((15,39)).i((12,35)).i((13,40)).i((13,34)).i((14,39)).i((12,34)).i((13,39)).i((11,34)).i((12,39)).
a(66,(9,33),1).r(66).a(35,(9,37),1).r(35).a(160,(9,38),1).r(160).a(169,(11,36),0).r(169).a(71,(10,38),0).r(71).a(155,(10,34),0).r(155).a(145,(10,35),0).r(145).a(81,(13,39),0).r(81).c((9,38)).g(r(81),(13,33)).g(r(145),(10,33)).g(r(169),(14,33)).g(r(71),(9,33)).g(r(160),(16,35)).g(r(66),(16,36)).g(r(35),(16,34)).
a(66,(9,32),0).m(66,(0,1),1).a(35,(8,37),0).m(35,(1,0),1).a(160,(8,38),0).m(160,(1,0),1).

%% Conversion program
node(C) :- o(C).
node(C) :- i(C).
min((X,Y)) :- X = #min{X' : node((X',_))}, Y = #min{Y' : node((_,Y'))}.

init(object(node,(X,Y)),value(at,(X-X'+1,Y-Y'+1))) :- node((X,Y)), min((X',Y')).
init(object(highway,(X,Y)),value(at,(X-X'+1,Y-Y'+1))) :- o((X,Y)), min((X',Y')).
init(object(robot,R),value(at,(X-X'+1,Y-Y'+1))) :- a(R,(X,Y),0), min((X',Y')).
init(object(shelf,R),value(at,(X-X'+1,Y-Y'+1))) :- g(r(R),(X,Y)), min((X',Y')).
init(object(checkpoint,(X,Y)),value(at,(X-X'+1,Y-Y'+1))) :- c((X,Y)), min((X',Y')).

#show init/2.
