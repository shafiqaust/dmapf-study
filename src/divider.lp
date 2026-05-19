#external show_nodes.

direction((-1,0)).
direction((1,0)).
direction((0,-1)).
direction((0,1)).
node(C) :- init(object(node,_),value(at,C)).

%% Only use the line below in full mode
% goal(robot(R),C) :- init(object(order,R),value(line,(P,_))), init(object(product,P),value(on,(S,_))), init(object(shelf,S),value(at,C)).

#show.
#show C : node(C), show_nodes.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#program divide(s, x_min, x_max, y_min, y_max).
#external show_exist(s).
coor(s,(X,Y)) :- node((X,Y)), X >= x_min, X < x_max, Y >= y_min, Y < y_max.

#show C : coor(s,C), show_exist(s).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#program part(s).
#external show_areas(s).
#external show_edges(s).
#external show_robots(s).

% border(s,(X,Y)) :- node(s,(X,Y)), not node(s,(X',Y')), direction((DX,DY)), (X,Y)=(X'-DX,Y'-DY), (X',Y')=(X+DX,Y+DY).
border(s,(X,Y)) :- node(s,(X,Y)), node(S,(X',Y')), s != S, direction((DX,DY)), (X,Y)=(X'-DX,Y'-DY), (X',Y')=(X+DX,Y+DY).
edge(s,(X,Y),(X',Y')) :- node(s,(X,Y)), node(s,(X',Y')), direction((DX,DY)), (X,Y)=(X'-DX,Y'-DY), (X',Y')=(X+DX,Y+DY).
edge(s,C,C) :- node(s,C).
robot(s,R,C) :- init(object(robot,R),value(at,C)), node(s,C).

#show (C,C') : edge(s,C,C'), show_edges(s).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#program link.
connect(S,S',(X,Y),(X',Y')) :- border(S,(X,Y)), border(S',(X',Y')), S != S', direction((DX,DY)), (X,Y)=(X'-DX,Y'-DY), (X',Y')=(X+DX,Y+DY).

%% Allow Border Contention
link(S,S',C,C') :- connect(S,S',C,C').

%% Disallow Border Contention
% Generate links without edge collision
% 1 {link(S1,S2,N1,N2); link(S2,S1,N2,N1)} 1 :- connect(S1,S2,N1,N2).

% Balance the number of outgoing and incoming links
% :- connect(S1,S2,_,_), L1 = #count{N : link(S1,S2,_,N)}, L2 = #count{N : link(S2,S1,_,N)}, |L1 - L2| > 1.

% No vertex collision
% :- link(_,_,N1,N3), link(_,_,N2,N3), N1 != N2.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% After adding area/3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#program result.
#external show_connections(A) : area(_,A,_).
#external show_corners(A,A') : area(S,A,C), area(S',A',C'), link(S,S',C,C').
#external show_nodes(A) : area(_,A,_).
#external show_nexttos(A) : area(_,A,_).
#external show_links(A,A') : area(S,A,C), area(S',A',C'), link(S,S',C,C').
#external show_abstract_links.

#show (A,N,(MinC,MinR),(MaxC,MaxR)) : area(S,A,_), N = #count{C : area(S,A,C)},
                                      MinC = #min{C : area(S,A,(C,_))}, MinR = #min{R : area(S,A,(_,R))},
                                      MaxC = #max{C : area(S,A,(C,_))}, MaxR = #max{R : area(S,A,(_,R))}, show_areas(S).
#show (A',S') : area(S,A,C), area(S',A',C'), link(S,S',C,C'), show_connections(A).
#show C : area(S,A,C), area(S',A',C'), link(S,S',C,C'), link(S,_,C,C''), C' != C'', show_corners(A,A').
#show l(C,C') : area(S,A,C), area(S',A',C'), link(S,S',C,C'), show_links(A,A').
#show (R,A,A',C,C') : robot(S,R,C), area(S,A,C), goal(robot(R),C'), area(_,A',C'), show_robots(S).
#show i(C) : area(_,A,C), show_nodes(A).
#show d(D) : direction(D), show_nexttos(_).
#show x((X,Y),(X',Y'),(DX,DY)) : area(S,A,(X,Y)), area(S,A,(X',Y')), direction((DX,DY)), (X,Y)=(X'-DX,Y'-DY), (X',Y')=(X+DX,Y+DY), show_nexttos(A).
#show l(A,A') : area(S,A,C), area(S',A',C'), link(S,S',C,C'), show_abstract_links.
