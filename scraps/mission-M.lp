#external show_area_counts.
#external show_borders.
#external show_cross_border_counts.
#external show_dests.
#external show_directions.
#external show_roots.

%#external show_num_neighbors.
%#external show_num_neighbor_areas.
%#external show_nodes.
%#external show_areas.
%#external show_links.

direction(0,(-1,0)).  % Direction ID must start from 0
direction(1,(1,0)).
direction(2,(0,-1)).
direction(3,(0,1)).
nextto(N1,N2,D) :- node(N1,(X,Y),A), node(N2,(X+DX,Y+DY),A), direction(D,(DX,DY)).

#show.
#show (A,C) : node(_,_,A), C = #count { N : node(N,_,A) }, show_area_counts.
#show (S,N) : link(S,_,_,N,_), show_borders.
#show (A1,A2,S,C) : link(S,A1,A2,_,_), C = #count { N : link(S,A1,A2,N,_) }, show_cross_border_counts.
#show (R,N) : dest(R,N), show_dests.
#show (R,0) : at(R,_,0), not dest(R,_), show_dests. % for robots that do not have a goal
#show (D,(DX,DY)) : direction(D,(DX,DY)), show_directions.
#show (R,N) : root(R,N), show_roots.

%#show N : N = #count { S : link(S,_,_,_,_) }, show_num_neighbors.
%#show N : N = #count { A1,A2 : link(_,A1,A2,_,_) }, show_num_neighbor_areas.
%#show (N,C) : node(N,C), show_nodes.
%#show (N,C) : nnode(N,C), show_nodes.
%#show (A,N) : area(A,N), show_areas.
%#show (S2,A1,A2,N1,N2) : link(S2,A1,A2,N1,N2), show_links.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#program migrate.
#external migrate(N,A1,A2) : node(N,_,A1), link(_,A1,A2,_,_).
#external migrate(N1,A1,A2) : nextto(N1,N2,_), not node(N1,_,_), node(N2,_,A1), link(_,A1,A2,_,_).
#external limit(0..L,A1,A2) : L = #count { N1,N2 : link(_,A1,A2,N1,N2) }, link(_,A1,A2,_,_).
#external show_migration.

% { asgn(R,N1,N2) : link(_,A1,A2,N1,N2) } :- migrate(R,A1,A2).
% :- asgn(R,N1,_), asgn(R,N2,_), N1 != N2.
% :- asgn(R1,N,_), asgn(R2,N,_), R1 != R2.
% :- asgn(R1,N1,N), asgn(R2,N2,N), R1 != R2, N1 != N2.  % for non-uniform maps

0 { asgn(R,N1,N2) : link(_,A1,A2,N1,N2) } 1 :- migrate(R,A1,A2).
:- asgn(R1,N,_), asgn(R2,N,_), R1 != R2.
:- asgn(R1,_,N), asgn(R2,_,N), R1 != R2.

:- C = #count { R : asgn(R,_,_), migrate(R,A1,A2) }, C < L, limit(L,A1,A2).

#show (S,R,N1,N2) : migrate(R,A1,A2), asgn(R,N1,N2), link(S,A1,A2,N1,N2), show_migration.

% migrate/4 and limit/4 are externals added from outside
% migrate(I,R,A1,A2) 
% #program migrate.
% { goal(R,N1,N2) : link(_,A1,A2,N1,N2) } :- migrate(R,A1,A2).
% :- goal(R,N1,_), goal(R,N2,_), N1 != N2.
% :- goal(R1,N,_), goal(R2,N,_), R1 != R2.
% :- goal(R1,N1,N), goal(R2,N2,N), R1 != R2, N1 != N2.  % for non-uniform maps
% goal(R,N) :- goal(R,N,_).

% :- C = #count { R : goal(R,_), migrate(R,A1,A2) }, C < L, limit(L,A1,A2).

% #show (S,R,N1,N2) : migrate(R,A1,A2), goal(R,N1,N2), link(S,A1,A2,N1,N2), show_migration.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%#program migrate(i).
%{ goal(i,R,S,N1,N2) : link(S,A1,A2,N1,N2) } :- robot(i,R).
%:- goal(i,R,_,N1,_), goal(i,R,_,N2,_), N1 != N2.
%:- goal(i,R1,_,N,_), goal(i,R2,_,N,_), R1 != R2.

%:- N = #count { R : goal(i,R,_,_,_) }, N < L, limit(L).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%#program assign_border(i).
%#external assign_border_query(i).
%{ assign(i,R,S,N1,N2) : link(S,A1,A2,N1,N2) } :- migrating(i,R,A1,A2), robot(i,R).
%:- assign(i,R,_,N1,_), assign(i,R,_,N2,_), N1 != N2.
%:- assign(i,R1,_,N,_), assign(i,R2,_,N,_), R1 != R2.

%:- N = #count { R : assign(i,R,_,_,_) }, N < L, limit(L).
% #external limit/1.

%#show (R,S,N1,N2) : assign(i,R,S,N1,N2), assign_border_query(i).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% We don't need this program
% #program init(i).
% at(R,N,i,0) :- at(R,N,i-1,T), T = #max { U : at(R,_,i-1,U) }, not migrate(R,i-1).

% % Is the code below necessary?
% 1 { subgoal(R1,N2,i) : node(N2,_), N2 != N1 } 1 :- at(R1,N1,i,0), at(R2,N1,i,1), not migrate(R2,i).
% goal(R,N,i) :- subgoal(R,N,i).
% :- goal(R1,N,i), goal(R2,N,i), R1 != R2.
% % - select the closest sub goal
% :- at(R,N1,i,0), subgoal(R,N2,i), node(N1,(X1,Y1)), node(N2,(X2,Y2)), node(N3,(X3,Y3)), not goal(_,N3,i), N1 != N3, |X3-X1|+|Y3-Y1| < |X2-X1|+|Y2-Y1|.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#program plan.
#external at(N,N,0) : node(N,_,_).                    % for existed robots
#external at(N,N,0) : nextto(N,_,_), not node(N,_,_). % for incoming robots
#external goal(N1,N2) : node(N1,_,_), node(N2,_,_).
#external goal(N1,N2) : nextto(N1,_,_), not node(N1,_,_), node(N2,_,_).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#program plan(t).
#external query(t).

%% Generate moves for existing robots
0 { move(R,D,t) : nextto(N,_,D) } 1 :- at(R,N,t-1).
% { move(R,D,t) : direction(D,_) } :- at(R,N,t-1).
% :- move(R,D1,t), move(R,D2,t), D1 != D2.
% :- move(R,D,t), at(R,N,t-1), not nextto(N,_,D).

% Force incoming robots to move in
:- at(R,N,0), not node(N,_,_), not move(R,_,1), query(t).

%% Effects of moves
at(R,N2,t) :- move(R,D,t), at(R,N1,t-1), nextto(N1,N2,D).

%% Inertia
at(R,N,t) :- at(R,N,t-1), not move(R,_,t).

%% Prevent edge collision
moveto(N1,N2,t) :- at(R,N1,t-1), move(R,D,t), nextto(N1,N2,D).
:- moveto(N1,N2,t), moveto(N2,N1,t).

%% Prevent vertex collision
:- { at(_,N,t) } > 1, node(N,_,_).

%% Goal checking
goal_reached(t) :- at(R,N,t) : goal(R,N).
:- query(t), not goal_reached(t).

#show (R,D,T,N) : move(R,D,T), at(R,N,T), T <= t, query(t).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%#program plan(i,t).
%#external query(i,t).

%{ move(R,D,i,t) : direction(D) } 1 :- at(R,_,i,t-1).

% - move/3 ----------------------------------------------------------------------
%at(R,N2,i,t) :- move(R,D,i,t), at(R,N1,i,t-1), nextto(N1,N2,D).
%             :- move(R,D,i,t), at(R,N,i,t-1), not nextto(N,_,D).

% - inertia ---------------------------------------------------------------------
%at(R,N,i,t) :- at(R,N,i,t-1), not move(R,_,i,t).

% - edge collision --------------------------------------------------------------
%moveto(N1,N2,i,t) :- move(R,D,i,t), at(R,N1,i,t-1), nextto(N1,N2,D).
%:- moveto(N1,N2,i,t), moveto(N2,N1,i,t).

% - vertex collision ------------------------------------------------------------
%:- { at(R,N,i,t) : at(R,_,i,_) } > 1, node(N,_).

%goal_reached(i,t) :- at(R,N,i,t) : goal(R,N,i).
%:- query(i,t), not goal_reached(i,t).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% #program show_plan.
% #external show_plan.
% #show move(R,D,I,T) : move(R,D,I,T), show_plan.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% #program answer.
% #external show_init.
% #external show_occurs.

% horizon(0,H) :- move(_,_,0,_), H = #max { T : move(_,_,0,T) }.
% horizon(0,0) :- not move(_,_,0,_).
% horizon(I,H1+H2) :- move(_,_,I,_), horizon(I-1,H1), H2 = #max { T : move(_,_,I,T) }.
% horizon(I,H1) :- not move(_,_,I,_), I = 1..N, N = #max { M : move(_,_,M,_) }, horizon(I-1,H1).

% #show init(object(node,N),value(at,C)) : init(object(node,N),value(at,C)), show_init.
% #show init(object(robot,R),value(at,C)) : init(object(robot,R),value(at,C)), show_init.
% #show init(object(shelf,S),value(at,C)) : init(object(shelf,S),value(at,C)), show_init.
% #show occurs(object(robot,R),action(move,D),T) : move(R,D,0,T), show_occurs.
% #show occurs(object(robot,R),action(move,D),H+T) : move(R,D,I,T), horizon(I-1,H), show_occurs.
