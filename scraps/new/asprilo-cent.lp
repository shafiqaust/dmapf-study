time(1..horizon).

node(C)   :- init(object(node,_), value(at,C)).
at(R,C,0) :- init(object(robot,R),value(at,C)).

direction((X,Y)) :- X=-1..1, Y=-1..1, |X+Y|=1.
nextto((X,Y),(X+DX,Y+DY),(DX,DY)) :- node((X,Y)), node((X+DX,Y+DY)), direction((DX,DY)).

{ move(R,D,T) : direction(D) } 1 :- at(R,_,T-1), time(T).

% - move/3 ----------------------------------------------------------------------
at(R,C2,T) :- move(R,D,T), at(R,C1,T-1), nextto(C1,C2,D), time(T).
           :- move(R,D,T), at(R,C,T-1), not nextto(C,_,D), time(T).

% - inertia ---------------------------------------------------------------------
at(R,C,T) :- at(R,C,T-1), not move(R,_,T), time(T).

% - edge collision --------------------------------------------------------------
moveto(C1,C2,T) :- move(R,D,T), at(R,C1,T-1), nextto(C1,C2,D), time(T).
:- moveto(C1,C2,T), moveto(C2,C1,T), time(T).

% - vertex collision ------------------------------------------------------------
:- { at(R,C,T) : at(R,_,_) } > 1, node(C), time(T).

% goal_reached(T) :- time(T), at(R,N,T) : goal(R,N).
% :- time(horizon), not goal_reached(horizon).

:- not at(R,C,horizon), goal(robot(R),C).

#show move/3.
