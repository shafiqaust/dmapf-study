time(1..horizon).

node(N,C) :- init(object(node,N),value(at,C)).
at(R,N,0) :- init(object(robot,R),value(at,C)), node(N,C).
goal(R,N) :- init(object(order,R),value(line,(P,_))), init(object(product,P),value(on,(S,_))), init(object(shelf,S),value(at,C)), init(object(node,N),value(at,C)).
direction((X,Y)) :- X=-1..1, Y=-1..1, |X+Y|=1.
nextto(N1,N2,(DX,DY)) :- node(N1,(X,Y)), node(N2,(X+DX,Y+DY)), direction((DX,DY)).

{ move(R,D,T) : direction(D) } 1 :- at(R,_,T-1), time(T).

% - move/3 ----------------------------------------------------------------------
at(R,N2,T) :- move(R,D,T), at(R,N1,T-1), nextto(N1,N2,D), time(T).
           :- move(R,D,T), at(R,N,T-1), not nextto(N,_,D), time(T).

% - inertia ---------------------------------------------------------------------
at(R,N,T) :- at(R,N,T-1), not move(R,_,T), time(T).

% - edge collision --------------------------------------------------------------
moveto(N1,N2,T) :- move(R,D,T), at(R,N1,T-1), nextto(N1,N2,D), time(T).
:- moveto(N1,N2,T), moveto(N2,N1,T), time(T).

% - vertex collision ------------------------------------------------------------
:- { at(R,N,T) : at(R,_,_) } > 1, node(N,_), time(T).

% goal_reached(T) :- time(T), at(R,N,T) : goal(R,N).
% :- time(horizon), not goal_reached(horizon).

:- not at(R,N,horizon), goal(R,N).

#show move/3.
