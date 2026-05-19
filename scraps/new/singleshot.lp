% = Combined Encoding for non-anonymous MAPF with possible locations  ==============================

% Move actions
% #include "./../asprilo/m/action-M.lp".
robot(R)                     :- init(object(robot,R),          _).
% shelf(S)                     :- init(object(shelf,S),          _).
% station(P)                   :- init(object(pickingStation,P), _).
% product(A)                   :- init(object(product,A),        _).

    isRobot(robot(R)) :- robot(R).
%     isShelf(shelf(S)) :- shelf(S).
% isStation(station(T)) :- station(T).
% isProduct(product(A)) :- product(A).
%     isOrder(order(O)) :- order(O).

%   order(      O            ) :- init(object(order,O),          _).
% ordered(order(O),product(A)) :- init(object(order,O),          value(line,(A,_))).      % IGNORING QUANTITIES
%  target(order(O),station(P)) :- init(object(order,O),          value(pickingStation,P)).

% shelved(shelf(S),product(A)) :- init(object(product,A),        value(on,(S,_))).        % IGNORING QUANTITIES

position(           (X,Y))   :- init(object(node,_),           value(at,(X,Y))).
% position(           C)   :- init(object(node,_),           value(at,C)).

% position(station(P),(X,Y))   :- init(object(pickingStation,P), value(at,(X,Y))).

position(  robot(R),(X,Y),0) :- init(object(robot,R),          value(at,(X,Y))).
% position(  robot(R),C,0) :- init(object(robot,R),          value(at,C)).
% position(  shelf(S),(X,Y),0) :- init(object(shelf,S),          value(at,(X,Y))).

% highway(            C    )   :- init(object(highway,_),        value(at,C)).

%#const horizon=1.
time(1..horizon).

direction((X,Y)) :- X=-1..1, Y=-1..1, |X+Y|=1.
nextto((X,Y),(DX,DY),(X',Y')) :- direction((DX,DY)), position((X,Y)), position((X',Y')),
                                 (X,Y)=(X'-DX,Y'-DY), (X',Y')=(X+DX,Y+DY).

{ move(R,D,T) : direction(D) } 1 :- isRobot(R), time(T).

% - move/3 ----------------------------------------------------------------------
position(R,C,T) :- move(R,D,T), position(R,C',T-1),     nextto(C',D,C).
                :- move(R,D,T), position(R,C ,T-1), not nextto(C ,D,_).

% - inertia ---------------------------------------------------------------------
position(R,C,T) :- position(R,C,T-1), not move(R,_,T), isRobot(R), time(T).

% - edge collision --------------------------------------------------------------
moveto(C',C,T) :- nextto(C',D,C), position(R,C',T-1), move(R,D,T).
 :- moveto(C',C,T), moveto(C,C',T), C < C'.

% - vertex collision ------------------------------------------------------------
 :- { position(R,C,T) : isRobot(R) }  > 1, position(C), time(T).

% - auxiliaries -----------------------------------------------------------------
 :- { position(R,C,T) } != 1, isRobot(R), time(T).    % REDUNDANT but PERFORMANT?


% Enforce assigned goal locations
% #include "./assigned-goal.lp".
:- not position(R,C,horizon), goal(R,C).


% Enforce possible locations
% #include "./possible_location.lp".
% #defined poss_loc/3.
% :- not poss_loc(R,C,T), position(R,C,T), isRobot(R), poss_loc(_,_,_).


% Output conversion
% #include "./../asprilo/m/output-M.lp".
% #show.
% #show init/2.
% #show occurs(object(robot,R), action(move,D),T) : move(robot(R),D,T).
#show move/3.
