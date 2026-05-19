robot(R) :- init(object(robot,R),_).
node(C) :- init(object(node,_), value(at,C)).
at(R,C,0) :- init(object(robot,R),value(at,C)).

direction((X,Y)) :- X=-1..1, Y=-1..1, |X+Y|=1.
% nextto((X,Y),(X+DX,Y+DY),(DX,DY)) :- node((X,Y)), node((X+DX,Y+DY)), direction((DX,DY)).  % This line runs slower than the line below
nextto((X,Y),(X',Y'),(DX,DY)) :- direction((DX,DY)), node((X,Y)), node((X',Y')), (X,Y)=(X'-DX,Y'-DY), (X',Y')=(X+DX,Y+DY).

#program plan(t).
#external query(t).
{ move(R,D,t) : direction(D) } 1 :- robot(R), t > 0.
% { move(R,D,t) : direction(D) } :- robot(R), t > 0.
% :- move(R,D,t), move(R,D',t), D != D'.

:- at(R,C,0), node(C), not move(R,_,1), query(t). % force robots outside to move in

at(R,C',t) :- at(R,C,t-1), move(R,D,t), nextto(C,C',D).
:- move(R,D,t), at(R,C,t-1), not nextto(C,_,D).
at(R,C,t) :- at(R,C,t-1), not move(R,_,t).

moveto(C,C',t):- at(R,C,t-1), move(R,D,t), nextto(C,C',D).
:- moveto(C,C',t), moveto(C',C,t).

:- { at(R,C,t) : robot(R) } > 1, node(C).

%% Goal checking
%% version 1
% :- not at(R,C,t), goal(robot(R),C), query(t).

%% version 2 + having c/1
g(t) :- at(R,C,t) : goal(robot(R),C).
:- not g(t), query(t).
:- g(t), at(R,C,t), c(C), not goal(robot(R),C), query(t).


#show move/3.

#script (python)
import clingo
from clingo.symbol import Number

def main(prg):
  prg.ground([("base", [])])

  for t in range(0,100):
    print("t = {}".format(t))
    prg.ground([("plan", [Number(t)])])

    atom = clingo.parse_term("query(" + str(t) + ")")
    prg.assign_external(atom, True)

    if str(prg.solve(on_model=print)) == "SAT":
      break

    prg.release_external(atom)
#end.
