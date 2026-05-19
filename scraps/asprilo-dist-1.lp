node(N,C) :- init(object(node,N),value(at,C)).
at(R,N,0) :- init(object(robot,R),value(at,C)), node(N,C).
goal(R,N) :- init(object(order,R),value(line,(P,_))), init(object(product,P),value(on,(S,_))), init(object(shelf,S),value(at,C)), init(object(node,N),value(at,C)).
direction((X,Y)) :- X=-1..1, Y=-1..1, |X+Y|=1.
nextto(N1,N2,(DX,DY)) :- node(N1,(X,Y)), node(N2,(X+DX,Y+DY)), direction((DX,DY)).

#program plan(t).
#external query(t).
{ move(R,D,t) : direction(D) } 1 :- at(R,_,t-1).

% - move/3 ----------------------------------------------------------------------
at(R,N2,t) :- move(R,D,t), at(R,N1,t-1), nextto(N1,N2,D).
           :- move(R,D,t), at(R,N,t-1), not nextto(N,_,D).

% - inertia ---------------------------------------------------------------------
at(R,N,t) :- at(R,N,t-1), not move(R,_,t).

% - edge collision --------------------------------------------------------------
moveto(N1,N2,t) :- move(R,D,t), at(R,N1,t-1), nextto(N1,N2,D).
:- moveto(N1,N2,t), moveto(N2,N1,t).

% - vertex collision ------------------------------------------------------------
:- { at(R,N,t) : at(R,_,_) } > 1, node(N,_).

goal_reached(t) :- at(R,N,t) : goal(R,N).
:- query(t), not goal_reached(t).

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

    if str(prg.solve()) == "SAT":
      break

    prg.release_external(atom)
#end.
