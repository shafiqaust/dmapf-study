robot(R) :- init(object(robot,R),_).
node(N,C) :- init(object(node,N), value(at,C)).
at(R,N,0) :- init(object(robot,R),value(at,C)), node(N,C).

direction((X,Y)) :- X=-1..1, Y=-1..1, |X+Y|=1.
nextto(N1,N2,(DX,DY)) :- node(N1,(X,Y)), node(N2,(X+DX,Y+DY)), direction((DX,DY)).

#program plan(t).
#external query(t).
{ move(R,D,N2,t) : nextto(N1,N2,D) } :- at(R,N1,t-1).
:- move(R,D1,_,t), move(R,D2,_,t), D1 != D2.
% :- r(R,N,0), o(N), not m(R,_,_,1), q(t).

at(R,N2,t) :- at(R,N1,t-1), move(R,D,N2,t), nextto(N1,N2,D).
at(R,N,t) :- at(R,N,t-1), not move(R,_,_,t).

moveto(N1,N2,t):- at(R,N1,t-1), move(R,D,N2,t), nextto(N1,N2,D).
:- moveto(N1,N2,t), moveto(N2,N1,t).

:- { at(_,N,t) } > 1, node(N,_).

:- query(t), not at(R,N,t), node(N,C), goal(robot(R),C).
% :-q(t),g(t),r(R,N,t),c(N),not g(R,N).
#show move/4.

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

