robot(R)          :- init(object(robot,R),_).
isRobot(robot(R)) :- robot(R).

position((X,Y))            :- init(object(node,_), value(at,(X,Y))).
position(robot(R),(X,Y),0) :- init(object(robot,R),value(at,(X,Y))).

direction((X,Y)) :- X=-1..1, Y=-1..1, |X+Y|=1.
nextto((X,Y),(DX,DY),(X',Y')) :- direction((DX,DY)), position((X,Y)), position((X',Y')),
                                 (X,Y)=(X'-DX,Y'-DY), (X',Y')=(X+DX,Y+DY).

#program plan(t).
#external query(t).
{ move(R,D,t) : direction(D) } 1 :- isRobot(R).

% - move/3 ----------------------------------------------------------------------
position(R,C,t) :- move(R,D,t), position(R,C',t-1), nextto(C',D,C).
:- move(R,D,t), position(R,C,t-1), not nextto(C,D,_).

% - inertia ---------------------------------------------------------------------
position(R,C,t) :- position(R,C,t-1), not move(R,_,t), isRobot(R).

% - edge collision --------------------------------------------------------------
moveto(C',C,t) :- nextto(C',D,C), position(R,C',t-1), move(R,D,t).
:- moveto(C',C,t), moveto(C,C',t), C < C'.

% - vertex collision ------------------------------------------------------------
:- { position(R,C,t) : isRobot(R) } > 1, position(C).

% - auxiliaries -----------------------------------------------------------------
:- { position(R,C,t) } != 1, isRobot(R).    % REDUNDANT but PERFORMANT?

% - enforce assigned goal locations
:- not position(R,C,t), goal(R,C), query(t).

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
