direction((X,Y)) :- X=-1..1, Y=-1..1, |X+Y|=1.
nextto(N1,N2,(DX,DY)) :- node(N1,(X,Y)), node(N2,(X+DX,Y+DY)), direction((DX,DY)).

#program init(i).
at(R,N,i,0) :- at(R,N,i-1,T), T = #max { U : at(R,_,i-1,U) }, not migrate(R,i-1).

%% Is the code below necessary?
1 { subgoal(R1,N2,i) : node(N2,_), N2 != N1 } 1 :- at(R1,N1,i,0), at(R2,N1,i,1), not migrate(R2,i).
goal(R,N,i) :- subgoal(R,N,i).
:- goal(R1,N,i), goal(R2,N,i), R1 != R2.
%%- select the closest sub goal
:- at(R,N1,i,0), subgoal(R,N2,i), node(N1,(X1,Y1)), node(N2,(X2,Y2)), node(N3,(X3,Y3)), not goal(_,N3,i), N1 != N3, |X3-X1|+|Y3-Y1| < |X2-X1|+|Y2-Y1|.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#program plan(i,t).
#external query(i,t).

{ move(R,D,i,t) : direction(D) } 1 :- at(R,_,i,t-1).

% - move/3 ----------------------------------------------------------------------
at(R,N2,i,t) :- move(R,D,i,t), at(R,N1,i,t-1), nextto(N1,N2,D).
             :- move(R,D,i,t), at(R,N,i,t-1), not nextto(N,_,D).

% - inertia ---------------------------------------------------------------------
at(R,N,i,t) :- at(R,N,i,t-1), not move(R,_,i,t).

% - edge collision --------------------------------------------------------------
moveto(N1,N2,i,t) :- move(R,D,i,t), at(R,N1,i,t-1), nextto(N1,N2,D).
:- moveto(N1,N2,i,t), moveto(N2,N1,i,t).

% - vertex collision ------------------------------------------------------------
:- { at(R,N,i,t) : at(R,_,i,_) } > 1, node(N,_).

goal_reached(i,t) :- at(R,N,i,t) : goal(R,N,i).
:- query(i,t), not goal_reached(i,t).

% #show move/4.

#script (python)
import clingo

def main(prg):
  prg.ground([("base", [])])

  # prg.add("g", [], "goal(14,240,0).migrate(14,0).goal(39,167,0).migrate(39,0).at(57,1261,2,1).move(57,(-1,0),2,1).at(29,1115,2,1).move(29,(0,1),2,1).at(63,292,2,1).move(63,(0,1),2,1).at(55,314,2,1).move(55,(0,1),2,1).at(30,255,2,1).move(30,(1,0),2,1).at(34,240,2,1).move(34,(-1,0),2,1).at(42,427,2,1).move(42,(-1,0),2,1).at(45,1021,2,1).move(45,(0,-1),2,1).goal(57,236,2).migrate(57,2).goal(55,167,2).migrate(55,2).goal(29,257,2).migrate(29,2).goal(42,255,2).migrate(42,2).goal(34,244,2).migrate(34,2).at(44,1115,3,1).move(44,(0,1),3,1).at(59,292,3,1).move(59,(0,1),3,1).at(9,1021,3,1).move(9,(0,-1),3,1).at(5,150,3,1).move(5,(0,-1),3,1).goal(59,167,3).migrate(59,3).goal(44,240,3).migrate(44,3).at(38,255,4,1).move(38,(1,0),4,1).goal(38,240,4).migrate(38,4).goal(63,179,5).goal(30,299,5).goal(9,223,5).goal(45,167,5).goal(5,360,5).")
  # prg.ground([("g", [])])

  for i in range(0, 2):
    if i == 0:
      prg.add("i0", [], "goal(14,240,0).migrate(14,0).goal(39,167,0).migrate(39,0).")
      prg.ground([("i0", [])])
    elif i == 1:
      prg.add("i1", [], "")
      prg.ground([("i1", [])])
    elif i == 2:
      prg.add("i2", [], "at(57,1261,2,1).move(57,(-1,0),2,1).at(29,1115,2,1).move(29,(0,1),2,1).at(63,292,2,1).move(63,(0,1),2,1).at(55,314,2,1).move(55,(0,1),2,1).at(30,255,2,1).move(30,(1,0),2,1).at(34,240,2,1).move(34,(-1,0),2,1).at(42,427,2,1).move(42,(-1,0),2,1).at(45,1021,2,1).move(45,(0,-1),2,1).goal(57,236,2).migrate(57,2).goal(55,167,2).migrate(55,2).goal(29,257,2).migrate(29,2).goal(42,255,2).migrate(42,2).goal(34,244,2).migrate(34,2).")
      prg.ground([("i2", [])])
    elif i == 3:
      prg.add("i3", [], "at(44,1115,3,1).move(44,(0,1),3,1).at(59,292,3,1).move(59,(0,1),3,1).at(9,1021,3,1).move(9,(0,-1),3,1).at(5,150,3,1).move(5,(0,-1),3,1).goal(59,167,3).migrate(59,3).goal(44,240,3).migrate(44,3).")
      prg.ground([("i3", [])])
    elif i == 4:
      prg.add("i4", [], "at(38,255,4,1).move(38,(1,0),4,1).goal(38,240,4).migrate(38,4).")
      prg.ground([("i4", [])])
    elif i == 5:
      prg.add("i5", [], "goal(63,179,5).goal(30,299,5).goal(9,223,5).goal(45,167,5).goal(5,360,5).")
      prg.ground([("i5", [])])

    prg.ground([("init", [i])])

    for t in range(0,11):
      print("i = {}, t = {}".format(i,t))
      prg.ground([("plan", [i,t])])

      atom = clingo.parse_term("query(" + str(i) + "," + str(t) + ")")
      prg.assign_external(atom, True)

      if str(prg.solve()) == "SAT":
        break

      prg.release_external(atom)
#end.

