root(2,4).
root(1,9).
root(7,13).
root(19,14).
root(16,18).
root(8,30).
root(10,33).
root(11,43).
root(15,45).
root(5,52).
root(20,54).
root(17,58).
root(9,62).
root(6,70).
root(18,71).
root(14,72).
root(4,77).
root(3,87).
root(12,95).
root(13,96).
dest(2,1).
dest(1,43).
dest(7,83).
dest(19,50).
dest(16,30).
dest(8,6).
dest(10,67).
dest(11,22).
dest(15,96).
dest(5,26).
dest(20,38).
dest(17,23).
dest(9,29).
dest(6,98).
dest(18,37).
dest(14,3).
dest(4,56).
dest(3,39).
dest(12,91).
dest(13,97).
node(73,(10,4),1).
node(26,(7,2),1).
node(18,(2,4),1).
node(95,(7,9),1).
node(89,(4,9),1).
node(64,(1,9),1).
node(44,(5,9),1).
node(63,(2,9),1).
node(58,(3,8),1).
node(47,(4,8),1).
node(71,(6,8),1).
node(92,(2,8),1).
node(86,(9,8),1).
node(78,(6,5),1).
node(20,(2,5),1).
node(67,(1,5),1).
node(9,(10,5),1).
node(56,(9,5),1).
node(8,(3,5),1).
node(85,(8,5),1).
node(41,(2,7),1).
node(90,(1,8),1).
node(43,(9,6),1).
node(10,(6,7),1).
node(79,(5,7),1).
node(65,(1,7),1).
node(77,(8,7),1).
node(99,(7,7),1).
node(2,(10,2),1).
node(35,(10,7),1).
node(62,(9,7),1).
node(37,(7,8),1).
node(84,(4,7),1).
node(14,(8,8),1).
node(61,(8,6),1).
node(27,(9,9),1).
node(74,(7,6),1).
node(87,(3,7),1).
node(40,(2,6),1).
node(50,(5,8),1).
node(3,(5,6),1).
node(100,(9,1),1).
node(96,(8,9),1).
node(49,(1,6),1).
node(33,(10,8),1).
node(80,(6,6),1).
node(51,(5,5),1).
node(4,(10,6),1).
node(70,(3,6),1).
node(22,(4,6),1).
node(36,(7,5),1).
node(6,(4,5),1).
node(11,(3,9),1).
node(57,(7,10),1).
node(68,(8,10),1).
node(29,(4,10),1).
node(38,(6,10),1).
node(60,(10,9),1).
node(94,(6,9),1).
node(25,(5,10),1).
node(48,(9,10),1).
node(69,(3,10),1).
node(23,(10,10),1).
node(93,(2,10),1).
node(1,(1,10),1).
node(98,(4,2),1).
node(21,(3,1),1).
node(24,(5,1),1).
node(54,(4,1),1).
node(7,(1,1),1).
node(15,(10,1),1).
node(82,(7,1),1).
node(53,(8,1),1).
node(83,(8,4),1).
node(17,(9,4),1).
node(72,(5,4),1).
node(76,(7,4),1).
node(32,(1,4),1).
node(19,(2,1),1).
node(66,(6,4),1).
node(88,(4,4),1).
node(55,(9,3),1).
node(13,(10,3),1).
node(46,(1,3),1).
node(97,(1,2),1).
node(30,(3,3),1).
node(42,(7,3),1).
node(31,(4,3),1).
node(91,(6,3),1).
node(45,(3,2),1).
node(81,(6,1),1).
node(34,(8,3),1).
node(59,(3,4),1).
node(12,(5,3),1).
node(5,(2,3),1).
node(52,(2,2),1).
node(28,(9,2),1).
node(75,(6,2),1).
node(16,(5,2),1).
node(39,(8,2),1).

direction(0,(-1,0)).  % Direction ID must start from 0
direction(1,(1,0)).
direction(2,(0,-1)).
direction(3,(0,1)).
nextto(N1,N2,D) :- node(N1,(X,Y),A), node(N2,(X+DX,Y+DY),A), direction(D,(DX,DY)).

#show. 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#program plan(t).
#external query(t).

%% Generate moves for existing robots
0 { move(R,D,t) : nextto(N,_,D) } 1 :- at(R,N,t-1), node(N,_,_).

% Force incoming robots to move in
1 { move(R,D,1) : nextto(N,_,D) } 1 :- at(R,N,0), not node(N,_,_).
:- query(0), at(_,N,0), not node(N,_,_).

% { move(R,D,t) : direction(D,_) } :- at(R,N,t-1), node(N,_,_).
% :- move(R,D1,t), move(R,D2,t), D1 != D2.
% :- move(R,D,t), at(R,N,t-1), not nextto(N,_,D).

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#script (python)
import clingo
import time

def main(prg):
  start = time.time()
  prg.ground([("base", [])])
  print("Ground base = {}".format(time.time() - start))

  start = time.time()
  prg.add("g", [], "at(2,4,0).at(1,9,0).at(7,13,0).at(19,14,0).at(16,18,0).at(8,30,0).at(10,33,0).at(11,43,0).at(15,45,0).at(5,52,0).at(20,54,0).at(17,58,0).at(9,62,0).at(6,70,0).at(18,71,0).at(14,72,0).at(4,77,0).at(3,87,0).at(12,95,0).at(13,96,0).goal(2,1).goal(1,43).goal(7,83).goal(19,50).goal(16,30).goal(8,6).goal(10,67).goal(11,22).goal(15,96).goal(5,26).goal(20,38).goal(17,23).goal(9,29).goal(6,98).goal(18,37).goal(14,3).goal(4,56).goal(3,39).goal(12,91).goal(13,97).")
  prg.ground([("g", [])])
  print("Set starts & goals = {}".format(time.time() - start))

  for t in range(0, 20):
    print("t = {}".format(t))

    start = time.time()
    prg.ground([("plan", [t])])
    print("Ground plan({}) = {}".format(t, time.time() - start))

    atom = clingo.parse_term("query(" + str(t) + ")")
    prg.assign_external(atom, True)

    start = time.time()
    if str(prg.solve()) == "SAT":
      print("Solve plan({}) = {}".format(t, time.time() - start))
      break
    print("Solve plan({}) = {}".format(t, time.time() - start))

    prg.release_external(atom)

#end.
