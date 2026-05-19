1 { m(R,(X',Y'),C,|X-X'|+|Y-Y'|) : l((X',Y'),C) } 1 :- a(R,(X,Y)).
0 { m(R,(X',Y'),C,|X-X'|+|Y-Y'|) : l((X',Y'),C) } 1 :- b(R,(X,Y)).
1 { m(R,(X',Y'),C,|X-X'|+|Y-Y'|) : l(C,(X',Y')) } 1 :- c(R,(X,Y)).
0 { m(R,(X',Y'),C,|X-X'|+|Y-Y'|) : l(C,(X',Y')) } 1 :- d(R,(X,Y)).
:- m(R1,C,_,_), m(R2,C,_,_), R1 != R2.
:- m(R1,_,C,_), m(R2,_,C,_), R1 != R2.
:- m(_,C,C',_), m(_,C',C,_).

:- #count { R : m(R,_,_,_) } != L, l(L).
#minimize { D : m(_,_,_,D) }.

#show m/4.

% 1 { m(R,(X',Y'),C,D) : l((X',Y'),C), D=|X-X'|+|Y-Y'| } 1 :- a(R,(X,Y)).
% 0 { m(R,(X',Y'),C,D) : l((X',Y'),C), D=|X-X'|+|Y-Y'| } 1 :- b(R,(X,Y)).
% 1 { m(R,(X',Y'),C,D) : l(C,(X',Y')), D=|X-X'|+|Y-Y'| } 1 :- c(R,(X,Y)).
% 0 { m(R,(X',Y'),C,D) : l(C,(X',Y')), D=|X-X'|+|Y-Y'| } 1 :- d(R,(X,Y)).
% :- m(R1,C,_,_), m(R2,C,_,_), R1 != R2.
% :- m(R1,_,C,_), m(R2,_,C,_), R1 != R2.
% :- m(_,C,C',_), m(_,C',C,_).

% :- #count { R : m(R,_,_,_) } != L, l(L).
% #minimize { D : m(_,_,_,D) }.

% #show m/4.

