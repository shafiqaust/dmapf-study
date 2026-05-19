1 { m(R,(X2,Y2),C3) : l((X2,Y2),C3) } 1 :- a(R,(X1,Y1)).
0 { m(R,(X2,Y2),C3) : l((X2,Y2),C3) } 1 :- b(R,(X1,Y1)).
1 { m(R,(X2,Y2),C3) : l(C3,(X2,Y2)) } 1 :- c(R,(X1,Y1)).
0 { m(R,(X2,Y2),C3) : l(C3,(X2,Y2)) } 1 :- d(R,(X1,Y1)).
:- m(R1,N,_), m(R2,N,_), R1 != R2.
:- m(R1,_,N), m(R2,_,N), R1 != R2.
:- m(_,N1,N2), m(_,N2,N1).
:- #count { R : m(R,_,_) } != L, l(L).

s(R,D) :- m(R,(X',Y'),_), a(R,(X,Y)), D=|X'-X|+|Y'-Y|.
s(R,D) :- m(R,(X',Y'),_), b(R,(X,Y)), D=|X'-X|+|Y'-Y|.
s(R,D) :- m(R,(X',Y'),_), c(R,(X,Y)), D=|X'-X|+|Y'-Y|.
s(R,D) :- m(R,(X',Y'),_), d(R,(X,Y)), D=|X'-X|+|Y'-Y|.
#minimize { D : s(_,D) }.

#show m/3.
#show s/2.
