#external q(t).
{m(R,D,t):d(D)}1:-r(R),t>0.
a(R,C',t):-a(R,C,t-1),m(R,D,t),x(C,C',D).
:-m(R,D,t),a(R,C,t-1),not x(C,_,D).
a(R,C,t):-a(R,C,t-1),not m(R,_,t).
w(C,C',t):-a(R,C,t-1),m(R,D,t),x(C,C',D).
:-w(C,C',t),w(C',C,t).
:-{a(_,C,t)}>1,i(C).
g(t):-a(R,C,t):g(r(R),C).
:-not g(t),q(t).
:-g(t),a(R,C,t),c(C),not g(r(R),C),q(t).
#show m/3.
