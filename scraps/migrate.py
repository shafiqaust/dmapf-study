# import clingo
# import time

# start = time.time()

# ctl = clingo.Control()
# ctl.load("migrate.lp")   
# ctl.ground([("base", [])])

# for i in range(0,100):
#   atom = clingo.parse_term("l(5,2,10)")
#   ctl.assign_external(atom, True)
#   ctl.solve()
#   ctl.assign_external(atom, False)

#   atom = clingo.parse_term("l(5,4,10)")
#   ctl.assign_external(atom, True)
#   ctl.solve()
#   ctl.assign_external(atom, False)

#   atom = clingo.parse_term("l(5,6,10)")
#   ctl.assign_external(atom, True)
#   ctl.solve()
#   ctl.assign_external(atom, False)

#   atom = clingo.parse_term("l(5,8,10)")
#   ctl.assign_external(atom, True)
#   ctl.solve()
#   ctl.assign_external(atom, False)

# # 0.62 s
# print("Total time = {}".format(time.time() - start))

#############################################################

# import clingo
# import time

# start = time.time()

# ctl1 = clingo.Control()
# ctl1.load("migrate2.lp")   
# ctl1.ground([("base", []), ("nextto", []), ("link2", [])])

# ctl2 = clingo.Control()
# ctl2.load("migrate2.lp")   
# ctl2.ground([("base", []), ("nextto", []), ("link4", [])])

# ctl3 = clingo.Control()
# ctl3.load("migrate2.lp")  
# ctl3.ground([("base", []), ("nextto", []), ("link6", [])])

# ctl4 = clingo.Control()
# ctl4.load("migrate2.lp")   
# ctl4.ground([("base", []), ("nextto", []), ("link8", [])])

# for i in range (0,100):
#   ctl1.solve()
#   ctl2.solve()
#   ctl3.solve()
#   ctl4.solve()

# # 0.62 s
# print("Total time = {}".format(time.time() - start))

#############################################################

# import clingo
# import time

# start = time.time()

# ctl1 = clingo.Control()
# ctl1.load("migrate2.lp")
# ctl1.ground([("base", []), ("link2", [])])
# ctl1.solve(on_model=lambda m: print("Answer: {}".format(m)))

# print("Total time = {}".format(time.time() - start))

#############################################################

# import clingo
# import time

# start = time.time()

# ctl1 = clingo.Control()
# ctl1.load("migrate3.lp")
# ctl1.ground([("base", []), ("link2", [])])
# ctl1.solve(on_model=lambda m: print("Answer: {}".format(m)))

# print("Total time = {}".format(time.time() - start))

#############################################################

import clingo
import time

start = time.time()

ctl1 = clingo.Control()
ctl1.load("migrate4.lp")
ctl1.ground([("base", []), ("link2", [])])
ctl1.solve(on_model=lambda m: print("Answer: {}".format(m)))

print("Total time = {}".format(time.time() - start))
