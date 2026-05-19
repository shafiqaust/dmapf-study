import clingo
import time

start = time.time()

ctl = clingo.Control()
ctl.load("movement.lp")
ctl.ground([("base", [])])

for t in range(0, 100):
  print("t = {}".format(t))
  ctl.ground([("plan", [t])])

  atom_query = clingo.parse_term("query(" + str(t) + ")")
  ctl.assign_external(atom_query, True)

  if str(ctl.solve(on_model=lambda m: print("Answer: {}".format(m)))) == "SAT":
    break

  ctl.release_external(atom_query)

print("Total time = {}".format(time.time() - start))

#############################################################

# import clingo
# import time

# start = time.time()

# ctl = clingo.Control()
# ctl.load("movement2.lp")
# ctl.ground([("base", [])])

# for t in range(0, 100):
#   print("t = {}".format(t))
#   ctl.ground([("plan", [t])])

#   atom_query = clingo.parse_term("query(" + str(t) + ")")
#   ctl.assign_external(atom_query, True)

#   if str(ctl.solve(on_model=lambda m: print("Answer: {}".format(m)))) == "SAT":
#     break

#   ctl.release_external(atom_query)

# print("Total time = {}".format(time.time() - start))
