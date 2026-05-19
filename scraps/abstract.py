# import clingo
# import time

# start = time.time()

# ctl = clingo.Control()
# ctl.load("abstract.lp")
# ctl.ground([("base", [])])

# for r in [8752, 8584, 9652, 2549, 7572, 
#           7045, 4168, 6706, 8909, 4092, 
#           645, 3218, 3302, 875, 992, 
#           4904, 2088, 5791, 3, 439, 
#           190, 547, 1651, 4836, 1161]:

#   print("r = {}".format(r))
#   atom_robot = clingo.parse_term("robot(" + str(r) + ")")
#   ctl.assign_external(atom_robot, True)

#   for i in range(0, 100):
#     print("i = {}".format(i))
#     ctl.ground([("plan", [r,i])])

#     atom_query = clingo.parse_term("query(" + str(r) + "," + str(i) + ")")
#     ctl.assign_external(atom_query, True)

#     if str(ctl.solve(on_model=lambda m: print("Answer: {}".format(m)))) == "SAT":
#       ctl.assign_external(atom_query, False)
#       break
#     ctl.assign_external(atom_query, False)

#   ctl.release_external(atom_robot)

# print("Total time = {}".format(time.time() - start))

##################################

# import clingo
# import time

# start = time.time()

# for r in [8752, 8584, 9652, 2549, 7572, 
#           7045, 4168, 6706, 8909, 4092, 
#           645, 3218, 3302, 875, 992, 
#           4904, 2088, 5791, 3, 439, 
#           190, 547, 1651, 4836, 1161]:
#   # print("r = {}".format(r))
#   ctl = clingo.Control()
#   ctl.load("abstract.lp")
#   ctl.add("base", [], "robot(" + str(r) + ").")
#   ctl.ground([("base", [])])

#   for i in range(0, 100):
#     # print("i = {}".format(i))
#     ctl.ground([("plan", [i])])

#     atom_query = clingo.parse_term("query(" + str(i) + ")")
#     ctl.assign_external(atom_query, True)

#     if str(ctl.solve(on_model=lambda m: print("Answer: {}".format(m)))) == "SAT":
#       break
#     ctl.assign_external(atom_query, False)

# print("Total time = {}".format(time.time() - start))

##################################

import clingo
import time

start = time.time()

target = {
  8752 : "270",
  8584 : "181",
  9652 : "77",
  2549 : "155",
  7572 : "306",
  7045 : "307",
  4168 : "132",
  6706 : "395",
  8909 : "107",
  4092 : "233",
  645  : "298",
  3218 : "151",
  3302 : "311",
  875  : "240",
  992  : "229",
  4904 : "101",
  2088 : "270",
  5791 : "391",
  3    : "189",
  439  : "317",
  190  : "159",
  547  : "162",
  1651 : "150",
  4836 : "111",
  1161 : "296"
}

for r in [8752, 8584, 9652, 2549, 7572, 
          7045, 4168, 6706, 8909, 4092, 
          645, 3218, 3302, 875, 992, 
          4904, 2088, 5791, 3, 439, 
          190, 547, 1651, 4836, 1161]:
  print("r = {}".format(r))
  ctl = clingo.Control()
  ctl.load("abstract.lp")
  ctl.add("base", [], "in(1,0).target(" + target[r] + ").")
  ctl.ground([("base", [])])

  for i in range(0, 100):
    # print("i = {}".format(i))
    ctl.ground([("plan", [i])])

    atom_query = clingo.parse_term("query(" + str(i) + ")")
    ctl.assign_external(atom_query, True)

    if str(ctl.solve(on_model=lambda m: print("Answer: {}".format(m)))) == "SAT":
    # if str(ctl.solve()) == "SAT":
      break
    ctl.assign_external(atom_query, False)

print("Total time = {}".format(time.time() - start))