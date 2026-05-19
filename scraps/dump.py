import time

s = ""

for i in range (0,1000):
  for j in range (0,10000):
    s += str(j) + " "
  s += "\n"

# s += "%\n"
# s += "1 2 3 4 5 6 7 8 9 10\n"
# s += "%\n"
# s += "11 12 13 14 15 16 17 18 19 20\n"

f = open("dump.txt", "w")
f.write(s)
f.close()


# start = time.time()

# for word in s.split():
#   n = int(word)

# print("Total time = {}".format(time.time() - start))
