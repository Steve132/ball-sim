#! /usr/bin/env python
import sys

f = open('even_more_predictive_data.txt', 'r')
out = open('even_more_predictive_data.csv', 'w')
out.write("Threads\tScale\tNumSpheres\tTimesteps\tTotal Time\tTotal Collisions\tWall Collisions\tSphere Collisions\t")
out.write("Total Checks\tAvg. Checks per Timestep\tAvg. Timesteps per Second\tAvg. Milliseconds per Timestep\t")
out.write("Time Dilation\tTotal Threads")
lineout = ""
for li in f.readlines():
	curlist = li.split()
	for items in curlist:
        # if the first item in the list is 'Threads', this is a new line
		if items == 'Threads':
			lineout += "\n"
			out.write(lineout)
			lineout = ""
		elif items.isdigit():
			lineout += items+"\t"
		elif items.rfind('.') != -1:
			lineout += items+"\t"
out.write(lineout)
f.close()
out.close()
