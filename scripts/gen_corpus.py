#!/usr/bin/env python

numbers = ["one","two","three","four","five","six","seven","eight","nine","zero"]

with open("../dict/gencorp.corpus",mode="w") as corpus:
	corpus.write("halt\nstop\n")
	for each in ["left","right"]:
		for every in ["go","move","rotate","twist"]:
			corpus.write(every+" "+each+"\n")
	for each in ["go","move"]:
		for every in ["","back "]:
			for other in numbers:
				corpus.write(each+" "+every+other+"\n")
	for each in numbers:
		for every in ["roslaunch node ","x ","y "]:
			corpus.write(every+each+"\n")
			if every != "roslaunch node ":
				corpus.write(every+"negative "+each+"\n")
	corpus.write("status\n")
	corpus.write("p field\n")
	corpus.write("goto\n")
	corpus.write("full\n")
	corpus.write("half\n")
	corpus.write("ignore\n")
	corpus.write("hey listen\n")
	corpus.write("exit\n")

    

