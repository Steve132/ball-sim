import sys


for fi,f in enumerate(sys.argv[1:]):
	lout=""
	lout+=f+"\t"
	lout+=str(2**fi)+"\t"
	for li,l in enumerate(open(f,'r')):
		if(li > 0):
			sr=l.split()		
			lout+=sr[len(sr)-1]+"\t"
	print lout
