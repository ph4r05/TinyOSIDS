#!/bin/env python
# simple python checker for implemented algorithm of linear regression for time synchronization
loctime =  [4294961295, 4294963295, 4294965295, 4294967295]
globtime = [1000, 2000, 3000, 4000]

# compute offset
offset = [0]*len(loctime)
for i in range(0,len(loctime)):
	offset[i] = globtime[i]-loctime[i];

# average 
newLocalAverage = sum(loctime)/len(loctime)
newOffsetAverage = sum(offset)/len(offset)
print "LocalAvg: %d\nOffsetAvg: %d" % (newLocalAverage, newOffsetAverage)

# compute localSum, offsetSum
localSum=0;
offsetSum=0;

for i in range(0,len(loctime)):
	a = loctime[i] - newLocalAverage
	b = offset[i] - newOffsetAverage
	
	localSum += a*a
	offsetSum += a*b

print "LocalSum: %d\nOffsetSum: %d" % (localSum, offsetSum)

skew = (1.0*offsetSum) / (1.0*localSum)
print "Skew: %2.5f" % skew

 
def predict(localTime):
	print "LocalTime: %d to global=%d" %(localTime, localTime + newOffsetAverage + skew * (localTime-newLocalAverage))


predict(9000)
predict(11000)
predict(12000)

