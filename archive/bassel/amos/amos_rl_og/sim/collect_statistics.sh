#This script can be called to collect statistics (e.g. number of trials 
# needed until convergence, etc).
#This script calls your developed experiment for 30 times one after another 
# (as soon as the experiment is terminated, the "start" process is called again 
# to execute the program one more time automatically).
#Note:Before you call this script please activate "Terminate after learning" 
#option in the controller code. Otherwise the process won't get terminated 
#and the next one won't get called. In addition, make sure that you activate 
#the writing option for whatever parameters you need to collect statistic 
#about (e.g. number of trials, etc).
#Note: the simulation will be called with the highest speed

for i in {1..30}
do
	./start -rtf 0
done
