import time

from subprocess import Popen,PIPE,run

process = Popen(['gz','sim', 'RHEA_swarm_DenseForest'], '-r','-s'). stdout=PIPE, stderr=PIPE)

timeout= 5
timer = 0

while process.poll() is None:
	print(timer)
	
run(['pkill', '--full', 'gz sim server'])
run(['pkill', '--full', 'gz sim gui'])
