
import sys
sys.path.append("/home/sebastian/Documents/automated-driving/python")
sys.path.append("/home/sebastian/Documents/automated-driving/build")
sys.path.append("/home/sebastian/Documents/automated-driving/python_binding")
sys.path.append("/home/sebastian/Documents/automated-driving/build/python_binding")
sys.path.append("/home/sebastian/Documents/automated-driving/python/fvks")
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.prediction.spot_prediction import SpotPrediction
from spot import *
import time
import matplotlib.pyplot as plt
from commonroad.visualization.draw_dispatch_cr import draw_object

# use the CommonRoadFileReader to read a XML-file
scenario, planning_task = CommonRoadFileReader('Scenarios/Release_2018a/example.xml').open()



s = SpotPrediction(0, 40, 0.1)
s.createRoadNetwork(scenario.lanelet_network.lanelets)
start = time.time()
s.setObstacles(scenario)
#s.setComputeOccDynamicBasedOfPedestrians(False)
s.setbCrossOfPedestrians(False)
s.setbStopOfPedestrians(False)
#s.setAmaxOfPedestrians(0.8)
#s.setLaneWidthOfPedestrians(8)
num_threads = 1
s.calcOccupancies(num_threads, 1)
print("num_threads: ", num_threads)
print("received elapsed time: ", s.elapsed_time)
end = time.time()
print(end - start)

fig,ax = plt.subplots(1)
draw_object(scenario)
ax.autoscale_view()
plt.axis('off')
plt.gca().set_aspect('equal')
plt.show(block=True)
#run_unittests()


	
