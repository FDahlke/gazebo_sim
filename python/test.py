#!/usr/bin/env python
# coding: utf-8

# In[1]:


import time
import cv2

import numpy as np

from swarm import Swarm
from person import Person


# In[2]:


NUM_DRONES = 3
GRID_SIZE = 100         # Size of the Forest
MOVE_DISTANCE = 1       # How far a Drone can move each Timestep
TIMESTEPS = 10
POPULATION_SIZE = 50
NUM_GENERATIONS = 10
MUTATION_RATE = 0.1


# In[3]:


from pymoo.algorithms.soo.nonconvex.de import DE
from pymoo.optimize import minimize
from pymoo.operators.sampling.lhs import LHS

algorithm = DE(
    pop_size=100,
    sampling=LHS(),
    variant="DE/rand/1/bin",
    CR=0.3,
    dither="vector",
    jitter=False
)
#TODO: define Problem correctly

#res = minimize(problem,
#               algorithm,
#               verbose=True,
#               seed=1,
#
#print("Best solution found: \nX = %s\nF = %s" % (res.X, res.F))


# In[4]:


def evaluate_solution(waypoints):
    # Placeholder for the scoring function
    score = 0
    # Chance of finding target x Ground Visibility x Distance to other Drones
    return score

    #TODO: Pymoo
def check_ground_visible(swarm, ids):
    for id in ids:
        depth_image = swarm.depth_images[id]
        #TODO: Save for position
        break

#def getNextWaypoints():
    


# In[5]:


#def mutate(solution):
    # Waypints in Solution get randomized
def crossover(parent1, parent2):
    crossover_point = random.randint(0, TIMESTEPS - 1)
    child = parent1[:crossover_point] + parent2[crossover_point:]
    return child


# Initialization of Drones and Solution Populations

# In[6]:


#Spawn Drones and move to initial position

# Create the swarm object by passing the name
# of the world from the .sdf world file.
swarm = Swarm("rhel_swarm")
#person = Person("rhel_swarm")

# Spawn X drones and keep the returning ids as handles
ids = swarm.spawn(NUM_DRONES)

# First waypoints
waypoints = np.empty((0,3),float)
for i in range (NUM_DRONES):
    waypoints= np.append(waypoints,np.array([[i-(NUM_DRONES/2),0.0,35.0]]), axis=0)

print(NUM_DRONES)
print(waypoints)

for d_z in range(0, 30, 5):
    print("Starting for")
    print(d_z)
    # Set the waypoints for all drones
    current_waypoints = waypoints - np.array([0.0, 0.0, d_z])
    swarm.waypoints(ids, current_waypoints)

    # Wait until the data has arrived
    time_delta = 0.01           # Delta time per sleep command in seconds
    time_passed = 0.0           # Time counter to keep track of the time in seconds
    timeout = 1.0               # Timeout in case something goes wrong

    timeout_occured = False

    while time_passed < timeout:
        # All frames for a waypoint called send
        # together, so it is enough to check the
        # last one.
        if swarm.received_frames[ids[-1]]:
            print("All frames recieved")
            for id in ids:
                rgb_image = swarm.rgb_images[id]
                #thermal_image = swarm.thermal_images[id]
                depth_image = swarm.depth_images[id]
                print(depth_image)
            break

        time.sleep(time_delta)
        time_passed += time_delta
    if time_passed >= timeout:
            timeout_occured = True
            print(f"Timeout reached for waypoints with dz={d_z}")

if timeout_occured:
    raise TimeoutError("Timeout occured while waiting for waypoint")


# In[ ]:




