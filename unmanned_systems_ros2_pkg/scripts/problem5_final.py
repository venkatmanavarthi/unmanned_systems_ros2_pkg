import numpy as np
import matplotlib.pyplot as plt
import time
import math as m
import pylab as pl
import tqdm

from AStartAlgorithm import AStartAlgorithm
from Dijkstras import Obstacle

np.random.seed(42)

class Path_store:
    def __init__(self, x, y, cost):
        self.x = x
        self.y = y
        self.cost = cost

deliver = [(0, 0), (9,4), (4,4), (1,9), (9,7), (6,14)]

gx = [x[0] for x in deliver]
gy = [x[1] for x in deliver]
cities = [1, 2, 3, 4, 5]

depth_paths = dict()


sx = 0
sy = 0

ox = [2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8, 8, 8, 8, 8, 2, 3, 4, 5, 6, 9, 10, 11, 12, 15, 2, 2, 2,  2,  2,  2,  5, 5,  5,  5,  5,  5,  5,  6,  7,  8,  9,  10, 11, 12, 12, 12, 12, 12]
oy = [2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2,  2,  2,  2,  3, 4, 5, 6, 7, 8, 9, 7, 7, 7, 7, 7, 6, 6,  6,  6,  6,  8, 9, 10, 11, 12, 13, 9, 10, 11, 12, 13, 14, 15, 12, 12, 12, 12, 12, 12, 8,  9,  10, 11, 12]
      
grid_size = 0.5
grid_x = 15
grid_y = 15
min_x = 0
min_y = 0
obs_radius = 0.1
robot_radius = 0.5 
x_min = 0
x_max = 15
y_min = 0
y_max = 15
grid_space = 0.5

# Initialize a cost matrix (distance cost to/from each waypoint)
cost_matrix = np.zeros([len(gx), len(gx)])
path_matrix = dict()
temp = dict()
depth_paths = dict()

obs_list = [Obstacle(ox[i], oy[i], obs_radius)
            for i in range(len(ox))]
t = time.time()
for i in range(0,len(gx)):
    for j in range(0,len(gx)): # Skip a to a estimate
        if i!=j:
            start_location = [gx[i],gy[i]]
            next_location =  [gx[j],gy[j]]
            if (tuple(start_location), tuple(next_location)) in temp:
                cost_matrix[i, j] = temp.get((tuple(start_location), tuple(next_location)))
                continue
            if (tuple(next_location), tuple(start_location)) in temp:
                cost_matrix[i, j] = temp.get((tuple(next_location), tuple(start_location)))
                continue
            # applying a star
            # print(start_location, next_location)
            astart = AStartAlgorithm(0, 0, 15, 15, 0.5)
            route, gcost, hcost, cost = astart.run(start=(start_location[0], start_location[1]), goal=(next_location[0], next_location[1]), r_radius=robot_radius, obs_list=obs_list)
            temp_cost = gcost
            cost_matrix[i,j] = temp_cost
            temp[tuple(start_location), tuple(next_location)] = temp_cost
            temp[tuple(next_location), tuple(start_location)] = temp_cost

            
            depth_paths[tuple(start_location), tuple(next_location)] = [[each[0], each[1]] for each in route]
            depth_paths[tuple(next_location), tuple(start_location)] = [[each[0], each[1]] for each in route]


adjacency_mat = cost_matrix

class Population():
    def __init__(self, bag, adjacency_mat):
        self.bag = bag
        self.parents = []
        self.score = 0
        self.best = None
        self.adjacency_mat = adjacency_mat

def init_population(cities, adjacency_mat, n_population):
    return Population(
        np.asarray([np.random.permutation(cities) for _ in range(n_population)]), 
        adjacency_mat
    )

pop = init_population(cities, adjacency_mat, 50)

def fitness(self, chromosome):
    return (sum([
        self.adjacency_mat[chromosome[i], chromosome[i + 1]]
        for i in range(len(chromosome) - 1)]) 
        + self.adjacency_mat[0, chromosome[0]])

Population.fitness = fitness


def evaluate(self):
    distances = np.asarray(
        [self.fitness(chromosome) for chromosome in self.bag]
    )
    self.score = np.min(distances)
    self.best = self.bag[distances.tolist().index(self.score)]
    self.parents.append(self.best)
    if False in (distances[0] == distances):
        distances = np.max(distances) - distances
    return distances / np.sum(distances)
    
Population.evaluate = evaluate

def select(self, k=4):
    fit = self.evaluate()
    while len(self.parents) < k:
        idx = np.random.randint(0, len(fit))
        if fit[idx] > np.random.rand():
            self.parents.append(self.bag[idx])
    self.parents = np.asarray(self.parents)

Population.select = select

def swap(chromosome):
    a, b = np.random.choice(len(chromosome), 2)
    chromosome[a], chromosome[b] = (
        chromosome[b],
        chromosome[a],
    )
    return chromosome

def crossover(self, p_cross=0.1):
    children = []
    count, size = self.parents.shape
    for _ in range(len(self.bag)):
        if np.random.rand() > p_cross:
            children.append(
                list(self.parents[np.random.randint(count, size=1)[0]])
            )
        else:
            parent1, parent2 = self.parents[
                np.random.randint(count, size=2), :
            ]
            idx = np.random.choice(range(size), size=2, replace=False)
            start, end = min(idx), max(idx)
            child = [None] * size
            for i in range(start, end + 1, 1):
                child[i] = parent1[i]
            pointer = 0
            for i in range(size):
                if child[i] is None:
                    while parent2[pointer] in child:
                        pointer += 1
                    child[i] = parent2[pointer]
            children.append(child)
    return children

Population.crossover = crossover

def mutate(self, p_cross=0.1, p_mut=0.1):
    next_bag = []
    children = self.crossover(p_cross)
    for child in children:
        if np.random.rand() < p_mut:
            next_bag.append(swap(child))
        else:
            next_bag.append(child)
    return next_bag
    
Population.mutate = mutate



def genetic_algorithm(
    cities,
    adjacency_mat,
    n_population=50,
    n_iter=1000,
    selectivity=0.75,
    p_cross=0.5,
    p_mut=0.3,
    print_interval=100,
    return_history=False,
    verbose=False,
):
    pop = init_population(cities, adjacency_mat, n_population)
    best = pop.best
    score = float("inf")
    history = []
    for i in tqdm.tqdm(range(n_iter)):
        # print(i)
        pop.select(n_population * selectivity)
        history.append(pop.score)
        if verbose:
            pass
            # print(f"Generation {i}: {pop.score}")
        elif i % print_interval == 0:
            pass
            # print(f"Generation {i}: {pop.score}")
        if pop.score < score:
            best = pop.best
            score = pop.score
        children = pop.mutate(p_cross, p_mut)
        pop = Population(children, pop.adjacency_mat)
    if return_history:
        return best, history
    return best
# print(adjacency_mat)
#returns the best order of waypoinst to visit
best = genetic_algorithm(cities, adjacency_mat, verbose=False, n_population=500, n_iter=2000)
#add the starting point to the list of waypoints
best = np.insert(best, 0, 0)

#best order of waypoints to visit
wp_order = []
for i in range(0,len(best)):
    wp_order.append([gx[best[i]],gy[best[i]]])
    
print("best order of waypoints to visit", wp_order)
elapsed_time = time.time() - t
print(elapsed_time)

# # Plotting   
# plt.plot(ox, oy, ".b")
# plt.plot(sx, sy, "xg")
# plt.plot(gx, gy, "xr")
# plt.axis([min_x-grid_size, grid_x+grid_size, min_y-grid_size, grid_y+grid_size])

# for i in range(1,len(best)):
#     plt.plot(path_matrix[best[i-1],best[i]].x,path_matrix[best[i-1],best[i]].y,'r-')
# #plt.plot(pathx, pathy, "-r")
# plt.xlabel('X Distance')
# plt.ylabel('Y Distance')
# plt.show()


fig, ax = plt.subplots()

x_values = np.arange(x_min, x_max + grid_space, grid_space)
y_values = np.arange(y_min, y_max + grid_space, grid_space)

# for y, x in zip(y_values, x_values):
#     index = compute_index(x_min, x_max, y_min, y_max, grid_space, x, y)
    # plt.text(x, y, str(index), color='red', fontsize=8, ha='center', va='center')

for obs in obs_list:
    obs_plot = plt.Circle((obs.x, obs.y), obs.radius, color='black')
    ax.add_patch(obs_plot)

for p in deliver:
    obs_plot = plt.Circle((p[0], p[1]), 0.5, color='green')
    ax.add_patch(obs_plot)

# obs_plot = plt.Circle((self.goal[0], self.goal[1]), self.r_radius, color='green')
# ax.add_patch(obs_plot)
for i in range(0, len(wp_order)-1):
    p = depth_paths[tuple(wp_order[i]), tuple(wp_order[i+1])]
    plt.plot([x[0] for x in p], [x[1] for x in p], c='red')

plt.xlim(x_min - grid_space, x_max + grid_space)
plt.ylim(y_min - grid_space, y_max + grid_space)
plt.show()
