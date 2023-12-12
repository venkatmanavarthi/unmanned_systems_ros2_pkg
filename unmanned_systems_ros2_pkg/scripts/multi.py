#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 17 21:12:26 2021

@author: fieldstd
"""


import numpy as np
import matplotlib.pyplot as plt
import random
import time
import math as m
import dijktras_astar

random.seed(1)
obs_positions = []
import numpy as np

class Obstacle():
    """
    This makes a point obstacle with a radius 
    Takes in an x coordinate and y coordinate and
    radius in meters
    """
    def __init__(self, x_pos:float, y_pos:float, radius:float) -> None:
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.radius = radius
        
    def is_inside(self,curr_x:float, curr_y:float, 
                  robot_radius:float=0) -> bool:
        """
        Compute euclidean distance from current location -> obstacle locatin
        Store this value in a variable: dist_from 
        If dist_from > obstacle size:
                return false
        Otherwise return true
        """
        dist_from = np.sqrt((curr_x - self.x_pos)**2 + (curr_y - self.y_pos)**2)
        
        if dist_from > self.radius + robot_radius:
            return False
        
        return True
    
class Agent:
    def __init__(self, i, xi, yi, speed, N_tasks, color):
        self.i = i # my index
        self.x = xi # position
        self.y = yi # position
        self.speed = speed # how fast I can move
        self.goal_task = [float('inf'), -1] # [cost to do it, which task I want to do]
        self.team_bids = [[float('inf'),-1]]*N_tasks # [their bid, agent who bid]
        self.color = str(color) # my color
        self.obs_list = None
        self.astar = dijktras_astar.DjikstraAstar()
   
    def find_best_task(self, tasks):
        # initialize search stuff
        min_cost = float('inf') # set cost to inf so I will find something
        min_i = -1 # tracks best task index
        if self.obs_list is None:
            obs_list = [Obstacle(x=ob[0], y=ob[1], radius=0.2) for ob in obs_positions]
        #0, 0, 100, 100, 0.5
        #print('pos: ', self.x, ',', self.y)
        for i, task in enumerate(tasks): # search over all tasks
            if task.active: # don't do tasks that are done
                # use your obstacle list call out astar and find the cost
                print(self.x, self.y, task.x, task.y)
                a = self.astart.run(start=(int(self.x), int(self.y)), goal=(task.x, task.y), r_radius=0.0, obs_list=obs_list)
                if a is not None:
                    route, gcost, hcost, cost = a
                else:
                    cost = float('inf')
                print(cost)
                # cost = np.sqrt(pow(task.x - self.x,2) + pow(task.y - self.y,2)) / self.speed # cost here is time
                #print('task[', i, '].cost: ', task.x, ', ', task.y)
                if cost < min_cost and cost < self.team_bids[i][0]: # is cost better than my current best and my team's current best
                    #print('new min_cost: ', cost)
                    min_cost = cost # it is, set it as new best - cost
                    min_i = i # new best task index
        #print('min_i found: ', min_i)
        self.goal_task = [min_cost, min_i] # set my goal task
   
    def at_goal(self, world): # check if I am at my goal
        task = world.tasks[self.goal_task[1]] # get my goal task
        return np.sqrt(pow(task.x - self.x,2) + pow(task.y - self.y,2)) <= self.speed # check if I am within 1 timestep
   
    def move_towards_goal(self, world): # move towards goal
        task = world.tasks[self.goal_task[1]] # get my goal task
        dx = task.x - self.x # x dist
        dy = task.y - self.y # y dist
        dist = np.sqrt(pow(dx,2) + pow(dy,2)) # euclid dist
        dx = self.speed * dx / dist # step dist x
        dy = self.speed * dy / dist # step dist y
        self.x = self.x + dx # step in x
        self.y = self.y + dy # step in y
       
    def advertise_bid(self, agents, world): # tell my team my plans
        for agent in agents: # go through all agents
            if agent != self: # don't update team bids with my bid
                # check if I am in coms range
                if np.sqrt(pow(agent.x - self.x,2) + pow(agent.y - self.y,2)) < world.coms_range:
                    # I am in coms range, update their list about the other task
                    agent.update_bid(self.goal_task, self.i)
                   
    def update_bid(self, task_bid, agent_index): # remove old bids and update with new bid
        for bid in self.team_bids:
            # remove old bids by same agent if they exist
            if bid[1] == agent_index:
                bid[0] = float('inf') # reset
                bid[1] = -1 # reset
       
        self.team_bids[task_bid[1]] = [task_bid[0], agent_index] # set new bid
                   
class Task: # what we're trying to do
    def __init__(self, xi, yi):
        self.x = int(xi)# x position
        self.y = int(yi) # y postion
        self.active = True # still need to be done? True means yes
 
class World: # holds all tasks
    def __init__(self, l_x, l_y, N_tasks, coms_range):
        self.x = l_x # world width
        self.y = l_y # world height
        self.N_tasks = N_tasks # number of tasks
        self.coms_range = coms_range # communications range
        self.tasks = [] # holds task
        for t in range(0,N_tasks): # initialize all tasks
            self.tasks.append(Task(random.random() * self.x, random.random() * self.y)) # initialize task
           
    def work_on_task(self, agent): # complete task if agent is there
        task = self.tasks[agent.goal_task[1]] # get agents goal task
        dist = np.sqrt(pow(task.x - agent.x,2) + pow(task.y - agent.y,2)) # dist to task
        if dist < agent.speed: # is agent actually at task they're claiming...
            task.active = False # they are! complete the task
            #print('agent', agent.i, 'completed task: ', agent.goal_task[1])
           
    def complete(self): # check if all the tasks are complete
        for task in self.tasks: # check all tasks
            if task.active:
                return False # found an active task, bailout
        return True
 
# params
N_tasks = 10
N_agents = 5
N_time_steps = 1000
N_obstacles = 100
# initialize world and tasks
world = World(100, 100, N_tasks, 1000)
 
# initialize agents
agents = []
colors = ['r','b','c','y','k']

for a in range(0,N_agents):
    agents.append(Agent(a, random.random() * world.x, random.random() * world.y, 2.0, N_tasks, colors[a%5]))
 
    
task_lists = world.tasks
obs_radius = 0.2

for i in range(N_obstacles):
    x_obs = random.randint(0, 100)
    y_obs = random.randint(0, 100)
    
    for task in task_lists:
        distance = m.dist([x_obs, y_obs], [task.x, task.y])
        if distance <= obs_radius:
            continue 
        
    for each_agent in agents:
        distance = m.dist([x_obs, y_obs], [each_agent.x, each_agent.y])
        if distance <= obs_radius:
            continue

    obs_positions.append((x_obs, y_obs))



#%%
# run simulation
s_time = time.time()
world_log = []
for ts in range(0,N_time_steps): # iterate time
    world_log.append([0]*2*N_agents) # log agent movement
    for agent in agents: # go through eaach agent
        agent.find_best_task(world.tasks) # find best tasks
        agent.advertise_bid(agents, world) # tell others my plans
        if agent.at_goal(world): # check if I have reached my goal
            world.work_on_task(agent) # at goal! work on the task
        else:
            agent.move_towards_goal(world) # not at goal, move towards goal
           
        world_log[-1][agent.i*2] = agent.x# log position
        world_log[-1][agent.i*2+1] = agent.y # log position
    if world.complete(): # check if all the tasks are complete
        break
    # for wl in world_log:
    #     for agent in agents:
    #         plt.plot(wl[agent.i*2], wl[agent.i*2+1],agent.color+'.')

    # # make a pretty plot
    # for pos in obs_positions:
    #     plt.plot(pos[0], pos[1], 'r')
    
    # for agent in agents:
    #     plt.plot(world_log[0][agent.i*2], world_log[0][agent.i*2+1], agent.color+'o')
 
    # for task in world.tasks:
    #     plt.plot(task.x, task.y, 'gs')
        
    # for pos in obs_positions:
    #     plt.scatter(pos[0], pos[1], c='cyan')
    

    # plt.grid()
    # plt.show()
    # plt.pause(0.05)
 

print('simulation took: ', time.time() - s_time, ' seconds')
       
# make a pretty plot
# for pos in obs_positions:
#     # print(len(pos[0]), len(pos[1]))
#     try:
#         plt.scatter(pos[0], pos[1], 'r')
#     except Exception as e:
#         print(pos)
#         print(e)

for pos in obs_positions:
    plt.plot(pos[0], pos[1], 'r')
for pos in obs_positions:
    plt.scatter(pos[0], pos[1], c='cyan')


for wl in world_log:
    for agent in agents:
        plt.plot(wl[agent.i*2], wl[agent.i*2+1],agent.color+'.')
   
for agent in agents:
    plt.plot(world_log[0][agent.i*2], world_log[0][agent.i*2+1], agent.color+'o')
 
for task in world.tasks:
    plt.plot(task.x, task.y, 'gs')
 
plt.grid()
plt.show()