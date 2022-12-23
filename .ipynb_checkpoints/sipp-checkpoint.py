import yaml
import argparse
import numpy as np
from math import fabs
from bisect import bisect


class State(object):
    def __init__(self, position=(-1, -1), time=0, interval=(0, np.inf)):
        self.position = tuple(position)
        self.time     = time
        self.interval = interval


class Grid(object):
    def __init__(self):
        self.interval_list = [(0, np.inf)]
        self.f = np.inf
        self.g = np.inf
        self.parent_state = State()
    
    
    def split_interval(self, s, last=False):
        
        for interval in self.interval_list:
            
            if last:
                if s <= interval[0]:
                    self.interval_list.remove(interval)
                elif s > interval[1]:
                    continue
                else:
                    self.interval_list.remove(interval)
                    self.interval_list.append((interval[0], s - 1))
            else:
                if s == interval[0]:
                    self.interval_list.remove(interval)
                    if s + 1 <= interval[1]:
                        self.interval_list.append((s + 1, interval[1]))
                
                elif s == interval[1]:
                    self.interval_list.remove(interval)
                    if s - 1 <= interval[0]:
                        self.interval_list.append((interval[0], s - 1))
                
                elif bisect(interval, s) == 1:
                    self.interval_list.remove(interval)
                    self.interval_list.append((interval[0], s - 1))
                    self.interval_list.append((s + 1, interval[1]))
            
            self.interval_list.sort()


class Graph(object):
    def __init__(self, map):
        self.map           = map
        self.dimensions    = map["map"]["dimensions"]
        self.obstacles     = [tuple(v) for v in map["map"]["obstacles"]]        
        self.dyn_obstacles = map["dynamic_obstacles"]
        
        self.sipp_graph = {}
        self.init_graph()
        self.init_intervals()
    

    def init_graph(self):
        
        for i in range(self.dimensions[0]):
            for j in range(self.dimensions[1]):
                self.sipp_graph.update({(i, j) : Grid()})
    

    def init_intervals(self):
        
        if not self.dyn_obstacles:
            return
        
        for schedule in self.dyn_obstacles.values():
            for i in range(len(schedule)):
                
                location = schedule[i]
                last     = i == len(schedule) - 1
                position = (location['x'], location['y'])
                self.sipp_graph[position].split_interval(location['t'], last)
                

    def position_is_valid(self, position):
        
        obs_check = position not in self.obstacles
        dim_check = position[0] in range(self.dimensions[0]) and  position[1] in range(self.dimensions[1])
        
        return dim_check and obs_check
    

    def get_neighbours(self, position):
        
        neighbours = []
        dirs = [(position[0], position[1] + 1), # up
                (position[0], position[1] - 1), # down
                (position[0] - 1, position[1]), # left
                (position[0] + 1, position[1])] # right
        
        for dir_ in dirs:
            if self.position_is_valid(dir_):
                neighbours.append(dir_)

        return neighbours


class Planner(Graph):
    def __init__(self, map, agent_id):
        Graph.__init__(self, map)
        self.start = tuple(map["agents"][agent_id]["start"])
        self.goal  = tuple(map["agents"][agent_id]["goal"])
        self.name  = map["agents"][agent_id]["name"]

    def get_successors(self, state):
        
        m_time     = 1
        successors = []
        neighbours = self.get_neighbours(state.position)

        for neighbour in neighbours:
            
            start_time = state.time + m_time
            end_time   = state.interval[1] + m_time
            
            for intr in self.sipp_graph[neighbour].interval_list:
                
                if intr[0] > end_time or intr[1] < start_time:
                    continue
                    
                time = max(start_time, intr[0]) 
                successors.append(State(neighbour, time, intr))
        
        return successors
    

    def get_heuristic(self, position):
        return fabs(position[0] - self.goal[0]) + fabs(position[1] - self.goal[1])


    def compute_plan(self):
        self.open = []
        wip = False
        cost = 1

        s_start = State(self.start, 0) 

        self.sipp_graph[self.start].g = 0.
        f_start = self.get_heuristic(self.start)
        self.sipp_graph[self.start].f = f_start

        self.open.append((f_start, s_start))

        while (not wip):
            if self.open == {}: 
                # Plan not found
                return 0
            s = self.open.pop(0)[1]
            successors = self.get_successors(s)
    
            for successor in successors:
                if self.sipp_graph[successor.position].g > self.sipp_graph[s.position].g + cost:
                    self.sipp_graph[successor.position].g = self.sipp_graph[s.position].g + cost
                    self.sipp_graph[successor.position].parent_state = s

                    if successor.position == self.goal:
                        print("Plan successfully calculated!!")
                        wip = True
                        break

                    self.sipp_graph[successor.position].f = self.sipp_graph[successor.position].g + self.get_heuristic(successor.position)
                    self.open.append((self.sipp_graph[successor.position].f, successor))

        # Tracking back
        wip = False
        self.plan = []
        current = successor
        while not wip:
            self.plan.insert(0,current)
            if current.position == self.start:
                wip = True
            current = self.sipp_graph[current.position].parent_state
        return 1
    
    
    def get_plan(self):
        
        path = []
        setpoint = self.plan[0]
        path.append({"x":setpoint.position[0],
                     "y":setpoint.position[1],
                     "t":setpoint.time})

        for i in range(len(self.plan) - 1):
            for j in range(self.plan[i + 1].time - self.plan[i].time - 1):
                setpoint = self.plan[i]
                temp_dict = {"x" : self.plan[i].position[0],
                             "y" : self.plan[i].position[1],
                             "t" : self.plan[i].time + j + 1}
                path.append(temp_dict)
            
            path.append({"x" : self.plan[i+1].position[0],
                         "y" : self.plan[i+1].position[1],
                         "t" : self.plan[i+1].time})
        
        return {self.name: path}
