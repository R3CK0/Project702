#Author: Nicholas Massad
#Date: 28/02/2023
import random

import game_engine as ge
import a_star as astar
from a_star import AStar2D
from rrt import RRT2D, RRTStar2D, InformedRRTStar2D
from fmt import FMTStar2D
from bit_star import BitStar
import time
import pandas as pd
from multiprocessing import Pool
import warnings
import numpy as np


def calculate_distance(node1, node2):
    return ((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2) ** 0.5
def path_length(path):
    length = 0
    for i in range(len(path)-1):
        length += calculate_distance(path[i], path[i+1])
    return length

class Benchmark:
    def __init__(self):
        self.window_width = 0
        self.window_height = 0
        self.grid_size = 0
        self.environement = None
        self.game_engine = None
        self.start_pos = None
        self.end_pos = None
        self.progress = False
        self.algo = {}
        self.path = None
        self.optimize_time = None
        self.free_space = None
        self.data = pd.DataFrame(columns=["Algorithm", "Size", "Obstacle Coverage", "Path Length", "Time", "Optimisation_time"])
    def build_environment(self, size, obstacle_coverage):
        if size == "small":
            self.window_width = 900
            self.window_height = 900
            self.grid_size = 30
            self.free_space = 784
            print("small : 784")
        elif size == "medium":
            self.window_width = 1200
            self.window_height = 1200
            self.grid_size = 20
            self.free_space = 3364
            print("medium : 3 364")
        elif size == "large":
            self.window_width = 1200
            self.window_height = 1200
            self.grid_size = 10
            self.free_space = 13924
            print("large : 13 924")
        elif size == "xlarge":
            self.window_width = 1200
            self.window_height = 1200
            self.grid_size = 4
            self.free_space = 88804
            print("xlarge : 88 804")
        self.game_engine = ge.GameEngine(self.window_width, self.window_height, self.grid_size, benchmark=True)
        self.game_engine.create_boundary()
        self.environement = {"width": self.window_width, "height": self.window_height, "grid_size": self.grid_size}

        obstacle_amount = int(self.free_space * obstacle_coverage)
        (x, y) = self.generate_random_point()
        self.start_pos = (x - (x % self.grid_size), y - (y % self.grid_size))
        self.game_engine.add_start_point(x, y)
        (x, y) = self.generate_random_point()
        self.end_pos = (x - (x % self.grid_size), y - (y % self.grid_size))
        self.game_engine.add_end_point(x, y)
        for i in range(obstacle_amount):
            (x, y) = self.generate_random_point()
            if (x - (x % self.grid_size), y - (y % self.grid_size)) != self.start_pos \
                    and (self.start_pos[0] + self.grid_size < x - (x % self.grid_size) or x - (x % self.grid_size) < self.start_pos[0] - self.grid_size
                    or self.start_pos[1] + self.grid_size < y - (y % self.grid_size) or y - (y % self.grid_size) < self.start_pos[1] - self.grid_size) \
                    and (self.end_pos[0] + self.grid_size < x - (x % self.grid_size) or x - (x % self.grid_size) < self.end_pos[0] - self.grid_size
                    or self.end_pos[1] + self.grid_size < y - (y % self.grid_size) or y - (y % self.grid_size) < self.end_pos[1] - self.grid_size):
                self.game_engine.add_obstacle(x, y)
        self.start_pos = (self.start_pos[0] + self.grid_size / 2, self.start_pos[1] + self.grid_size / 2)
        self.end_pos = (self.end_pos[0] + self.grid_size / 2, self.end_pos[1] + self.grid_size / 2)
    def generate_random_point(self):
        return (random.randint(self.grid_size, self.window_width-self.grid_size), random.randint(self.grid_size, self.window_height - self.grid_size))

    # TODO: load algorithms
    def load_algorithms(self):
        self.algo["astar"] = AStar2D(self.environement, self.game_engine, benchmark=True)
        self.algo["rrt"] = RRT2D(self.environement, self.game_engine, 1000, benchmark=True)
        self.algo["rrt*"] = RRTStar2D(self.environement, self.game_engine, 1000, benchmark=True)
        self.algo["informed rrt*"] = InformedRRTStar2D(self.environement, self.game_engine, 1000, benchmark=True)
        self.algo["fmt*"] = FMTStar2D(self.environement, self.game_engine, radius_multiplier=1.8, benchmark=True)
        self.algo["bit*"] = BitStar(self.environement, self.game_engine, radius_multiplier=1.8, K=150, benchmark=True)
        self.game_engine = None


    def run_benchmarking(self, attemps, start_optim, times_to_double, obstacle_coverage = None):
        map_sizes = ["small", "medium", "large", "xlarge"]
        if obstacle_coverage is None:
            obstacle_coverages = [0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35]
        else:
            obstacle_coverages = np.arange(obstacle_coverage[0], obstacle_coverage[1], 0.05)
        algorithms = ["astar", "rrt", "rrt*", "informed rrt*",'fmt*', "bit*"]
        optim_algo = ["rrt*", "informed rrt*", "bit*"]
        pool = Pool(processes=attemps)

        for size in map_sizes:
            for obstacle_coverage in obstacle_coverages:
                self.build_environment(size, obstacle_coverage)
                self.load_algorithms()
                for algo in algorithms:
                    print("Running benchmarking for {} with {}% obstacle coverage".format(algo, obstacle_coverage*100))
                    args = [tuple((algo, 0)) for i in range(attemps)]

                    results = pool.map(self.run_benchmarking_attemps, args)
                    for result in results:
                        if result is not None:
                            if algo == "astar":
                                while result[0] == 0:
                                    self.build_environment(size, obstacle_coverage)
                                    self.load_algorithms()
                                    args = [tuple((algo, 0)) for i in range(attemps)]
                                    results = pool.map(self.run_benchmarking_attemps, args)
                                    result = results[0]
                            # add data to dataframe
                            self.data = self.data.append({
                                "Algorithm": algo,
                                "Size": size,
                                "Obstacle Coverage": obstacle_coverage*100,
                                "Path Length": result[0],
                                "Time": result[1],
                                "Optimisation_time" : 0
                            }, ignore_index=True)
                    self.save_data()
        for size in map_sizes:
            for obstacle_coverage in obstacle_coverages:
                self.build_environment(size, obstacle_coverage)
                self.load_algorithms()
                for algo in optim_algo:
                    for optim in range(times_to_double):
                        optim_time = start_optim* 2**optim
                        print("Running benchmarking for {} with {}% obstacle coverage and optimisation time of {}".format(algo, obstacle_coverage*100, optim_time))
                        args = [tuple((algo, optim_time)) for i in range(attemps)]
                        results = pool.map(self.run_benchmarking_attemps, args)
                        for result in results:
                            if result is not None:
                                # add data to dataframe
                                self.data = self.data.append({
                                    "Algorithm": algo,
                                    "Size": size,
                                    "Obstacle Coverage": obstacle_coverage*100,
                                    "Path Length": result[0],
                                    "Time": result[1],
                                    "Optimisation_time" : optim_time
                                }, ignore_index=True)
                        self.save_data()


    def run_benchmarking_attemps(self, args):
        algo = args[0]
        optim_time = args[1]
        time_start = time.time()
        if optim_time == 0:
            path = self.algo[algo].find_path(self.start_pos, self.end_pos)
        else:
            path = self.algo[algo].find_path(self.start_pos, self.end_pos, optim_time)
        if path is not None:
            return tuple((path_length(path), time.time() - time_start))
        elif algo == "astar" and path is None:
            print("impassable")
            return tuple((0, time.time() - time_start))
        else:
            return None
    def save_data(self):
        self.data.to_csv("benchmark_data.csv")

def main():
    bench = Benchmark()
    bench.run_benchmarking(20, 0.1, 8)

if __name__ == "__main__":
    warnings.filterwarnings("ignore", category=DeprecationWarning)
    main()