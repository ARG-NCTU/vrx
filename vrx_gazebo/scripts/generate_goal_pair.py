#!/usr/bin/env python3
import random
import csv
import math

class GoalPairGenerate:
    def __init__(self, csv_path, pair_count=10, fixed_distance=50):
        self.csv_path = csv_path
        self.pair_count = pair_count
        self.fixed_distance = fixed_distance

    def generate_random_goal(self):
        while True:
            start_x = random.randint(0, 100)
            start_y = random.randint(0, 100)

            angle = random.uniform(0, 2 * math.pi)

            goal_x = start_x + self.fixed_distance * math.cos(angle)
            goal_y = start_y + self.fixed_distance * math.sin(angle)

            if 0 <= goal_x <= 100 and 0 <= goal_y <= 100:
                break
        
        return start_x, start_y, goal_x, goal_y

    def write_to_csv(self):
        with open(self.csv_path, 'w', newline='') as csvfile:
            fieldnames = ['StartX', 'StartY', 'GoalX', 'GoalY']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            for _ in range(self.pair_count):
                start_x, start_y, goal_x, goal_y = self.generate_random_goal()
                writer.writerow({'StartX': start_x, 'StartY': start_y, 'GoalX': goal_x, 'GoalY': goal_y})
        print('Done')

if __name__ == '__main__':
    path = './start_goal_pairs.csv'
    goal_pair_generate = GoalPairGenerate(csv_path = path, pair_count=300, fixed_distance=50)
    goal_pair_generate.write_to_csv()

