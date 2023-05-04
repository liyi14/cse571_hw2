from envs.ArmEnvironment import ArmEnvironment
from envs.CarEnvironment import CarEnvironment
import argparse
import sys
import random
import numpy as np
import matplotlib.pyplot as plt
import time

def get_args():
    parser = argparse.ArgumentParser(description='script for testing planners')

    parser.add_argument('-s', '--scene', type=str, default='2dof_robot_arm',
                        help='The environment to plan on, 2dof_robot_arm, 3dof_robot_arm, car')
    parser.add_argument('-m', '--map', type=str, default='envs/map2.txt',
                    help='The environment to plan on')    
    parser.add_argument('-p', '--planner', type=str, default='astar',
                        help='Please specify a planner: (astar, rrt, rrtstar, nonholrrt)')
    # parser.add_argument('-s', '--start', nargs='+', type=float, required=True)
    # parser.add_argument('-g', '--goal', nargs='+', type=float, required=True)
    parser.add_argument('-eps', '--epsilon', type=float, default=1.0, help='Epsilon for A*')
    parser.add_argument('-eta', '--eta', type=float, default=1.0, help='eta for RRT/RRT*')
    parser.add_argument('-b', '--bias', type=float, default=0.05, help='Goal bias for RRT/RRT*')
    parser.add_argument('--seed', type=int, default=0, help='Random seed for RRT/RRT*')
    parser.add_argument('-o', '--num_obstacles', type=int, default=2, help='Number of obstacles to add to the environment')
    parser.add_argument('-v', '--visualize', action='store_true', help='Visualize the configuration space')

    return parser.parse_args()

if __name__ == "__main__":
    args = get_args()
        
    if args.num_obstacles == 0:
        obstacles = []
    elif args.num_obstacles == 1:
        obstacles = [([-0.3, 0, 1.3], [0.25, 0.25, 0.25])]
    elif args.num_obstacles == 2:
        obstacles = [([0.3, 0, 0.6], [0.25, 0.25, 0.25]),
                    ([-0.3, 0, 1.3], [0.25, 0.25, 0.25]),]
    
    random.seed(args.seed)
    np.random.seed(args.seed)
    if args.scene == 'car':
        assert args.planner == 'nonholrrt'
        start_pos = np.array([40, 100, 4.71]).reshape((3, 1))
        goal_pos = np.array([350, 150, 1.57]).reshape((3, 1))
        args.map = 'envs/car_map.txt'
        env = CarEnvironment(args.map, start_pos, goal_pos)
        env.init_visualizer()
    elif args.scene == '2dof_robot_arm':
        urdf_file = "urdf/2dof_planar_robot.urdf"
        start_pos = (0, 0)
        goal_pos = [0, 0, 2.0]
        env = ArmEnvironment(urdf_file, start_pos, goal_pos, obstacles, vis_plan=args.visualize)
    elif args.scene == '3dof_robot_arm':
        urdf_file = "urdf/3dof_planar_robot.urdf"
        start_pos = (0, 0, 0)
        goal_pos = [0, 0, 2.0]
        env = ArmEnvironment(urdf_file, start_pos, goal_pos, obstacles, vis_plan=args.visualize)
    # if args.visualize:
    #     env.visualize_c_space()
        
    if args.planner == "astar":
        from Astar import AstarPlanner
        # Instantiate the A* algorithm
        planner = AstarPlanner(env, args.epsilon)
    if args.planner.lower() == 'rrt':
        from RRTPlanner import RRTPlanner
        # Instantiate the RRT algorithm
        planner = RRTPlanner(env)
    if args.planner.lower() == 'rrtstar':
        from RRTStarPlanner import RRTStarPlanner
        # Instantiate the RRT algorithm
        planner = RRTStarPlanner(env)
    if args.planner.lower() == 'nonholrrt':
        from RRTPlannerNonholonomic import RRTPlannerNonholonomic
        # Instantiate the RRT algorithm
        planner = RRTPlannerNonholonomic(env)
    
    # Get the path from the planner
    
    start_time = time.time()
    path = planner.Plan(env.start, env.goal)
    end_time = time.time()
    
    print("Time cost: ", end_time-start_time)
    
    if args.scene == 'car':
        # Visualize the final path.
        tree = None
        visited = None
        if args.planner != 'astar':
            tree = planner.tree
        else:
            visited = planner.visited
        env.visualize_plan(path, tree, visited)
        plt.show()
    elif (args.scene == '2dof_robot_arm') or (args.scene == '3dof_robot_arm'):
        if (args.planner == 'astar') and (args.scene == '2dof_robot_arm'):
            planner.plot()
        else:
            if args.visualize:
                tree = None
                visited = None
                tree = planner.tree
                print(path, tree, visited)
                env.visualize_plan(path.T, tree, visited)
                plt.show()
        if path is not None:
            env.follow_path(path)
        else:
            print("you have to implement Plan() to compute the cost!")