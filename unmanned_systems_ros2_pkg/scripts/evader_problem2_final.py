#!/usr/bin/env python3
import rclpy
import math as m
from Dijkstras import Dijkstras, Obstacle
from random import randint
from rclpy.node import Node
from unmanned_systems_ros2_pkg import TurtleBotNode, quaternion_tools


def generate_random_waypoints(n_random_waypoints:int, max_val:int)->list:
    """generate random waypoints from 1 to 1"""
    
    random_wp_list = []
    for i in range(0,n_random_waypoints+1):
        rand_x = randint(0, max_val)
        rand_y = randint(0, max_val)
        random_wp_list.append((rand_x, rand_y))
        
    return random_wp_list

def compute_desired_heading(current_pos:list, des_pos:list) -> float:
    """compute desired heading based on positions"""
    return m.atan2(des_pos[1] - current_pos[1] , des_pos[0] - current_pos[0])

def compute_dist_error(current_pos:list, des_pos:list)->float:
    """compute distance error"""
    return m.dist(des_pos,current_pos)

def compute_heading_error(current_heading:float, des_heading:float) -> float:
    """compute heading error in radians"""
    return des_heading - current_heading

def gimme_da_loot(turtlebot:TurtleBotNode, waypoint:list) -> list:
    """helper function"""
    desired_heading = compute_desired_heading(
        turtlebot.current_position, waypoint)
    
    heading_error = compute_heading_error(
        turtlebot.orientation_euler[2], desired_heading)

    dist_error = compute_dist_error(
        turtlebot.current_position, waypoint)
    
    return [desired_heading, heading_error, dist_error]


def main() -> None:
    rclpy.init(args=None)
    
    turtlebot_evader = TurtleBotNode.TurtleBotNode('turtle', 'evader')    
    turtlebot_evader.current_position = [2.0, 1.0]
    turtlebot_evader.move_turtle(0.0,0.0)

    set_random = True
    is_done = False
    n_random_waypoints =  3
    heading_tol = 0.1; #radians
    dist_tolerance = 0.25 #meters
    
    turn_speed = 0.1 #rad/speed
    line_speed = 0.1 #m/s
    stop_speed = 0.0 

    Start_x = 2
    Start_y = 1

    Goal_x = 7
    Goal_y = 2

    obs_list = [Obstacle(each[0], each[1], radius=0.25) for each in [(5, 0), (5, 1), (5, 2), (5, 3), (5, 4), (0, 5), (1, 4), (2, 3), (3, 2), (3, 3)]]

    dijkstras = Dijkstras(0, 0, 15, 15, 1)
    route = dijkstras.run(start=(Start_x, Start_y), goal=(Goal_x, Goal_y), r_radius=0.1, obs_list=obs_list)
    waypoints = [[each[0], each[1]] for each in route]
    print(waypoints)
    dijkstras.plot_route(route=route)
    while rclpy.ok():

        if is_done == True:
            print("I'm done")
            turtlebot_evader.move_turtle(stop_speed, stop_speed)
            rclpy.shutdown()

        for waypoint in waypoints:
            print("current waypoint is", waypoint)
            
            desired_heading, heading_error, dist_error = gimme_da_loot(turtlebot_evader, waypoint)

            while (abs(dist_error) >= dist_tolerance) or (abs(heading_error) >= heading_tol):
        
                if abs(dist_error) >= dist_tolerance and  abs(heading_error) <= heading_tol:
                    turtlebot_evader.move_turtle(line_speed, stop_speed)
                elif abs(dist_error) < dist_tolerance and  abs(heading_error) >= heading_tol:
                    turtlebot_evader.move_turtle(stop_speed, turn_speed)
                else:
                    turtlebot_evader.move_turtle(line_speed, turn_speed)
                
                desired_heading, heading_error, dist_error = gimme_da_loot(turtlebot_evader, waypoint)
                
                rclpy.spin_once(turtlebot_evader)
                                
        is_done = True
                        

if __name__=="__main__":
    main()
