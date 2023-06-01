#!/usr/bin/env python3

import rospy
from math import sqrt
import numpy as np
import heapq
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from math import *

grid_viz=None
previous_plan_variables=None

path_pub = rospy.Publisher('/path', Path, queue_size=1)

def euclidean_distance(index, goal_index, width):
  print('Calculating Euclidean Distance')  
  index_x = index % width
  index_y = int(index / width)
  goal_x = goal_index % width
  goal_y = int(goal_index / width)

  distance = (index_x - goal_x) ** 2 + (index_y - goal_y) ** 2
  return sqrt(distance)

def map_callback(msg):
  print('Receiving Map')
  global costmap, width, height, resolution, origin, grid_viz

  costmap = msg.data
  width = msg.info.width
  height = msg.info.height
  resolution = msg.info.resolution
  origin = msg.info.origin.position

#   print('width',width)
#   print('height',height)
#   print('resolution',resolution)
#   print('origin',origin)

def goal_callback(msg):
  print('Received the Goal Point')
  global costmap, width, height, resolution, origin, previous_plan_variables

  if costmap is None:
    rospy.logwarn("Costmap is not yet available. Skipping goal processing.")
    return

  start_index=(0,0)
  start_index = int((0 - origin.y) / resolution) * width + int((0 - origin.x) / resolution)
  goal_pose = msg
  goal_pose.header.stamp = rospy.Time.now()
  goal_index = int((goal_pose.pose.position.y - origin.y) / resolution) * width + int((goal_pose.pose.position.x - origin.x) / resolution)
  path, previous_plan_variables = astar(start_index, goal_index, width, height, costmap, resolution, origin, None, previous_plan_variables)

  path_msg = Path()
  path_msg.header.stamp = rospy.Time.now()
  path_msg.header.frame_id = "map"
  for index in path:
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = (index % width) * resolution + origin.x
    pose.pose.position.y = int(index / width) * resolution + origin.y
    pose.pose.orientation.w = 1
    path_msg.poses.append(pose)
  path_pub.publish(path_msg)
  print('Planning Done')

def find_neighbors(index, width, height, costmap, orthogonal_step_cost):
  
  print('Calculating Neighbours')  
  neighbors = []
  diagonal_step_cost = orthogonal_step_cost * 1.41421
  lethal_cost = 150

  upper = index - width
  if upper > 0:
    if costmap[upper] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[upper]/255
      neighbors.append([upper, step_cost])

  left = index - 1
  if left % width > 0:
    if costmap[left] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[left]/255
      neighbors.append([left, step_cost])

  upper_left = index - width - 1
  if upper_left > 0 and upper_left % width > 0:
    if costmap[upper_left] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[upper_left]/255
      neighbors.append([index - width - 1, step_cost])

  upper_right = index - width + 1
  if upper_right > 0 and (upper_right) % width != (width - 1):
    if costmap[upper_right] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[upper_right]/255
      neighbors.append([upper_right, step_cost])

  right = index + 1
  if right % width != (width + 1):
    if costmap[right] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[right]/255
      neighbors.append([right, step_cost])

  lower_left = index + width - 1
  if lower_left < height * width and lower_left % width != 0:
    if costmap[lower_left] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[lower_left]/255
      neighbors.append([lower_left, step_cost])

  lower = index + width
  if lower <= height * width:
    if costmap[lower] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[lower]/255
      neighbors.append([lower, step_cost])

  lower_right = index + width + 1
  if (lower_right) <= height * width and lower_right % width != (width - 1):
    if costmap[lower_right] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[lower_right]/255
      neighbors.append([lower_right, step_cost])

  return neighbors

def astar(start_index, goal_index, width, height, costmap, resolution, origin, grid_viz, previous_plan_variables):

  open_list = []
  closed_list = set()
  parents = dict()
  g_costs = dict()
  f_costs = dict()
  g_costs[start_index] = 0
  f_costs[start_index] = 0
  start_cost = 0 + euclidean_distance(start_index, goal_index, width)
  open_list.append([start_index, start_cost])
  shortest_path = []
  path_found = False

  while open_list:
    open_list.sort(key = lambda x: x[1])   
    current_node = open_list.pop(0)[0]
    closed_list.add(current_node)
    if current_node == goal_index:
      path_found = True
      print('Path Found')
      break    
    neighbors = find_neighbors(current_node, width, height, costmap, resolution)
    for neighbor_index, step_cost in neighbors:
      if neighbor_index in closed_list:
        continue

      g_cost = g_costs[current_node] + step_cost
      h_cost = euclidean_distance(neighbor_index, goal_index, width)
      f_cost = g_cost + h_cost

      in_open_list = False
      for idx, element in enumerate(open_list):
        if element[0] == neighbor_index:
          in_open_list = True
          break

      if in_open_list:
        if f_cost < f_costs[neighbor_index]:
          g_costs[neighbor_index] = g_cost
          f_costs[neighbor_index] = f_cost
          parents[neighbor_index] = current_node
          open_list[idx] = [neighbor_index, f_cost]
      else:
        g_costs[neighbor_index] = g_cost
        f_costs[neighbor_index] = f_cost
        parents[neighbor_index] = current_node
        open_list.append([neighbor_index, f_cost])
  print('Done traversing nodes in open_list')

  if not path_found:
    rospy.logwarn('No path found!')
    return shortest_path

  if path_found:
      node = goal_index
      shortest_path.append(goal_index)
      while node != start_index:
          shortest_path.append(node)
          node = parents[node]
  shortest_path = shortest_path[::-1]
  print('Done reconstructing path')
  return shortest_path, None

def call():
    rospy.init_node('a_star_planner')
    print('Started planning Node')
    rospy.Subscriber('/map', OccupancyGrid, map_callback, queue_size=1)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':  
    try:
        call()
    except rospy.ROSInterruptException:
        pass
