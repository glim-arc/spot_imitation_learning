from __future__ import division
import numpy as np
import math
import rospy
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
import time

class RRT_Node:
    def __init__(self, conf):
        self.data = conf #x,y conf
        self.parent = None
        self.children = []

    def set_parent(self, parent):
        self.parent = parent

    def add_child(self, child):
        self.children.append(child)

def sample_conf(goal_conf):
    x = np.random.uniform(-15, 15)
    y = np.random.uniform(-15, 15)
    
    rand_conf = [x, y]
    flag = (goal_conf[0] == x and goal_conf[1] == y)

    return rand_conf, flag
   
def find_nearest(rand_node, node_list):
    min = float('inf')
    nearest_node = RRT_Node([0, 0])

    for node in node_list:

        dist = 0
        for i in range(2):
            dist += (node.data[i]-rand_node.data[i])**2

        if (dist < min):
            min = dist
            nearest_node = node

    return nearest_node

def CollisionCheck(node):
        """
        Checks whether a given configuration is valid. (collides with obstacles)
        """
        dof = 2
        obstacleList = [(-12.75, 6.58, 5.4, 7.2), 
                        (-5.04, 2.1, 0.25, 0.25), 
                        (-5.1, -0.05, 0.25, 0.25), 
                        (-5.02, 5.7, 0.25, 0.25), 
                        (-5.02, 8.02, 0.25, 0.25), 
                        (8.41, 8.82, 0.25, 0.25), 
                        (1.99, -5.88, 0.25, 0.25),
                        (-0.21, -11, 0.4, 0.4),
                        (-5.01, 1.46, 0.4, 0.4),
                        (4.58, 4.13, 0.6, 0.6),
                        (6.09, 1, 0.5, 0.5),
                        (2.84, 1.03, 0.5, 0.5),
                        (2.4, -2.91, 0.5, 0.5),
                        (5.96, -2.99, 0.5, 0.5),
                        (-5.08, -15.38, 10, 7)]
        
        s = np.zeros(10, dtype=np.float32)
        s[0] = node[0]
        s[1] = node[0] + 0.281621
        s[2] = node[0] - 0.281621
        s[3] = s[2]
        s[4] = s[1]
        s[5] = node[1] 
        s[6] = node[1] + 0.281621
        s[7] = node[1] - 0.281621
        s[8] = s[6]
        s[9] = s[7]

        for (ox, oy, sizex,sizey) in obstacleList:
            counter = 0
            for j in range(dof + 3):
                if (s[j] > ox and s[j] < ox+sizex): #check left and righ side of circle
                    counter += 1

                if (s[j+5] > oy and s[j+5] < oy+sizey): #check up and lower side of circle
                    counter += 1
                
                if counter == 2:
                    return False
                
                counter = 0

        return True  # safe'''

def steer_to(rand_node, nearest_node):
    dist = 0
    stepsize = 0.05
    for i in range(2):
            dist += (nearest_node.data[i]-rand_node.data[i])**2

    steps = round(dist / stepsize)
    if steps == 0:
        steps = 1
    delta = [0,0,0]
    for i in range(2):
            delta[i] = (rand_node.data[i]-nearest_node.data[i]) / steps
    
    start = [nearest_node.data[0], nearest_node.data[1]]
    for i in range(1, steps+1):
        for j in range(2):
            start[j] += delta[j]
        if not CollisionCheck(start):
            return False
    return True


def insert_node_bfs(rand_node, nearest_node, tree):

    visited = []
    queue = []

    queue.append(tree)
    visited.append(tree)
    if (tree.data == nearest_node.data):
        nearest_node.add_child(rand_node)
        rand_node.set_parent(nearest_node)  
        return
    
    
    while queue:
        nodeP = queue.pop(0)
        for nodeC in nodeP.children:
            if nodeC not in visited:
                if (nodeC.data == nearest_node.data):
                    nearest_node.add_child(rand_node)
                    rand_node.set_parent(nearest_node)  
                    return
                queue.append(nodeC)
                visited.append(nodeC)

    return



def RRT(start_pose, end_pose):
    tree = RRT_Node(start_pose)
    node_list = []
    node_list.append(tree)

    path_not_found = True
    
    while path_not_found:
        rand_conf, flag = sample_conf(end_pose)
        q_rand = RRT_Node(rand_conf)
            
        q_nearest = find_nearest(q_rand, node_list)


        if steer_to(q_rand, q_nearest):
            insert_node_bfs(q_rand, q_nearest, tree)
            node_list.append(q_rand)

            if flag:
                break

            dist_from_goal = 0
            for i in range(2):
                dist_from_goal += (end_pose[i]-q_rand.data[i])**2

            if (math.sqrt(dist_from_goal) < 0.3):
                break


    
    backtrace_node = q_rand
    path = []

    while backtrace_node:
        path.append(backtrace_node.data)
        backtrace_node = backtrace_node.parent
        
    path.reverse()

    return path


###############################################################################
#your implementation ends here

if __name__ == "__main__":

    # start and goal
    start_pose = (0, 0)
    end_pose = (8,-4)
    
	# place holder to save the solution path
    path_conf = None


    path_conf = RRT(start_pose, end_pose)
    #assert all(steer_to(RRT_Node(path_conf[i]), RRT_Node(path_conf[i+1])) for i in range(len(path_conf) - 1))

    if path_conf is None:
        # pause here
        input("no collision-free path is found within the time budget, finish?")
    else:
        # execute the path
        for q in path_conf:
            robotpose_msg = ModelState()
            robotpose_msg.model_name = 'cricket_ball'
            robotpose_msg.pose.position.x = q[0]
            robotpose_msg.pose.position.y = q[1]
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            set_state(robotpose_msg)
            time.sleep(1)