#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionFeedback
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point

import tf
import math
import numpy as np
import scipy.ndimage
import scipy.misc
#left_top = PoseStamped()
#targets = {0: (6.16306495,3.5174586), 1:(2.68351054,-3.2659452), 2:(2.67493558,5.02056122), 3:(-0.144605,1.593281)}
#targets = {1: (-0.989295840263,1.9834952344)}
#targets = {1: (2.0,2.0)}
#targets = {1: (-0.8,-2), 0: (-0.19,0.74)}
class navigation():

    def __init__(self, max_distance=2):
        self._map = None;
	self._map_info = None;
        self._inflated_map = None;
        self._tree = []
        self._max_dis = max_distance
        self._preprocessing = False
        self._map_load = rospy.Subscriber("/map",OccupancyGrid, self.load_map);
        self._target   = rospy.Subscriber("/human_location",Point, self.find_position);
        self._pub      = rospy.Publisher("/human_obstacle",Point,queue_size=10);
        self.test()
    def load_map(self,loaded_map):
	self._map = loaded_map.data;
	self._map_info = loaded_map.info;
        self.load_to_array()
        self.build_tree()
        self._preprocessing = True
    def load_to_array(self):
        height = self._map_info.height
        width = self._map_info.width
        #inflated_map = np.array(self._map,dtype=np.int8).reshape(height,width)
        #self._inflated_map = np.array((height,width), dtype = np.int8)
        #self._inflated_map = np.where(self._map>10)
        self._inflated_map = [True if self._map[i]>10 else False for i in range(len(self._map))]
        self._inflated_map = np.array(self._inflated_map).reshape(height,width)
        self._inflated_map = scipy.ndimage.morphology.binary_dilation(self._inflated_map)
        #scipy.misc.toimage(self._inflated_map,cmin=0.0,cmax=1.0).save('out.jpg')
    def build_tree(self):
        # this method builds a tree to find the closest point with distance 
        # less than max distance
        # the tree is a list of tuples
        # each tuple is as such ((i,j),distance,child), where (i,j) are the 
        # differences in row and col respecitvely between the center node
        max_pixels = int(self._max_dis/self._map_info.resolution)
        visited = np.ones((2*max_pixels+1,2*max_pixels+1))
        self._tree.append([(0,0),0,[1,2,3,4,5,6,7,8]])
        current_layer = [1,2,3,4,5,6,7,8]
        angle_dis = math.sqrt(2)*self._map_info.resolution
        linea_dis = 1*self._map_info.resolution
        print (max_pixels,angle_dis,linea_dis)
        self._tree.append([( 0, 1),linea_dis,[]])
        self._tree.append([( 1, 0),linea_dis,[]])
        self._tree.append([( 1, 1),angle_dis,[]])
        self._tree.append([( 0,-1),linea_dis,[]])
        self._tree.append([(-1, 0),linea_dis,[]])
        self._tree.append([(-1,-1),angle_dis,[]])
        self._tree.append([(-1, 1),angle_dis,[]])
        self._tree.append([( 1,-1),angle_dis,[]])
        for t in self._tree:
            i,j = t[0]
            visited[max_pixels+i,max_pixels+j] = 0
        for k in range(max_pixels-1):
            next_layer = []
            for i  in current_layer:
                l = self._tree[i]
                index = l[0]
                if index[0] == 0:
                    temp_index = (0,index[1]+1 if index[1]>0 else index[1]-1)
                    if visited[max_pixels+temp_index[0],max_pixels+temp_index[1]] == 1:
                        visited[max_pixels+temp_index[0],max_pixels+temp_index[1]] = 0
                        distance = self._tree[i][1]
                        distance = distance+linea_dis
                        if distance <= self._max_dis:
                            self._tree[i][2].append(len(self._tree))
                            next_layer.append(len(self._tree))
                            self._tree.append([temp_index,distance,[]])

                elif index[1] == 0:
                    temp_index = (index[0]+1 if index[0]>0 else index[0]-1,0)
                    if visited[max_pixels+temp_index[0],max_pixels+temp_index[1]] == 1:
                        visited[max_pixels+temp_index[0],max_pixels+temp_index[1]] = 0
                        distance = self._tree[i][1]
                        distance = distance+linea_dis
                        if distance <= self._max_dis:
                            self._tree[i][2].append(len(self._tree))
                            next_layer.append(len(self._tree))
                            self._tree.append([temp_index,distance,[]])
                else:
                    temp_index = (index[0],index[1]+1 if index[1]>0 else index[1]-1)
                    if visited[max_pixels+temp_index[0],max_pixels+temp_index[1]] == 1:
                        visited[max_pixels+temp_index[0],max_pixels+temp_index[1]] = 0
                        distance = self._tree[i][1]
                        distance = distance+linea_dis
                        if distance <= self._max_dis:
                            self._tree[i][2].append(len(self._tree))
                            next_layer.append(len(self._tree))
                            self._tree.append([temp_index,distance,[]])
                    temp_index = (index[0]+1 if index[0]>0 else index[0]-1,index[1])
                    if visited[max_pixels+temp_index[0],max_pixels+temp_index[1]] == 1:
                        visited[max_pixels+temp_index[0],max_pixels+temp_index[1]] = 0
                        distance = self._tree[i][1]
                        distance = distance+linea_dis
                        if distance <= self._max_dis:
                            self._tree[i][2].append(len(self._tree))
                            next_layer.append(len(self._tree))
                            self._tree.append([temp_index,distance,[]])
                    temp_index = (index[0]+1 if index[0]>0 else index[0]-1,index[1]+1 if index[1]>0 else index[1]-1)
                    if visited[max_pixels+temp_index[0],max_pixels+temp_index[1]] == 1:
                        visited[max_pixels+temp_index[0],max_pixels+temp_index[1]] = 0
                        distance = self._tree[i][1]
                        distance = distance+angle_dis
                        if distance <= self._max_dis:
                            self._tree[i][2].append(len(self._tree))
                            next_layer.append(len(self._tree))
                            self._tree.append([temp_index,distance,[]])
            current_layer = next_layer
    def test(self):
        if self._preprocessing:
            for target in targets.keys():
                x = targets[target][0]
                y = targets[target][1]
                grid_x = x-self._map_info.origin.position.x
                grid_x = grid_x/self._map_info.resolution
                grid_y = y-self._map_info.origin.position.y
                grid_y = grid_y/self._map_info.resolution
                row = int(math.floor(grid_y))
                col = int(math.floor(grid_x))
                dist,loca = self.find_shortest((row,col))
                print (dist,loca)
            return True
    def find_position(self,loc):
        p = Point()

        if self._preprocessing == True:
            print (loc)
            grid_x = loc.x-self._map_info.origin.position.x
            grid_x = grid_x/self._map_info.resolution
            grid_y = loc.y-self._map_info.origin.position.y
            grid_y = grid_y/self._map_info.resolution
            row = int(math.floor(grid_y))
            col = int(math.floor(grid_x))
            dist,loca = self.find_shortest((row,col))
            p.x = loca[0]
            p.y = loca[1]
            p.z = dist
            self._pub.publish(p)
            return dist,loca
        p.x = 0
        p.y = 0
        p.z = -1
        self._pub.publish(p)
        return -1,(0,0)
    def find_shortest(self, loc):
        indeces = []
        height = self._map_info.height
        width = self._map_info.width
        for k in self._tree[0][2]:
            indeces.append(k)
        while(len(indeces)>0):
            index = indeces.pop(0)
            temp_index_local = self._tree[index][2]
            for k in temp_index_local:
                relative_location = self._tree[k][0]
                location = (loc[0]+relative_location[0],loc[1]+relative_location[1])
                if location[0]>=0 and location[0]<height and location[1]>=0 and location[1]<width:
                    if self._inflated_map[location[0],location[1]]:
                        return self._tree[k][1],self._tree[k][0]
                    else:
                        indeces.append(k)
        return -1,(0,0)



        
def main():
    rospy.init_node('ray_cast_node', anonymous=True)
    rate = rospy.Rate(1)
    nav = navigation()
    tested = True
    while not rospy.is_shutdown():
        if not tested:
            tested = nav.test()
        rospy.sleep(1)
        
if __name__ == "__main__":
    main()

