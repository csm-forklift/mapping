#!/usr/bin/env python


import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose


class PublishMap():
    def __init__(self):
        # Create node objects
        rospy.init_node("publish_map")
        self.map_pub = rospy.Publisher("/test_map", OccupancyGrid, queue_size=10)
        self.rate = rospy.Rate(10)

        # Create the occupancy grid
        self.res = 0.5 # m/cell
        self.width = 10 # m
        self.height = 10 # m
        self.width_cells = self.width/self.res # num cells
        self.height_cells = self.height/self.res # num cells
        self.map = OccupancyGrid()
        self.map.header.frame_id = "base_footprint"
        self.map.info.resolution = self.res
        self.map.info.width = self.width_cells
        self.map.info.height = self.height_cells
        self.map_pose = Pose()
        self.map_pose.position.x = -self.width/2.
        self.map_pose.position.y = -self.height/2.
        self.map_pose.position.z = 0
        self.map_pose.orientation.x = 0
        self.map_pose.orientation.y = 0
        self.map_pose.orientation.z = 0
        self.map_pose.orientation.w = 1
        self.map.info.origin = self.map_pose
        self.data = [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
            -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
            -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
            -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
            -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,
            -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,0,0,
            -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,0,0,0,0,
            -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,0,0,0,0,0,0,
            -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,0,0,0,0,0,0,0,0,
            -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,0,100,100,0,0,0,0,0,0,
            -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,0,100,100,0,0,0,0,0,0,
            -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,100,100,0,0,0,0,0,0,
            -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,0,0,0,0,0,0,
            -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,0,0,0,0,
            -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,0,0,
            -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,
            -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
            -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
            -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
            -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]

        self.map.data = self.data

        # Run loop
        self.loop()

    def loop(self):
        while not rospy.is_shutdown():
            # Publish the map
            self.map.header.stamp = rospy.Time.now()
            self.map_pub.publish(self.map)

            self.rate.sleep()

def main():
    publish_map = PublishMap()


main()
