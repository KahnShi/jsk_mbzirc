#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)

# Copyright (c) 2016, JSK Robotics Laboratory, The University of Tokyo
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
import rospy
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math

__author__ = 'shifan@jsk.imi.i.u-tokyo.ac.jp (Shi Fan)'

class LaneVisualize:
    def init(self):
        self.__lane_path_ins = Path()
        self.__lane_path_out = Path()
        self.__lane_ins_pub = rospy.Publisher("/lane_path_ins", Path, queue_size=1)
        self.__lane_out_pub = rospy.Publisher("/lane_path_out", Path, queue_size=1)
        self.lane_data_init()

    def lane_data_init(self):
        circle_radius = 20.0
        circle_distance = 56.56
        road_broadth = 3.0
        circle_points_number = 54
        # todo: header
        self.__lane_path_out.header.stamp = rospy.Time.now()
        self.__lane_path_out.header.frame_id = "world"
        out_start_pose = PoseStamped()
        out_start_pose.header = self.__lane_path_out.header
        out_start_pose.pose.position.x = road_broadth/2.0*math.cos(math.pi/4.0)
        out_start_pose.pose.position.y = road_broadth/2.0*math.sin(math.pi/4.0)
        self.__lane_path_out.poses.append(out_start_pose)

        self.__lane_path_ins.header = self.__lane_path_out.header
        ins_start_pose = PoseStamped()
        ins_start_pose.header = self.__lane_path_ins.header
        ins_start_pose.pose.position.x = -road_broadth/2.0*math.cos(math.pi/4.0)
        ins_start_pose.pose.position.y = -road_broadth/2.0*math.sin(math.pi/4.0)
        self.__lane_path_ins.poses.append(ins_start_pose)
        ## right down corner
        out_r_d_pose = PoseStamped()
        out_r_d_pose.header = out_start_pose.header
        out_r_d_pose.pose.position.x = out_start_pose.pose.position.x + circle_radius*math.cos(math.pi/4.0)
        out_r_d_pose.pose.position.y = out_start_pose.pose.position.y - circle_radius*math.sin(math.pi/4.0)
        self.__lane_path_out.poses.append(out_r_d_pose)

        ins_r_d_pose = PoseStamped()
        ins_r_d_pose.header = ins_start_pose.header
        ins_r_d_pose.pose.position.x = ins_start_pose.pose.position.x + circle_radius*math.cos(math.pi/4.0)
        ins_r_d_pose.pose.position.y = ins_start_pose.pose.position.y - circle_radius*math.sin(math.pi/4.0)
        self.__lane_path_ins.poses.append(ins_r_d_pose)

        ## right up corner
        r_circle_center_x = circle_distance/2.0
        avg_ang = 270.0/circle_points_number
        for i in range (0,circle_points_number):
            cur_ang = math.pi*(avg_ang*i-135)/180.0
            out_r_u_pose = PoseStamped()
            out_r_u_pose.header = out_start_pose.header
            out_r_u_pose.pose.position.x = r_circle_center_x + (circle_radius-road_broadth/2.0)*math.cos(cur_ang)
            out_r_u_pose.pose.position.y = (circle_radius-road_broadth/2.0)*math.sin(cur_ang)
            self.__lane_path_out.poses.append(out_r_u_pose)

            ins_r_u_pose = PoseStamped()
            ins_r_u_pose.header = ins_start_pose.header
            ins_r_u_pose.pose.position.x = r_circle_center_x + (circle_radius+road_broadth/2.0)*math.cos(cur_ang)
            ins_r_u_pose.pose.position.y = (circle_radius+road_broadth/2.0)*math.sin(cur_ang)
            self.__lane_path_ins.poses.append(ins_r_u_pose)

        ## left down corner
        l_circle_center_x = -circle_distance/2.0
        avg_ang = 270.0/circle_points_number
        for i in range (0,circle_points_number):
            cur_ang = math.pi*(-avg_ang*i-45)/180.0
            out_l_d_pose = PoseStamped()
            out_l_d_pose.header = out_start_pose.header
            out_l_d_pose.pose.position.x = l_circle_center_x + (circle_radius+road_broadth/2.0)*math.cos(cur_ang)
            out_l_d_pose.pose.position.y = (circle_radius+road_broadth/2.0)*math.sin(cur_ang)
            self.__lane_path_out.poses.append(out_l_d_pose)

            ins_l_d_pose = PoseStamped()
            ins_l_d_pose.header = ins_start_pose.header
            ins_l_d_pose.pose.position.x = l_circle_center_x + (circle_radius-road_broadth/2.0)*math.cos(cur_ang)
            ins_l_d_pose.pose.position.y = (circle_radius-road_broadth/2.0)*math.sin(cur_ang)
            self.__lane_path_ins.poses.append(ins_l_d_pose)

        ## mid point
        self.__lane_path_out.poses.append(out_start_pose)
        self.__lane_path_ins.poses.append(ins_start_pose)


    def pub_lane(self):
        self.__lane_ins_pub.publish(self.__lane_path_ins)
        self.__lane_out_pub.publish(self.__lane_path_out)

if __name__ == '__main__':
    rospy.init_node('truck_stop_flag', anonymous=True)
    rate = rospy.Rate(0.5)
    my_lane = LaneVisualize()
    my_lane.init()
    cnt = 0
    while not rospy.is_shutdown():
        cnt += 1
        if cnt > 10:
            break
        my_lane.pub_lane()
        rate.sleep()
