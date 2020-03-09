#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseArray


from std_msgs.msg import Float32

from dynamic_reconfigure.msg import Config

#coefficients for subscribing
Kp = 0.7
Kd = 0.3

# rostopics to subscribe
goal_position = '/goal'
current_velocity = '/mavros/local_position/velocity_local'
current_position = '/mavros/local_position/pose'
target_velocity = '/mavros/setpoint_velocity/cmd_vel_unstamped'

dynamic_reconfigure = '/dynamic_tutorials/parameter_updates'
obstacles = '/obstacles'

def update_u():

    global goal_pos
    global cur_pos
    global goal_pos

    global tolerance_distance
    global constant_k
    global closest_obstacle
    global obstacle_positions
    ts.header.stamp = rospy.Time.now()


    repulsor_x = 1e-8
    repulsor_y = 1e-8


    # constant_k*(1/distance_closest - 1/tolerance_distance)
    closest_distance = euclidean_distance(cur_pos,closest_obstacle)
    if closest_distance<tolerance_distance:
        repulsor_x += constant_k*(1/closest_distance - 1/tolerance_distance)**2
        repulsor_y += constant_k*(1/closest_distance - 1/tolerance_distance)**2




    # calculating linear velocities pd + field
    u_x = Kp*(goal_pos.pose.position.x-cur_pos.pose.position.x) + Kd*(-cur_vel.twist.linear.x ) - repulsor_x
    u_y = Kp*(goal_pos.pose.position.y-cur_pos.pose.position.y) + Kd*(-cur_vel.twist.linear.y ) - repulsor_y
    u_z = Kp*(goal_pos.pose.position.z-cur_pos.pose.position.z) + Kd*(-cur_vel.twist.linear.z )



    # constraints on velocity in each direction
    if u_x > 2:
        u_x = 2
    elif u_x < -2:
        u_x = -2

    if u_y > 2:
        u_y = 2
    elif u_y < -2:
        u_y = -2

    if u_z > 1.5:
        u_z = 1.5
    elif u_z < -1.5:
        u_z = -1.5

    ts.twist.linear.x = u_x
    ts.twist.linear.y = u_y
    ts.twist.linear.z = u_z

    #angular part
    g_z_rot = trans_q_to_e(goal_pos)
    c_z_rot = trans_q_to_e(cur_pos)        
    ts.twist.angular.z = Kp*(g_z_rot - c_z_rot) + Kd*(0.0 - cur_vel.twist.angular.z)

def trans_q_to_e(obj):
    qx = obj.pose.orientation.x
    qy = obj.pose.orientation.y
    qz = obj.pose.orientation.z
    qw = obj.pose.orientation.w

    rotateZa0 = 2.0 * (qx * qy + qw * qz)
    rotateZa1 = qw * qw + qx * qx - qy * qy - qz * qz;
    rotateZ = 0.0;
    if rotateZa0 != 0.0 and rotateZa1 != 0.0:
        rotateZ = np.arctan2(rotateZa0, rotateZa1)
    return rotateZ

#just simple euclidean distance between 2 points
def euclidean_distance(drone_pos,obstacle):
    drone_pos = drone_pos.pose.position
    drone_pos = [drone_pos.x, drone_pos.y, drone_pos.z]

    dist = [(drone_pos[i]-obstacle[i])**2 for i in range(3)]

    overall_dist = np.sum(dist)**0.5
    return overall_dist

def calculate_repulsor(drone,obstacles):
    repulsor_x = [(drone[0]-obstacles[0])]

# finding x,y,z positions of closest obstacle to our drone
def find_closest(msg):
    global cur_pos
    global closest_obstacle
    global obstacle_positions
    obstacle_positions = list(msg.poses) #each pose is an array with positions in x,y,z and orientation x,y,z,w
    # writing positions of each obstacle into an array
    obstacle_positions = [[i.position.x,i.position.y,i.position.z] for i in obstacle_positions]
    # # finding distance between drone and each obstacle
    distances = [euclidean_distance(cur_pos, next_obstacle) for next_obstacle in obstacle_positions]
    closest = np.argmin(distances)
    closest_obstacle = obstacle_positions[closest] #position in x,y,z of closest obstacle

# dynamically updating with dynamic_reconfigure
def dynamic_updates(msg):
    global tolerance_distance
    global constant_k

    tolerance_distance = msg.doubles[0].value
    constant_k = msg.doubles[1].value

# function for checking some message with subscriber
def checking(msg):
    print(msg.pose)

def update_cur_pos(msg):
    global cur_pos
    cur_pos = msg


def update_cur_vel(msg):
    global cur_vel
    cur_vel = msg


def update_goal(msg):
    global goal_pos
    goal_pos = msg


if __name__=='__main__':
    rospy.init_node('talker')

    global cur_pos
    cur_pos = PoseStamped()

    global cur_vel
    cur_vel = TwistStamped()

    global goal_pos
    goal_pos = PoseStamped()

    global ts
    ts = TwistStamped()
    ts.header.frame_id = 'map'

    global obstacle_positions
    obstacle_positions = []



    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

    rate = rospy.Rate(50)
    rospy.Subscriber(goal_position, PoseStamped, update_goal)  # need to write function
    rospy.Subscriber(current_velocity, TwistStamped, update_cur_vel)  # need to write function
    rospy.Subscriber(current_position, PoseStamped, update_cur_pos)  # need to write function

    # dynamically updated parameters
    global tolerance_distance
    tolerance_distance = 10
    global constant_k
    constant_k = 5
    rospy.Subscriber(dynamic_reconfigure, Config, dynamic_updates)

    # subscribe to obstacles
    global closest_obstacle
    closest_obstacle = [3,3,3]
    rospy.Subscriber(obstacles, PoseArray, find_closest)



    while not rospy.is_shutdown():
        update_u()
        pub.publish(ts)
        rate.sleep()

