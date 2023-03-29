#!/usr/bin/env python
from math import pow, atan2, sqrt, pi, asin, cos
import numpy as np
from tf.transformations import euler_from_quaternion

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from std_msgs.msg import Bool, Int32MultiArray, Float32
from nav_msgs.msg import Odometry

DELAY_TIME = 0.005

THRESHOLD_YAW_RADIANS = (1.5)*pi/180    # when turning to goal, this is the tolerance +/- in radians 
DISTANCE_TOLERANCE = 0.01   # robot will travel to this value in metres around the goal position

MAX_LINEAR_VEL_X = 0.25     # maximum linear x velocity to use in m/s
MAX_ANGULAR_VEL_Z = 0.25    # maximum angular z velocity to use in rad/s
MIN_ANGULAR_VEL_Z = 0.00    # minimum angular z velocity to use in rad/s

VEL_PUBLISH_RATE = 20    # 5Hz velocity message publish rate
LED_PUBLISH_RATE = 3    # 3Hz LED message publish rate
TEST_PUBLISH_RATE = 1    # 1Hz test messages publish rate
QUEUE_SIZE = 10

BALL_RADIUS = 0.096 # m (actual ball radius measured by meter rule)

# Camera Parameters
HORIZONTAL_FOV = 68 # (degrees) for D435, 68 is measured, diff from spec: https://www.intel.com/content/www/us/en/support/articles/000030385/emerging-technologies/intel-realsense-technology.html
VERTICAL_FOV = 57
HORIZONTAL_RESOLUTION = 1280 # px
VERTICAL_RESOLUTION = 720 # px

KP_ANG = 0.8

#Possible SmartCart STATES:
STATE_AT_GOAL = 0
STATE_GET_NEXT_GOAL = 1
STATE_TURN_TO_GOAL = 2
STATE_DRIVE_TO_GOAL = 3
STATE_COMPLETE = 4

class SmartCart:
    def __init__(self):
        self.goal_pose = Pose(Point(0.0,0.0,0.0), Quaternion(0.0,0.0,0.0,1.0))
        self.current_pose = Pose(Point(0.0,0.0,0.0), Quaternion(0.0,0.0,0.0,1.0))
        self.starting_pose = Pose(Point(0.0,0.0,0.0), Quaternion(0.0,0.0,0.0,1.0))
        self.startingDistance = 0.0 # Set current distance between current_pose and goal_pose

        self.vel = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0))

        self.goalIndex = 0
        self.currentYaw = 0.0   # robot's current angle in radians relative to odom frame's positive x-axis; CCW is positive
        self.deltaYaw_unfiltered = 0.0
        self.deltaYaw = 0.0 # difference between goalYaw and currentYaw

        self.LED = Bool()
        self.LED.data = 0

        self.target_pos = Int32MultiArray(data=[-1,-1])
        self.target_dist = float(-1.0)
        self.target_radius = float(-1.0)

        self.ball_radius = 0.1

        self.waypoints = []

        rospy.init_node('preset_path', anonymous=True)

        # NEEDED PUBLISHERS & SUBSCRIBERS
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = QUEUE_SIZE)
        self.vel_rate = rospy.Rate(VEL_PUBLISH_RATE)

        self.LED_pub = rospy.Publisher('LEDsignal', Bool, queue_size = QUEUE_SIZE)
        self.LED_rate = rospy.Rate(LED_PUBLISH_RATE)

        self.target_pose_pub = rospy.Publisher('target_pose', Pose, queue_size = QUEUE_SIZE)

        
        # subscriber that subscribes to the "Odom" topic and calls the function "odomProcess"
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomProcess)

        self.target_pos_sub = rospy.Subscriber('target_position', Int32MultiArray, self.target_pos_process)
        self.target_dist_sub = rospy.Subscriber('target_distance', Float32, self.target_dist_process)
        self.target_radius_sub = rospy.Subscriber('ball_radius', Float32, self.target_radius_process)

        self.state = STATE_AT_GOAL # Set state so that Initially, we get next goal from user
        print("SmartCart Initialized")


    #Publishing Functions
    def set_vel(self, forward, turn):
        self.vel.linear.x = forward
        
        if turn >= MAX_ANGULAR_VEL_Z:
            self.vel.angular.z = MAX_ANGULAR_VEL_Z
        elif 0 <= turn <= MIN_ANGULAR_VEL_Z:
            self.vel.angular.z = MIN_ANGULAR_VEL_Z
        
        elif turn < (-1 * MAX_ANGULAR_VEL_Z):
            self.vel.angular.z = -1 * MAX_ANGULAR_VEL_Z
        elif (-1 * MIN_ANGULAR_VEL_Z) <= turn < 0:
            self.vel.angular.z = -1 * MIN_ANGULAR_VEL_Z
        
        else:
            self.vel.angular.z = turn

        self.vel_pub.publish(self.vel)
        self.vel_rate.sleep()

    def set_LED(self, LED_state):
        self.LED.data = LED_state
        self.LED_pub.publish(self.LED)
        
    #Helper Functions
    def odomProcess(self, odomData):
        self.current_pose.position.x = odomData.pose.pose.position.x
        self.current_pose.position.y = odomData.pose.pose.position.y
        self.currentYaw = euler_from_quaternion([odomData.pose.pose.orientation.x, odomData.pose.pose.orientation.y, odomData.pose.pose.orientation.z, odomData.pose.pose.orientation.w])[2]

    def target_pos_process(self, posData):
        self.target_pos = posData.data

    def target_dist_process(self, distData):
        self.target_dist = distData.data / 1000.0

    def target_radius_process(self, radData):
        self.target_radius = radData.data

    def euclidean_distance(self):
        return sqrt( pow((self.goal_pose.position.x - self.current_pose.position.x), 2) + pow((self.goal_pose.position.y - self.current_pose.position.y), 2) )
    
    def distance_travelled(self):
        return sqrt( pow((self.starting_pose.position.x - self.current_pose.position.x), 2) + pow((self.starting_pose.position.y - self.current_pose.position.y), 2) )
    
    def get_deltaYaw(self):
        # atan2(y, x) returns value of atan(y/x) in radians. The atan2() method returns a numeric value between -pi and pi representing the angle theta of a (x, y) point and positive x-axis.
        self.deltaYaw_unfiltered = atan2(self.goal_pose.position.y - self.current_pose.position.y, self.goal_pose.position.x - self.current_pose.position.x) - self.currentYaw

        if self.deltaYaw_unfiltered > pi:
            return (self.deltaYaw_unfiltered - 2*pi)
        elif self.deltaYaw_unfiltered < (-1*pi):
            return (2*pi + self.deltaYaw_unfiltered)
        else:
            return self.deltaYaw_unfiltered
    
    def locate_next_waypoint(self, verbose=True):
        if self.target_dist < 0.01 or self.target_dist > 10 or np.isnan(self.target_dist):
            # If target_dist is a "garbage value", return (-1,-1,-1)
            return Pose(position = Point(x = -1, y = -1, z = -1), orientation = Quaternion(w=1))
        else:
            # Convert position of ball to 3D position in camera frame
            # robot frame: x is forward, y is left, z is up, d is straight line to ball center (origin camera)
            # camera frame: x is right, y is down (origin is top left)
            depth = self.target_dist + BALL_RADIUS # Euclidean depth to ball center in m
            x_cam = self.target_pos[0] - HORIZONTAL_RESOLUTION / 2 # Horizontal pixels from center of camera
            y_cam = self.target_pos[1] - VERTICAL_RESOLUTION / 2 # Vertical pixels from center of camera

            # New code assuming the camera corrects for depth to be x
            # Construct right triangle with sides x_px, y_px, d_px in robot frame
            x_px = (HORIZONTAL_RESOLUTION/2) / np.tan(np.deg2rad(HORIZONTAL_FOV)/2)
            y_px = x_cam

            # Now we have a pixel scale
            pixel_scale = depth / x_px # m/px

            # Now we can convert the pixel values to meters
            y = -1 * x_cam * pixel_scale
            x = depth

            if verbose:
                print('x: {:.3f} y: {:.3f} d_calc: {:.3f}, cam_x: {:.1f}, cam_y: {:.1f}'.format(x, y, sqrt(pow(x, 2) + pow(y, 2)), self.target_pos[0], self.target_pos[1]))
            waypoint = Pose(position = Point(x = self.current_pose.position.x + x, y = self.current_pose.position.y + y, z = 0), orientation = Quaternion(w=1.0 ))
            self.target_pose_pub.publish(waypoint)
            return waypoint

    def get_next_waypoint(self):
        print("Getting next Waypoint..... \n Target distance: {} mm".format(self.target_dist))
        target_pose = self.locate_next_waypoint()
        self.waypoints.append(target_pose)

    # Goal Setting/Getting Functions
    # STATE 0
    def atGoal(self):
        self.set_vel(0.0, 0.0)
        self.set_LED(1)
        if len(self.waypoints) != 0:
            self.waypoints.pop(0)
        print("Goal Reached!")
        print("")
        print("")

        self.state = STATE_GET_NEXT_GOAL
        print("Current state is: 1 (STATE_GET_NEXT_GOAL)")

    # STATE 1
    def getNextGoal(self):
        if len(self.waypoints) == 0:
            # Wait for next waypoint
            raw_input("Press Enter to register next waypoint")
            self.get_next_waypoint()
            print("Got Next waypoint : ", self.waypoints[0])

        self.goal_pose.position.x = self.waypoints[0].position.x
        self.goal_pose.position.y = self.waypoints[0].position.y
        self.goalIndex += 1
        rospy.sleep(1.0)
        print("CURRENT : ", self.current_pose)
        print("GOAL : ", self.goal_pose)

        if self.euclidean_distance() > DISTANCE_TOLERANCE:
            self.startingDistance = self.euclidean_distance()
            self.starting_pose.position.x = self.current_pose.position.x
            self.starting_pose.position.y = self.current_pose.position.y

            self.set_LED(0)
            self.state = STATE_TURN_TO_GOAL
            print("Current state is: 2 (STATE_TURN_TO_GOAL)")
        else:
            print("Goal position you input is too close to the current position, please change your waypoints so they are greater than ", DISTANCE_TOLERANCE, "metres from each other")
            self.state = STATE_COMPLETE

    # STATE 2
    def turnToGoal(self):
        self.deltaYaw = self.get_deltaYaw()

        if abs(self.deltaYaw) > THRESHOLD_YAW_RADIANS:
            self.set_vel(0.0, (KP_ANG * self.deltaYaw) )
        else:
            self.set_vel(0.0, 0.0)
            print("NOW FACING goal_pose")
            rospy.sleep(1.0)

            self.state = STATE_DRIVE_TO_GOAL
            print("current state is: 3 (STATE_DRIVE_TO_GOAL)")

    # STATE 3
    def driveToGoal(self):
        if (self.euclidean_distance() > DISTANCE_TOLERANCE) and (self.distance_travelled() < self.startingDistance):
            self.set_vel(MAX_LINEAR_VEL_X, (KP_ANG * self.deltaYaw) )
            # print("distance travelled is:", self.distance_travelled())
        else:
            self.state = STATE_AT_GOAL
            print("current state is: 0 (STATE_AT_GOAL)")

if __name__ == "__main__":
    try:
        cart = SmartCart()
        while not rospy.is_shutdown():  #run infinite loop 
            cart.locate_next_waypoint(verbose=True) # Constantly run to locate waypoints
            if cart.state == STATE_AT_GOAL:        #STATE_AT_GOAL = 0
                # print("current state is: 0 (STATE_AT_GOAL)")
                cart.atGoal()
            elif cart.state == STATE_GET_NEXT_GOAL:  #STATE_GET_NEXT_GOAL = 1
                # print("current state is: 1 (STATE_GET_NEXT_GOAL)")
                cart.getNextGoal()
            elif cart.state == STATE_TURN_TO_GOAL:   #STATE_TURN_TO_GOAL = 2
                # print("current state is: 2 (STATE_TURN_TO_GOAL)")
                cart.turnToGoal()
            elif cart.state == STATE_DRIVE_TO_GOAL:  #STATE_DRIVE_TO_GOAL = 3
                # print("current state is: 3 (STATE_DRIVE_TO_GOAL)")
                cart.driveToGoal()
            elif cart.state == STATE_COMPLETE:
                print("Path is complete!  Shutting Down...")
                cart.set_vel(0.0, 0.0)
                break
            else:
                print("ERROR: NOT in any state")
                cart.set_vel(0.0, 0.0)

    except rospy.ROSInterruptException: pass
