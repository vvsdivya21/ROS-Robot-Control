import time
import math
import numpy as np
import random
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from quantum_qubit_msgs.msg import WheelVelocities


class QuantumQubitNode(Node):
    def __init__(self):
        super().__init__('quantum_qubit_control_node')
        self.robot_prefix = 'quantum_qubit'
        #the below code calls the on_off service and subscribes to the topics quantum_qubit/get/pose, quantum_qubit/set/WheelVelocities and quantum_qubit/kinematics/set/position
        #the robot_flag is set to false in the beginning, it will be set to true when the robot is turned on
        self.robot_flag = False
        #the initial and final robot positions are taken as None at the start
        self.robot_pose = None
        self.final_robot_position = None
        #now creating the publishers and subscribers
        self.on_off_service = self.create_service(SetBool,
                                                  f'/{self.robot_prefix}/on_off', 
                                                  self.on_off_service_callback)
        self.velocity_publisher = self.create_publisher(Twist, 
                                                        f'/{self.robot_prefix}/cmd_vel', 
                                                        10)
        self.publish_WheelVelocities = self.create_publisher(WheelVelocities,
                                                             f'/{self.robot_prefix}/set/WheelVelocities', 
                                                             10)
        self.twist = Twist()
        #we are creating timer to get the start time of the code
        self.timer = self.create_timer(0.1, self.robot_planning)
        self.start_time = self.get_clock().now()

        self.get_pose_subscriber= self.create_subscription(
            PoseStamped,
            f'/{self.robot_prefix}/get/pose',
            self.get_pose_callback,
            10
        )

        self.get_position_subscriber = self.create_subscription(
            PointStamped,
            f'/{self.robot_prefix}/kinematics/set/position',
            self.get_position_callback,
            10
        )

    #the function waits for sometimetime to switch on the robot after recieving the data to turn on
    def get_pose_callback(self, msg):
        self.robot_pose = msg

    def get_position_callback(self, msg):
        self.final_robot_position = msg
    def robot_planning(self):
        if self.robot_flag:
                #we calculate the time now and store it in the variable t0
                #t1 is the time that has passed from t0, we get this by calculating the difference between time now and the time when we switched on
                t0 = self.get_clock().now()
                t1 = (t0 - self.start_time).nanoseconds / 1e9
                #if the time has passed above 120 seconds/ 2 minutes, then switch off the robot
                if t1 >= 120.0:
                    self.get_logger().info(f"Switching off the robot, battery over as 2 minutes have passed")
                    self.switch_off_robot()
                #if the left and right wheel velocities are 0, then switch off the robot
                if self.robot_pose is not None and self.final_robot_position is not None:
                    i=self.left_and_right_wheel_velocities()
                    if i==0:
                        self.get_logger().info(f"Switching off the robot, battery over as 2 minutes have passed")
                        self.switch_off_robot()
                #if the robot has reached its desired destination, then turn off the robot
                    else:
                        self.get_logger().info(f"Switching off, The Robot has reached its Target position!")
                        self.switch_off_robot()
                    self.velocity_publisher.publish(self.twist)
        else:
            self.twist = Twist()

    #This function is called when robot is needed to be switched off
    #the robot reduces its velocity instead and turns off
    #the implementation has been made here showing the velocities decreasing and finally reaching zero as the function is called and thereby turning of the robot
    def switch_off_robot(self):
        if self.robot_flag:
            self.get_logger().info("the robot is switching off and its velocity is decreasing.")
            # here we take 20 as the number of units to reduce gradually
            #we chose the delay between each unit as 0.2 for it to reduce slowly

            for i in range(20):
               
                vr = self.twist.linear.x * (1.0 - (i / 20))
                vl = self.twist.linear.x * (1.0 - (i / 20))
                publish_the_message = WheelVelocities()
                #publishing each velocity of right wheel as it in decreasing in steps of 20 with 0.2 delay 
                publish_the_message.vr = vr
                # publishing each velocity of left wheel as it in decreasing in steps of 20 with 0.2 delay 
                publish_the_message.vl = vl
                self.publish_WheelVelocities.publish(publish_the_message)
                time.sleep(0.2)  #the delay is given here
            self.twist = Twist()
            self.velocity_publisher.publish(self.twist)
            self.robot_flag = False
            #after the robot has reached zero velocity, we updated the robot flag to false and switched off the robot
            self.get_logger().info("Robot is now turned off")
    def on_off_service_callback(self, request, response):
        if request.data == True:
            self.get_logger().info("Recieved the Request, Robot will Switch on")
            time.sleep(3)
         #as we can observe here, the robot_flag is set to true after the time.sleep function, this is to have a waiting time in between
            self.robot_flag = True
            self.get_logger().info("Robot is Switched on.")
            # The response has been set to true only after we waited for sometime
            response.success = True
        else:
            #if the service responds with false, ie; to turn off the robot, then here we call switch_off_robot() function
            self.switch_off_robot()
            response.success = True
        return response


    def left_and_right_wheel_velocities(self):
        #this function is to calculate the velocities of each wheel, ie; the left and right wheel velocities
        r = 0.02
        l = 0.25

        #r represents the radius of the robot
        #l represents the distance of the base
        #we get the angle between the robot and the position using the abelow atan2 method
        position_angle = math.atan2(self.final_robot_position.point.y - self.robot_pose.pose.position.y, self.final_robot_position.point.x -  self.robot_pose.pose.position.x)

        #the angle of the robot
        phiz_degree=np.degrees(2 * math.atan2(self.robot_pose.pose.orientation.z, self.robot_pose.pose.orientation.w))

        #how much is the difference/ error between the two angles
        rotation_angle = position_angle - (2 * math.atan2(self.robot_pose.pose.orientation.z, self.robot_pose.pose.orientation.w))

        #the time taken is given by multiplying the distance with angular velocity which is 2*pi*frequency
        #here frequency is given as 0.01, therefore using it in the equation to get angular velocity
        time_taken_for_robot=(r*rotation_angle)/( 2*math.pi*0.01)
        time_start=self.get_clock().now()
        #we again start a timer to implement the logic
        while(self.get_clock().now() - time_start).nanoseconds / 1e9 < time_taken_for_robot:
            self.get_logger().info(f" {phiz_degree}: Robot is turning to align with the position given")

            vr = ( 2*math.pi*0.01)*(r+(l/2))
            #the velocity of the left wheel is opposite to that of right wheel in this case
            vl = -vr
            #now publishing the velocities of left and right wheels
            publish_message = WheelVelocities()
            publish_message.vr = vr
            publish_message.vl = vl
            self.publish_WheelVelocities.publish(publish_message)
        #to find the linear velocity, after it aligns itself with the X-axis
        displacement_of_robot = math.sqrt((self.final_robot_position.point.x- self.robot_pose.pose.position.x)**2+(self.final_robot_position.point.y-self.robot_pose.pose.position.y)**2)
        start_time_disp=self.get_clock().now()
        self.get_logger().info(f"Robot is moving linearly now")
        while True:
            t0 = self.get_clock().now()
            t1 = (t0 - self.start_time).nanoseconds / 1e9
            if t1 >= 120.0:
                return 0
            else:
                vr = random.uniform(0.001,1.000)
                vl = vr
                disp_time=(displacement_of_robot-0.05)/vr
                publish_message = WheelVelocities()
                publish_message.vr = vr
                publish_message.vl = vl
                self.publish_WheelVelocities.publish(publish_message)
                if((self.get_clock().now() - start_time_disp).nanoseconds / 1e9 >= disp_time):
                    break
        self.get_logger().info(f"Robot has finally reached the position!")
        return 1

def main(args=None):
    rclpy.init(args=args)
    node = QuantumQubitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()