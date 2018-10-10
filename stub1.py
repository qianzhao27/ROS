#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from math import pow, atan2, sqrt
from std_msgs.msg import String


class Quadrotor:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('quadrotor_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/ground_truth_to_tf/pose',
                                                PoseStamped, self.update_pose)

        self.ps = PoseStamped()
        self.pose = Point()
        self.pose.x = self.ps.pose.position.x
        self.pose.y = self.ps.pose.position.y
        self.pose.z = self.ps.pose.position.z
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.ps = data
        self.pose.x = round(self.ps.pose.position.x, 4)
        self.pose.y = round(self.ps.pose.position.y, 4)
        self.pose.z = round(self.ps.pose.position.z, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2) )
    # perpendicula distance
    def perpend_distance(self,goal_pose):
        return sqrt(pow((goal_pose.z - self.pose.z),2))
    # z_vel
    def z_vel(self,goal_pose,constant= 0.5):
        return constant * (goal_pose.z - self.pose.z)

    def linear_vel(self, goal_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose)) # - self.pose.theta)

    def move2goal(self):
        """Moves the turtle to the goal."""
        goal_pose = Point()

        # Get the input from the user.
        goal_pose.x = input("Set your x goal: ")
        goal_pose.y = input("Set your y goal: ")
        goal_pose.z = input("Set your y goal: ") #add new para. z

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = input("Set your tolerance: ")

        vel_msg = Twist()

        while not rospy.is_shutdown():
            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control
            
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = self.z_vel(goal_pose)
            # Publishing our vel_msg
            if self.perpend_distance <= distance_tolerance:
                vel_msg.linear.z = 0
            
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()
            
            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)
            if self.euclidean_distance(goal_pose) <= distance_tolerance:
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()


        
        # If we press control + C, the node will stop.
        #rospy.spin()

if __name__ == '__main__':
    try:
        x = Quadrotor()
        x.move2goal()

    except rospy.ROSInterruptException:
        pass
