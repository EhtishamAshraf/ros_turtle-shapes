#!/usr/bin/env python 
# above command is telling the interpreter to use python 2 to run the code, adjust it accordingly

"""
The code draws different shapes with turtlesim.
A.	1. Square	2. Rectangle	3. Triangle	4.Circle	5.Go to the goal position
B.	Distance between two points is found out using Euclidean distance formula: https://www.cuemath.com/euclidean-distance-formula/ (which 		is derived from Pythagorean theorm)
C.	Angle between two points is found out using: https://www.youtube.com/watch?v=Qh15Nol5htM
"""

# import required libraries:
import rospy 				# ros library, it has functions like creating node, creating publisher
import math 				# math library, it has functions like atan2, pow, sqrt
from time import sleep 			# time library
from geometry_msgs.msg import Twist	# The geometry_msgs contains the variable type Twist, The Twist message is composed by 3 linear 					  components and 3 angular components. Our topic '/turtle1/cmd_vel' uses the Twist message.
from turtlesim.msg import Pose 	# The turtlesim.msg contains the Pose message type, which is the one published to the topic '/turtle1/pose'

# initializing global variables:
init_pose_x = 0
init_pose_y = 0
new_pose_x = 0
new_pose_y = 0

#Turtle class:
class Turtle:

    # Constructor. This method is called whenever an object of the class is created & it initializes the attributes of the class
    # self keyword is used to represent objects of the given class. if "self" argument is not added, then same class couldn't hold information 	     of all the objects
    def __init__(self):
        rospy.init_node('turtlebot', anonymous=True) # Creates a node with name 'turtlebot' and make sure it is a unique node (using 							     anonymous=True).
	# Publisher which will publish to the topic '/turtle1/cmd_vel'.we publish velocities to the topic to move the robot
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10) 
	# A subscriber to the topic '/turtle1/pose'. self.pose_callback is called when a message of type Pose is received. we subscribe to get 		  the position of the turtle in cartesian coordinates.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.pose_callback) 
        self.pose = Pose() # creating pose object
        self.rate = rospy.Rate(10) # publish rate 10 Hz (i.e. 100ms)

    #Callback function which is called when a new message of type Pose is received by the subscriber
    def pose_callback(self,data):
	# The pose_callback method is a callback function which will be used by the subscriber: it will get the turtle current pose and save   		it in the self.pose attribute: 
        self.pose = data # when it's called, get current pose of the turtle (data variable contains the current pose of the turtle)
        self.pose.x = round(self.pose.x, 4) # rounding off to 4 digits
        self.pose.y = round(self.pose.y, 4)
    
    # calculating the distance between goal and start points
    def distance_to_goal(self,final_x,final_y):
        return math.sqrt(math.pow((final_x - self.pose.x), 2) + math.pow((final_y - self.pose.y), 2))

    # function to set the angular velocity using Proportional controller   
    def linear_velocity(self,final_x,final_y,KLinear = 1.5):
        return KLinear * self.distance_to_goal(final_x,final_y) # multiplying with distance to goal to slow down the speed with the distance 								(when distance b/w start & goal is bigger, then speed is higher, and vice verse)
    
    # function to set the angular velocity using Proportional controller
    def angular_vel(self, hdg_error, KAngular = 1.5):
        return KAngular * hdg_error # multiplying with heading error to slow down the speed with the angle (when angle difference is bigger 					    then rotation speed is higher, and vice verse)
    
    # function to set the steering angle of the robot:
    def set_direction(self,final_x,final_y,threshold):
        print("x = "+str(self.pose.x))
        print("y = "+str(self.pose.y))        
        print("goal x = "+str(final_x))
        print("goal y = "+str(final_y))
		# This link provides details of the function: https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Math/atan2
		# math.atan2() is a bit different than math.atan()
        desired_hdg = math.atan2(final_y - self.pose.y, final_x - self.pose.x) # calculate the required angle	
        curr_hdg = self.pose.theta		# current angle of the robot
	hdg_error = desired_hdg - curr_hdg	# calculate difference between current and required angle
        print("desired heading "+str(desired_hdg)+" current hdg "+str(curr_hdg))

        vel_msg = Twist()
        while abs(hdg_error) > threshold:		    # keep rotating the robot while difference of angles is greater than threshold
            hdg_error = desired_hdg - self.pose.theta 	    # keep checking the difference (current theta will keep changing continuously)
            vel_msg.angular.z = self.angular_vel(hdg_error) # set the angular velocity of the robot using "angular velocity" function
            self.velocity_publisher.publish(vel_msg) 	    # publish the velocity value
        
        print("desired heading reached")

    # function to reach the goal point:    
    def move_to_goal(self,final_x,final_y,threshold):
        vel_msg = Twist()
        while self.distance_to_goal(final_x,final_y) > threshold: # while distance to the goal > threshold, keep moving robot in same direction
	    # set the linear velocity of the robot using "linear velocity" function
            vel_msg.linear.x = self.linear_velocity(final_x,final_y) 
            self.velocity_publisher.publish(vel_msg) # publish the velocity value

        print("desired position reached")

    # code to draw square:
    def square(self):
	global init_pose_x # variables to store initial x & y of the robot
	global init_pose_y
	init_pose_x = self.pose.x # current (initial) value of x & y is stored in the variables	
	init_pose_y = self.pose.y
	print('Robot is currently at x: '+ str(init_pose_x)+ ' and y: '+ str(init_pose_y))
	length = input("Please input length of the square: ")
	length = length+self.pose.x # add the current x value of the robot to the length of the 'square' to draw square from the initial value.
	#setting limits:	
	if length>=10.5444:
		print("Length exceeded the limit so has been cut down to 5!!")
		length = 10.5444

	# draw the square by calling move to goal and set direction functions.
	# first and second arguments are the x & y position whereas third argument is the tolerance 
	self.move_to_goal(length,init_pose_y,0.00001)
	self.set_direction(length,length,0.00001)
	self.move_to_goal(length,length,0.00001)
	self.set_direction(init_pose_x,length,0.00001)
	self.move_to_goal(init_pose_x,length,0.00001)
	self.set_direction(init_pose_x,init_pose_y,0.00001)
	self.move_to_goal(init_pose_x,init_pose_y,0.1)
	self.set_direction(length,init_pose_y,0.01)
	
	# Stopping our robot after the movement is over.
	vel_msg = Twist() # Twist message has 3 linear and 3 angular components
	vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg) # publishing the velocity values to stop the robot
        rospy.spin() # If we press control + C, the node will stop.

    # code to draw rectangle: Uncommented lines are same as square function (refer to square function)
    def rectangle(self):
	global init_pose_x
	global init_pose_y
	init_pose_x = self.pose.x	
	init_pose_y = self.pose.y
	print('Robot is currently at x: '+ str(init_pose_x)+ ' and y: '+ str(init_pose_y))
	length_x = input("Please input horizontal length of the rectangle: ")
	length_y = input("Please input vertical length of the rectangle: ")
	length_x = length_x+self.pose.x
	length_y = length_y+self.pose.y
	if length_x>=10.5444:
		print("Length exceeded the limit so has been cut down to 5!!")
		length_x = 10.5444
	elif length_y>=10.5444:
		print("Length exceeded the limit so has been cut down to 5!!")
		length_y = 10.5444

	self.move_to_goal(length_x,init_pose_y,0.00001)
	self.set_direction(length_x,length_y,0.00001)
	self.move_to_goal(length_x,length_y,0.00001)
	self.set_direction(init_pose_x,length_y,0.00001)
	self.move_to_goal(init_pose_x,length_y,0.00001)
	self.set_direction(init_pose_x,init_pose_y,0.00001)
	self.move_to_goal(init_pose_x,init_pose_y,0.1)
	self.set_direction(length_x,init_pose_y,0.01)		

	vel_msg = Twist()
	vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.spin() 

    # code to draw triangle: Uncommented lines are same as square function (refer to square function)
    def triangle(self):
	global init_pose_x
	global init_pose_y
	init_pose_x = self.pose.x	
	init_pose_y = self.pose.y
	print('Robot is currently at x: '+ str(init_pose_x)+ ' and y: '+ str(init_pose_y))
	length_x = input("Please input length of base of the triangle: ")
	length_y = input("Please input length of perpendicular of the triangle: ")
	length_x = length_x+self.pose.x
	length_y = length_y+self.pose.y
	if length_x>=10.5444:
		print("Length exceeded the limit so has been cut down to 5!!")
		length_x = 10.5444
	elif length_y>=10.5444:
		print("Length exceeded the limit so has been cut down to 5!!")
		length_y = 10.5444

	self.move_to_goal(length_x,init_pose_y,0.00001)
	self.set_direction(length_x,length_y,0.00001)
	self.move_to_goal(length_x,length_y,0.00001)
	self.set_direction(init_pose_x,init_pose_y,0.00001)
	self.move_to_goal(init_pose_x,init_pose_y,0.1)
	self.set_direction(length_x,init_pose_y,0.01)

	vel_msg = Twist()
	vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.spin() 
    
    # code to draw circle: Uncommented lines are same as square function (refer to square function)
    def circle(self):
	global init_pose_y
	global new_pose_y # store current y position of the robot in this variable	
	init_pose_y = self.pose.y # store initial y in this variable
	print('Robot is currently at x: '+ str(init_pose_x)+ ' and y: '+ str(init_pose_y))
	radius = input("Please input radius of the circle: ")
	if radius>=4:
		print("Radius exceeded the limit so has been cut down to 5!!")
		radius = 4
	#Move the robot a bit in the circular direction
	vel_msg = Twist()
	vel_msg.linear.x = radius
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 1.5
	self.velocity_publisher.publish(vel_msg)
	sleep (1)
	# find difference between initial y and current y (: current y is still 0 and it will be updated inside the while loop)
	error_y = new_pose_y - init_pose_y
	# keep rotating the robot in the circle until the difference between initial y and current y is greater than 0.001
	while abs(error_y)>0.001:
		new_pose_y = self.pose.y # update the value of the current y position of the robot
		error_y = new_pose_y - init_pose_y # calculate the error to update the value of the error
		vel_msg = Twist()
		# circle is drawn by continuously setting the linear velocity along x and angular velocity along z and publish it to the topic.
		vel_msg.linear.x = radius
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = 1.5
		self.velocity_publisher.publish(vel_msg)
	
	vel_msg = Twist()
	vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.spin() 	

    # Function to go to a goal position: Uncommented lines are same as square function (refer to square function)
    def goal(self):
	global init_pose_x
	global init_pose_y
	init_pose_x = self.pose.x	
	init_pose_y = self.pose.y
	print('Robot is currently at x: '+ str(init_pose_x)+ ' and y: '+ str(init_pose_y))
	length_x = input("Please input goal x position: ")
	length_y = input("Please input goal y position: ")

	if length_x>=10.5444:
		print("Length exceeded the limit so has been cut down to 10.5!!")
		length_x = 10.5
	elif length_x<=1.5:
		print("Length below the limit so has been cut down to 1.5!!")
		length_x = 1.5
	elif length_y>=10.5444:
		print("Length exceeded the limit so has been cut down to 10.5!!")
		length_y = 10.5
	elif length_y<=1.5:
		print("Length below the limit so has been cut down to 1.5!!")
		length_y = 1.5

	self.set_direction(length_x,length_y,0.01)
	self.move_to_goal(length_x,length_y,0.1)

	vel_msg = Twist()
	vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.spin() 
		
#Main loop:
if __name__ == '__main__': # command to run the code only when this file is run directly and not when imported as a module

    try: # command to test the code when there's no error/exception in the code
	obj = Turtle() # create an object of the class

	# take user input and based on the choice call the specific function of the Turtle class.
	User_choice = raw_input("What would you like to draw? or would you like to reach the goal? ")
	if User_choice == "square":
        	obj.square()
	elif User_choice == "triangle":
		obj.triangle()
	elif User_choice == "rectangle":
		obj.rectangle()
	elif User_choice == "circle":
		obj.circle()
	elif User_choice == "goal":
		obj.goal()
	else:
		print("Please enter one of the following: square, rectangle, triangle, circle, goal")
    # except is used to execute try block code when there's an error/exception in the code:  
    except rospy.ROSInterruptException: # pass if there's a Ros interrupt 
        pass