#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
import tf.transformations as tf_
import math as M
import time as T

pub = None
sub = None
R_1 = False
Step = 0
W = 0
L = 0
Z = 0
Z_Level = 0
Speed_Rate_Z = 1
Speed_Rate_X = 1
Speed_Rate_Y = 1
Speed_Rate_Ang = 1
Position_x = 0
Position_y = 0
Position_z = 0
Rotation_Y_D = 0
Rotation_Y_R = 0
R_90 = False
R_0 = False
R_090 = False
R__ = False
X_F = 0
Y_F = 0
Z_F = 0
Check_Sum_Z = False
Check_Sum_X = False
Check_Sum_Y = False
Check_Yaw_Z = False
Kp = 0.1
control_output = 0
tolerance = 0.1
tolerance_1 = 0.01
target_angle = 0
error_angle = 0
control_output_angle = 0
counter_x = 0
counter_y = 0
s = False 
R_090 = False
R_90 = False 
counter_step = 5
M_1 = False 
R__2 = True
step_in_z = -3 
Level = 0 
def move_in_Z(rate):
    global Speed_Rate_Z
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.linear.y = 0
    twist_msg.linear.z = Speed_Rate_Z * rate
    pub.publish(twist_msg)


def move_in_X(rate):
    global Speed_Rate_X
    twist_msg = Twist()
    twist_msg.linear.x = Speed_Rate_X * rate
    twist_msg.linear.y = 0
    twist_msg.linear.z = 0
    pub.publish(twist_msg)


def move_in_Y(rate):
    global Speed_Rate_Y
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.linear.y = Speed_Rate_Y * rate
    twist_msg.linear.z = 0
    pub.publish(twist_msg)


def yaw_(rate):
    global Speed_Rate_Ang
    twist_msg = Twist()
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = Speed_Rate_Ang * rate
    pub.publish(twist_msg)


def stop_vehicle():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.linear.y = 0
    twist_msg.linear.z = -0.000000001
    pub.publish(twist_msg)


def callback(msg):

    global Position_x, Position_y, Position_z
    global Check_Sum_Z, Check_Sum_X, Check_Sum_Y
    global X_F, Y_F, Z_F
    global Rotation_Y_D, Rotation_Y_R
    global Kp, control_output
    global Check_Yaw_Z, R_1, Step
    global counter_x, counter_y
    global R_90, R_090, R_0, R__
    global R__1, s,R_90 ,R_090, counter_step,M_1, R__2, step_in_z, Z_Level, Level
    R__1 = True
    Position_x = msg.pose.position.x
    Position_y = msg.pose.position.y
    Position_z = msg.pose.position.z
    Rotation_Y_R = tf_.euler_from_quaternion(
        [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    )
    Rotation_Y_D = M.degrees(float(Rotation_Y_R[2]))

    def P_Controller(input, z =0, degrees=0, l=L, w=W, step = Step, back = 1):

        if input == 1:
            global target_depth, error, control_output, Kp
            target_depth = z
            error = target_depth - Position_z
            control_output = Kp * error
            return control_output

        elif input == 2:
            global target_angle, error_angle, control_output_angle
            target_angle = degrees
            if target_angle == 180:
               if Rotation_Y_D < 0:
                 error_angle = -target_angle - Rotation_Y_D
               else:
                  error_angle = target_angle - Rotation_Y_D
            else:
                 error_angle = target_angle - Rotation_Y_D
            control_output_angle = Kp * error_angle
            return control_output_angle

        elif input == 3:
            global target_depth_y, error_y, control_output_y
            target_depth_y = w
            error_y = target_depth_y - Position_x
            control_output_y = Kp * error_y
            return error_y

        elif input == 4:
            global target_depth_x, error_x, control_output_x
            target_depth_x = l
            error_x = target_depth_x + Position_y
            control_output_x = Kp * error_x
            return control_output_x

        elif input == 5:
            global target_depth_x_1, error_x_1, control_output_x_1
            target_depth_x_1 = step
            error_x_1 = target_depth_x_1 - Position_x
            control_output_x_1 = Kp * error_x_1
            return control_output_x_1

        elif input == 6:
            global target_depth_x_2, error_x_2, control_output_x_2
            target_depth_x_2 = l
            error_x_2= target_depth_x_2 - Position_y
            control_output_x_2 = Kp * error_x_2
            return control_output_x_2

        elif input == 7:
            global target_depth_x_3, error_x_3, control_output_x_3
            target_depth_x_3 = step
            error_x_3 = target_depth_x_3 + Position_x  
            control_output_x_3 = Kp * error_x_3
            return control_output_x_3
 
    #if Level != Z_Level:
    if  not Check_Sum_Z:
	      if abs(round(P_Controller(1,z =step_in_z), 2)) > tolerance:
		    move_in_Z(P_Controller(1,z =step_in_z)) 
		    rospy.loginfo("depth_1")     

	      else:
		  stop_vehicle()
		  print("out")
		  Check_Sum_Z = True
		  step_in_z+=Z #-4 
                  counter_step = 5

	      
    if Check_Sum_Z and counter_x != 4 : 
	       
	       if counter_x % 2 == 0:
	      
		    if abs(round(P_Controller(2, degrees= -90), 2)) > 0.01 and not s:
			yaw_(P_Controller(2, degrees= -90)) 
			rospy.loginfo("OuT_1")   

		    if  abs(round(P_Controller(4, l=L), 2)) > 0.01 and abs(round(P_Controller(2, degrees = -90), 2)) < 0.03 and not s :
			move_in_X(P_Controller(4, l=L)) 
			rospy.loginfo("OuT_2")

		    if abs(round(P_Controller(2, degrees = -90), 2)) < 0.02 and abs(round(P_Controller(4,l= L), 2)) < 0.02 and not s :
			s = True
			R_090 = True 
		  
		    if abs(round(P_Controller(2, degrees = 0), 2)) > 0.01 and R_090:
			yaw_(P_Controller(2, degrees = 0))
			rospy.loginfo("OuT_3")
		    

		    if abs(round(P_Controller(5, step=counter_step), 2)) > 0.01 and abs(round(P_Controller(2, degrees = 0), 2)) < 0.04 and R_090 :
			move_in_X(P_Controller(5, step=counter_step)) # 5 #15
			rospy.loginfo("OuT_4")

		    if abs(round(P_Controller(5, step=counter_step), 2)) < 0.03 and abs(round(P_Controller(2, degrees = 0), 2)) < 0.02 and R_090: 
			R_090 = False 
			R_90 = True
			counter_x += 1
			counter_step += Step #10 #20

	       if counter_x % 2 == 1:

		    if abs(round(P_Controller(2, degrees = 90), 2)) > 0.01 and R_90:
			yaw_(P_Controller(2, degrees=90))
			rospy.loginfo("OuT_5")

		    if abs(round(P_Controller(6, l=0), 2)) > 0.02 and abs(round(P_Controller(2, degrees = 90), 2)) < 0.3 and R_90 :
			move_in_X(P_Controller(6, l=0))
			rospy.loginfo("OuT_6")

		    if abs(round(P_Controller(6, l=0), 2)) < 0.03 and abs(round(P_Controller(2, degrees =  90), 2)) < 0.02 and R_90:
			R_90 = False 
			R__ = True 


		    if abs(round(P_Controller(2, degrees =  0), 2)) > 0.01 and R__:
			yaw_(P_Controller(2,degrees =  0))
			rospy.loginfo("OuT_7")

		    if abs(round(P_Controller(2, degrees = 0), 2)) < 0.06 and R__:
			M_1 = True
			R__ = False 

		    if abs(round(P_Controller(5, step=counter_step), 2)) > 0.07 and M_1 and abs(round(P_Controller(2, degrees =  0), 2)) < 0.09 :
			move_in_X(P_Controller(5, step=counter_step)) 
			rospy.loginfo("OuT_8")

		    if abs(round(P_Controller(5, step=counter_step), 2)) < 0.08 and abs(round(P_Controller(2, 0), 2)) < 0.07 and M_1:
			counter_x += 1
			counter_step += Step 
			M_1 = False
		        s = False 
		        R__2 = True

    if Check_Sum_Z and counter_x == 4 : 

	       if abs(round(P_Controller(2, degrees = 180), 2)) > 0.001  :
		   yaw_(P_Controller(2, degrees = 180))
		   rospy.loginfo("OuT_9" +str(Rotation_Y_D))


	       if abs(round(P_Controller(7, step=-1), 2)) > 0.001 and abs(round(P_Controller(2, degrees = 180), 2)) < 0.03  : #0.40
			move_in_X(P_Controller(7, step=-1))
			rospy.loginfo("OuT_10")
	       
	       if abs(round(P_Controller(7, step=-1), 2)) < 0.01 and abs(round(P_Controller(2, degrees = 180), 2)) < 0.02 and R__2:
		 R__2 = False 
		 Check_Sum_Z = False
		 counter_x = 0
		 #Level+=1 


### Human Condition Here
            

def main():
    global X_F, Y_F, Z_F, W, L, Z, pub, sub, Speed_Rate_Z, Speed_Rate_X, Speed_Rate_Y, Speed_Rate_Ang, Step, Z_Level

    rospy.init_node("search_rescue")
    pub = rospy.Publisher("/rexrov/cmd_vel", Twist, queue_size=20)
    sub = rospy.Subscriber(
        "/rexrov/ground_truth_to_tf_rexrov/pose", PoseStamped, callback, queue_size=20
    )
    W = rospy.get_param("/Width_Distance_Y", -3)
    L = rospy.get_param("/Length_Distance_X", 7)
    Z = rospy.get_param("/Depth_Distance", -1)
    Step = rospy.get_param("/Step", 5)
    Z_Level = rospy.get_param("/Z_Level", 5)
    Speed_Rate_Z = rospy.get_param("/Speed_Rate_Z", 100)
    Speed_Rate_X = rospy.get_param("/Speed_Rate_X", 120)
    Speed_Rate_Y = rospy.get_param("/Speed_Rate_Y", 100)
    Speed_Rate_Ang = rospy.get_param("/Speed_Rate_Ang", 1)

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("Code has been terminated")
    except Exception as e:
        print("Unknown error occurred:", str(e))

