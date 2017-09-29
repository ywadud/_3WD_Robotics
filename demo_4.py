import os
from numpy import array, dot, zeros
from numpy.linalg import norm
from math import sqrt, pi, sin, cos, atan2
from time import time
from myRobot import *

# Set up sampling period T_s and stopping time T_f
T_s = 0.02
T_f = 8.0

# Set up initial posture
x_0 = 0.0
y_0 = 0.0
theta_0 = pi/6.0

# Set up goal position
x_f = 0.7
y_f = 1.0
theta_f = pi/6.0

# Set up p_0 and p_f
p_0 = array([x_0, y_0, theta_0]).T
p_f = array([x_f, y_f, theta_f]).T

# Initialize d_min and d_free (See Eq. 4.4 and 4.5)
d_min = 0.08
d_free = d_min*1.25

# Set up error tolerance to be within 1cm of target in both x and y directions
epsilon = sqrt(2.0*(1.0/100.0)**2)

# Set up controller gains (required in Eq. 4.11 and 4.12)
k_rho = 1.0
k_beta = -1.0
k_alpha = (2.0/pi)*k_rho - (5.0/3.0)*k_beta + 0.5


#set initial angular velocity as 000
w = array([0.0, 0.0, 0.0]).T


# Initialize vector pr_dot to be used for inverse kinematics
pr_dot = array([0.0,0.0,0.0]).T


# Initialize vector d. The sensor measurement d_i will ne assigned to d[i]
d = array([10.0, 10.0, 10.0, 10.0, 10.0, 10.0]).T

# Initialize a 3x6 matrix u_iR. The vector delta_i^R will be assigned to
# the i-th column of this matrix
u_iR = zeros(shape=(3,6))


# Initialize a 6x3 matrix sensor_loc to store the sensor frame
# locations as per Table 4.1. The i-th row of this matrix corresponds
# to the i-th ROW of Table 4.1.
R = 0.13
halfR = R/2.0
root3Rover2 = sqrt(3)*R/2.0
sensor_loc = array([
   [-halfR, -root3Rover2, -(2.0/3.0)*pi],
   [-root3Rover2, halfR, (5.0/6.0)*pi],
   [0, R, (1.0/2.0)*pi],
   [halfR, root3Rover2, (1.0/3.0)*pi],
   [root3Rover2, halfR, (1.0/6.0)*pi],
   [root3Rover2, -halfR, -(1.0/6.0)*pi]
])

# open a file to store data for plotting later
f = open('p.csv', 'w')
os.chmod('p.csv', 0666)


# Initial robot
robot = myRobot(T_s)
robot.initialize(theta_0)


# Compute distance to goal, rho, using Eq. 4.1
dp = p_f - p_0
rho = norm(dp[0:2])  # Note: In Python, the range 0:2 means [0,1] NOT [0,1,2]


start_time = time()

# Initialize additional variables
p = p_0
goal_reached = False
elapsed_time = 0.0

obstacle = 0
xtmp = 0
ytmp = 0

# Control loop
while ( (not goal_reached) and (elapsed_time < T_f) ):
	robot.get_readings_update()

	# save data to file for plotting
	f.write('%10.3e %10.3e % 10.3e %10.3e %10.3e\n' % (elapsed_time, p[0], p[1], p[2], rho))
	#print 'p', p[0], p[1], p[2], rho
	theta = robot.orientation
	w = robot.angular_velocities

	# Use forward kinematics to get p_dot
	p_dot = robot.forward_kinematics(w, theta)

	HRO = robot.HMatrix(p) #3x3 matrix
	#print 'raw sen val', robot.ir_sensors_raw_values

	# Get sensor measurements w.r.t. sensor frames
	
	alpha = atan2(dp[1],dp[0]) - theta

    	# Check if there is a risk of collision with obstacle(s).
    	# If true, determine the temporary goal vectors delta_goal^0 and delta_goal^R
    	# and use them to find the temporary alpha and rho.
    	# See Eqs. 4.4 to 4.11.
	
	for i in range(0,6):
		d[i] = robot.Vraw_to_distance(robot.ir_sensors_raw_values[i])
		
	   	if (d[i] <= d_min):
			obstacle = 1
			print 'Obstacle:',i
			break

	if (obstacle == 1):
		for i in range(0,6):
			u_iR[0,i]=d[i]
    	    		u_iR[2,i]=1
		
		for i in range(0,6):
    	    		if d[i] <= d_free:
				u_iR[0,i]=0
	   		sen_i = sensor_loc[i,:]
            		u_iR[:,i] = dot(robot.HMatrix(sen_i),u_iR[:,1])
        		xtmp = 0
        	for j in range(0,6):
            		xtmp = xtmp+u_iR[0,j]
			ytmp = ytmp+u_iR[1,j]

        	delta_goal_robot=array([xtmp,ytmp,1]).T

    		delta_goal_base=dot(HRO,delta_goal_robot)
        	dp = delta_goal_base - p
        	rho = norm(dp[0:2])
        	alpha = atan2(delta_goal_robot[1],delta_goal_robot[0])-theta

	#print 'p_temp ',p_temp
	obstacle = 0

	# Determine angle alpha (remember to used atan2 instead of atan)
	
	# Determine the angle beta
	beta = -(alpha + theta)
		
	# Determine linear and angular velocities of the robot (v and w) using the control law
	# given in Eqs. 4.12 and 4.13
	v = k_rho*rho
	w = k_alpha*alpha + k_beta*beta
		
	pr_dot[0] = v*cos(theta)
	pr_dot[1] = v*sin(theta)
	pr_dot[2] = w
	
    	# Now use Inverse Kinematics to determine wheel ref velocities
	wheel_ref_vel = robot.inverse_kinematics(pr_dot, theta)

    	# Execute motion
	robot.set_angular_velocities(wheel_ref_vel)

    	# Odometry update
	p = p + (p_dot * T_s)

    	# Replace calculated update for theta with measured value from IMU
	#p[2]= robot.orientation

    	# Check to see if goal is reached
	dp = p_f - p
	rho= norm(dp[0:2])
	goal_reached = ( rho <= epsilon)
	
    	# time update
	elapsed_time = time() - start_time


print 'ELPASED TIME = %s rho = %s' % (elapsed_time, rho)

# Either goal is reached or current_time > T_f
if goal_reached:
	print 'Goal is reached, error norm is', rho
else:
	print 'Failed to reach goal, error norm is', rho

robot.stop()
robot.close()

f.close()
