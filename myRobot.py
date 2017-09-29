from numpy import array, dot
from math import sin, cos, exp, sqrt, pi, exp

from Robot3WD import Robot

class myRobot(Robot):
    def __init__(self, sampling_period):
        Robot.__init__(self, sampling_period)

    def inverse_kinematics(self, p_dot, theta):
      	L = self._L
      	wheel_radius = self._wheel_radius
      	r = wheel_radius
      	M=array([[((sin(theta))/r),-1.0*((cos(theta)/r)),(-L/r)],[((sin(pi/3.0 - theta))/r),((cos(pi/3.0 - theta))/r),(-L/r)],[-1.0*((sin(pi/3.0 + theta))/r),((cos(pi/3.0 + theta))/r),(-L/r)]])
      	wheel_angular_velocities= dot(M,p_dot)
      	return wheel_angular_velocities

    def move_left(self, vx, theta):
        p_dot = array([-vx, 0.0, 0.0]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)

    def move_forward(self, vy, theta):
        p_dot = array([0.0, vy, 0.0]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)
        
    def move_backward(self, vy, theta):
        p_dot = array([0.0, -vy, 0.0]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)
        
    def move_right(self, vx, theta):
        p_dot = array([vx, 0.0, 0.0]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)
        
    def rotate_CCW(self, th_dot, theta):
        p_dot = array([0.0, 0.0, th_dot]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)
          
    def rotate_CW(self, th_dot, theta):
        p_dot = array([0.0, 0.0, -th_dot]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)

    def forward_kinematics(self, wheel_angular_velocities, theta):
        L = self._L
        wheel_radius = self._wheel_radius

      	r = wheel_radius
      	a11=(2.0/3.0)*sin(theta)
      	a12=(((sqrt(3.0))/3.0)*cos(theta))-((1.0/3.0)*sin(theta))
      	a13=(-1.0*((sqrt(3.0))/3.0)*cos(theta))-((1.0/3.0)*sin(theta))
      	a21=(-1.0)*((2.0*cos(theta))/3.0)
      	a22=((sqrt(3.0)/3.0)*sin(theta))+((1.0/3.0)*cos(theta))
      	a23=(-1.0*((sqrt(3.0))/3.0)*sin(theta))+((1.0/3.0)*sin(theta))
      	a31=(-1.0*(3.0*L))
      	a32=a31
      	a33=a31
      		
      	M=array([[r*a11,r*a12,r*a13],[r*a21,r*a22,r*a23],[r*a31,r*a32,r*a33]])
      	p_dot = dot(M,wheel_angular_velocities)
        return p_dot
# end of myRobot class

# Define the HMatrix function
    def HMatrix(self, q):
   # input q is a vector with 3 elements
      	h11 = cos(q[2]) #this is theta
      	h12 = -sin(q[2])
      	h13 = q[0] #this is x
     
      	h21 = sin(q[2])
      	h22 = cos(q[2])
      	h23 = q[1] #this is y
     
      	h31 = 0
      	h32 = 0
      	h33 = 1.0 
     
      	H = array([[h11, h12, h13], [h21, h22, h23], [h31, h32, h33]])
      	return H

# Define the Vraw_to_distance function
    def Vraw_to_distance(self, Vraw):
      	c1 = 0.6208766
      	c2 = 0.05412
      	d = c1*exp(-1.0*c2*sqrt(Vraw))
      	return d
