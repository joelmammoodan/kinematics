#import libraries
import numpy as np

#Link Length
L1=1.0
L2=1.0


#gives catesian coordinates of the end effector of SCARA ARM
def forward_kinematics(theta1,theta2):
    x=L1*np.cos(theta1)+L2*np.cos(theta1+theta2)
    y=L1*np.sin(theta1)+L2*np.sin(theta1+theta2)
    return(x,y)


#gives the joint angles given the end effector position
def inverse_kinematics(x,y):
    val=(x**2+y**2-L1**2-L2**2)/2*L1*L2
    theta2=np.arccos(val)
    k1=L1+L2*np.cos(theta2)
    k2=L2*np.sin(theta2)
    theta1=np.arctan2(y,x)-np.arctan2(k2,k1)
    return (theta1,theta2)


#testing
theta1=np.radians(30)
theta2=np.radians(40)


x,y=forward_kinematics(theta1,theta2)
t1,t2=inverse_kinematics(x,y)

print(theta1,theta2)
print(t1,t2)
