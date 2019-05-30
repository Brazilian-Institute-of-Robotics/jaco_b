#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import Float64MultiArray
import pybullet as p
import time
import math
from datetime import datetime
from datetime import datetime

pub = rospy.Publisher('/joint_state', Float64MultiArray)
rospy.init_node('bullet_bridge')
msg = Float64MultiArray()

clid = p.connect(p.SHARED_MEMORY)
if (clid<0):
	p.connect(p.GUI)
p.loadURDF("plane.urdf",[0,0,-0.3])
jackal = p.loadURDF("jackal_description/jackal.urdf",[0.290388,0.329902,-0.230270],[0.002328,-0.000984,0.996491,0.083659])
for i in range (p.getNumJoints(jackal)):
	print(p.getJointInfo(jackal,i))
jaco  = p.loadURDF("kinova_description/jaco.urdf", 0.193749,0.345564,0.120208,0.002327,-0.000988,0.996491,0.083659)

ob = jaco
var = (p.getNumJoints(ob))
jointPositions=[ 3.559609, 0.411182, 0.862129, 1.744441, 0.077299, -1.129685, 0.006001 ]
for jointIndex in range (p.getNumJoints(ob)):
	p.resetJointState(ob,jointIndex,jointPositions[jointIndex])


#put jaco on top of jackal

cid = p.createConstraint(jackal,-1,jaco,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0.,0.,-.32],[0,0,0,1])


baseorn = p.getQuaternionFromEuler([3.1415,0,0.3])
baseorn = [0,0,0,1]
#[0, 0, 0.707, 0.707]

#p.resetBasePositionAndOrientation(jaco,[0,0,0],baseorn)#[0,0,0,1])
jacoEndEffectorIndex = 6
numJoints = p.getNumJoints(jaco)
if (numJoints!=7):
	exit()
	

#lower limits for null space
ll=[-.967,-2	,-2.96,0.19,-2.96,-2.09,-3.05]
#upper limits for null space
ul=[.967,2	,2.96,2.29,2.96,2.09,3.05]
#joint ranges for null space
jr=[5.8,4,5.8,4,5.8,4,6]
#restposes for null space
rp=[0,0,0,0.5*math.pi,0,-math.pi*0.5*0.66,0]
#joint damping coefficents
jd=[0.1,0.1,0.1,0.1,0.1,0.1,0.1]

for i in range (numJoints):
	p.resetJointState(jaco,i,rp[i])

p.setGravity(0,0,-10)
t=0.
prevPose=[0,0,0]
prevPose1=[0,0,0]
hasPrevPose = 0
useNullSpace = 0

useOrientation =0 
#If we set useSimulation=0, it sets the arm pose to be the IK result directly without using dynamic control.
#This can be used to test the IK result accuracy.
useSimulation = 0
useRealTimeSimulation = 1
p.setRealTimeSimulation(useRealTimeSimulation)
#trailDuration is duration (in seconds) after debug lines will be removed automatically
#use 0 for no-removal
trailDuration = 15
basepos =[0,0,0]	
ang = 0
ang=0

def accurateCalculateInverseKinematics( jaco, endEffectorId, targetPos, threshold, maxIter):
	closeEnough = False
	iter = 0
	dist2 = 1e30
	while (not closeEnough and iter<maxIter):
		jointPoses = p.calculateInverseKinematics(jaco,jacoEndEffectorIndex,targetPos)
		for i in range (numJoints):
			p.resetJointState(jaco,i,jointPoses[i])
		ls = p.getLinkState(jaco,jacoEndEffectorIndex)	
		newPos = ls[4]
		diff = [targetPos[0]-newPos[0],targetPos[1]-newPos[1],targetPos[2]-newPos[2]]
		dist2 = (diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2])
		closeEnough = (dist2 < threshold)
		iter=iter+1
	print ("Num iter: "+str(iter) + "threshold: "+str(dist2))
	return jointPoses


wheels=[2,3,4,5]
#(2, b'front_left_wheel', 0, 7, 6, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'front_left_wheel_link')
#(3, b'front_right_wheel', 0, 8, 7, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'front_right_wheel_link')
#(4, b'rear_left_wheel', 0, 9, 8, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'rear_left_wheel_link')
#(5, b'rear_right_wheel', 0, 10, 9, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'rear_right_wheel_link')
wheelVelocities=[0,0,0,0]
wheelDeltasTurn=[1,-1,1,-1]
wheelDeltasFwd=[1,1,1,1]
while not rospy.is_shutdown():
	keys = p.getKeyboardEvents()
	shift = 0.01
	wheelVelocities=[0,0,0,0]
	speed = 5.0
	for k in keys:
		if ord('s') in keys:
				p.saveWorld("state.py")
		if ord('a') in keys:
			basepos =  basepos=[basepos[0],basepos[1]-shift,basepos[2]]
		if ord('d') in keys:
                        basepos =  basepos=[basepos[0],basepos[1]+shift,basepos[2]]

		if p.B3G_LEFT_ARROW in keys:
			for i in range(len(wheels)):
				wheelVelocities[i] =  wheelVelocities[i] - speed*wheelDeltasTurn[i]
		if p.B3G_RIGHT_ARROW in keys:
			for i in range(len(wheels)):
				wheelVelocities[i] =  wheelVelocities[i] +speed*wheelDeltasTurn[i]
		if p.B3G_UP_ARROW in keys:
			for i in range(len(wheels)):
				wheelVelocities[i] = wheelVelocities[i] + speed*wheelDeltasFwd[i]
		if p.B3G_DOWN_ARROW in keys:
			for i in range(len(wheels)):
				wheelVelocities[i] = wheelVelocities[i]  -speed*wheelDeltasFwd[i]
    
	baseorn = p.getQuaternionFromEuler([0,0,ang])
	for i in range(len(wheels)):
		p.setJointMotorControl2(jackal,wheels[i],p.VELOCITY_CONTROL,targetVelocity=wheelVelocities[i], force=1000)
	#p.resetBasePositionAndOrientation(jaco,basepos,baseorn)#[0,0,0,1])
	if (useRealTimeSimulation):
		t = time.time()#(dt, micro) = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f').split('.')
		#t = (dt.second/60.)*2.*math.pi
	else:
		t=t+0.001
	
	if (useSimulation and useRealTimeSimulation==0):
		p.stepSimulation()
	
	for i in range (1):
		#pos = [-0.4,0.2*math.cos(t),0.+0.2*math.sin(t)]
		pos = [0.2*math.cos(t),0,0.+0.2*math.sin(t)+0.7]
		#pos = [0.5,0.5,0.8]
		#end effector points down, not up (in case useOrientation==1)
		orn = p.getQuaternionFromEuler([0,-math.pi,0])
		
		if (useNullSpace==1):
			if (useOrientation==1):
				jointPoses = p.calculateInverseKinematics(jaco,jacoEndEffectorIndex,pos,orn,ll,ul,jr,rp)
			else:
				jointPoses = p.calculateInverseKinematics(jaco,jacoEndEffectorIndex,pos,lowerLimits=ll, upperLimits=ul, jointRanges=jr, restPoses=rp)
		else:
			if (useOrientation==1):
				jointPoses = p.calculateInverseKinematics(jaco,jacoEndEffectorIndex,pos,orn,jointDamping=jd)
			else:
				threshold =0.001
				maxIter = 100
				jointPoses = accurateCalculateInverseKinematics(jaco,jacoEndEffectorIndex,pos, threshold, maxIter)
	
		if (useSimulation):
			for i in range (numJoints):
				p.setJointMotorControl2(bodyIndex=jaco,jointIndex=i,controlMode=p.POSITION_CONTROL,targetPosition=jointPoses[i],targetVelocity=0,force=500,positionGain=1,velocityGain=0.1)
		else:
			#reset the joint state (ignoring all dynamics, not recommended to use during simulation)
			for i in range (numJoints):
				p.resetJointState(jaco,i,jointPoses[i])

	ls = p.getLinkState(jaco,jacoEndEffectorIndex)	
	if (hasPrevPose):
		p.addUserDebugLine(prevPose,pos,[0,0,0.3],1,trailDuration)
		p.addUserDebugLine(prevPose1,ls[4],[1,0,0],1,trailDuration)
	prevPose=pos
	prevPose1=ls[4]
	hasPrevPose = 1		
	p.getCameraImage(320,200, flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX, renderer=p.ER_BULLET_HARDWARE_OPENGL)
	joint_velocity = [j[1] for j in p.getJointStates(jaco, range(7))]
	joint_position = [j[0] for j in p.getJointStates(jaco, range(7))]
	msg.data = joint_velocity
	pub.publish(msg)
	print(msg.data[1])
	#msg.data[2]
	#input("Press Enter to continue...")

