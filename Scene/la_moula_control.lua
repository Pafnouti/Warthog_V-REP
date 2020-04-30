print "Working..."

function subscriber_speed_motorArD_callback(msg)
	spd = msg.data
	sim.setJointTargetVelocity(motorArD, -spd)
	sim.addStatusbarMessage('speed_motorArD subscriber received : spd ='..spd)
end

function subscriber_speed_motorArG_callback(msg)
	spd = msg.data
	sim.setJointTargetVelocity(motorArG, -spd)
	sim.addStatusbarMessage('speed_motorArG subscriber received : spd ='..spd)
end

function subscriber_speed_motorAvD_callback(msg)
	spd = msg.data
	sim.setJointTargetVelocity(motorAvD, -spd)
	sim.addStatusbarMessage('speed_motorAvD subscriber received : spd ='..spd)
end

function subscriber_speed_motorAvG_callback(msg)
	spd = msg.data
	sim.setJointTargetVelocity(motorAvG, -spd)
	sim.addStatusbarMessage('speed_motorAvG subscriber received : spd ='..spd)
end

function subscriber_buoyancyForceAvG_callback(msg)
	AvG_force = {msg.linear.x, msg.linear.y, msg.linear.z}

end

function subscriber_buoyancyForceAvD_callback(msg)
	f = {msg.linear.x, msg.linear.y, msg.linear.z}
	pos = {0, 0, 0}
	body = sim.getObjectHandle("roueAvD")
	sim.addForce(body, pos, f)
end

function subscriber_buoyancyForceArG_callback(msg)
	f = {msg.linear.x, msg.linear.y, msg.linear.z}
	pos = {0, 0, 0}
	body = sim.getObjectHandle("roueArG")
	sim.addForce(body, pos, f)
end

function subscriber_buoyancyForceArD_callback(msg)
	f = {msg.linear.x, msg.linear.y, msg.linear.z}

	
	body = sim.getObjectHandle("roueArD")
	sim.addForce(body, pos, f)
end

function getPose(objectName)
	-- This function get the object pose at ROS format geometry_msgs/Pose
	objectHandle = sim.getObjectHandle(objectName)
	relTo = -1
	p = sim.getObjectPosition(objectHandle,relTo)
	o = sim.getObjectQuaternion(objectHandle,relTo)
	return {
	  		position={x=p[1],y=p[2],z=p[3]},
	  		orientation={x=o[1],y=o[2],z=o[3],w=o[4]}
		   }
end

function getTransformStamped(objHandle,name,relTo,relToName)
	-- This function retrieves the stamped transform for a specific object
	t = sim.getSystemTime()
	p = sim.getObjectPosition(objHandle,relTo)
	o = sim.getObjectQuaternion(objHandle,relTo)
	return {
	 		header={
	 				stamp=t,
	 				frame_id=relToName
	  				},
	  		child_frame_id=name,
	  		transform={
	 					translation={x=p[1],y=p[2],z=p[3]},
	 					rotation={x=o[1],y=o[2],z=o[3],w=o[4]}
	  				  }
			}
end

function sysCall_init()
	-- The child script initialization
	objectName = "chassis"
	objectHandle = sim.getObjectHandle(objectName)
	-- get left and right motors handles
	motorArD = sim.getObjectHandle("motorArD")
	motorAvD = sim.getObjectHandle("motorAvD")
	motorArG = sim.getObjectHandle("motorArG")
	motorAvG = sim.getObjectHandle("motorAvG")
	pivotD = sim.getObjectHandle("pivotD")
	pivotG = sim.getObjectHandle("pivotG")	

	rosInterfacePresent = simROS
	-- Prepare the publishers and subscribers :
	if rosInterfacePresent then
	  publisher1 = simROS.advertise('/simulationTime', 'std_msgs/Float32')
	  publisher2 = simROS.advertise('/pose', 'geometry_msgs/Pose')
	  publisher3 = simROS.advertise('/vrep_wheel_AvG', 'geometry_msgs/Pose')
	  publisher4 = simROS.advertise('/vrep_wheel_AvD', 'geometry_msgs/Pose')
	  publisher5 = simROS.advertise('/vrep_wheel_ArG', 'geometry_msgs/Pose')
	  publisher6 = simROS.advertise('/vrep_wheel_ArD', 'geometry_msgs/Pose')
	  subscriber1 = simROS.subscribe('/vrep_speed_motorArD', 'std_msgs/Float32', 'subscriber_speed_motorArD_callback')
	  subscriber2 = simROS.subscribe('/vrep_speed_motorArG', 'std_msgs/Float32', 'subscriber_speed_motorArG_callback')
	  subscriber3 = simROS.subscribe('/vrep_speed_motorAvD', 'std_msgs/Float32', 'subscriber_speed_motorAvD_callback')
	  subscriber4 = simROS.subscribe('/vrep_speed_motorAvG', 'std_msgs/Float32', 'subscriber_speed_motorAvG_callback')
	  subscriber5 = simROS.subscribe('/buoyancyForceAvG', 'geometry_msgs/Twist', 'subscriber_buoyancyForceAvG_callback')
	  subscriber6 = simROS.subscribe('/buoyancyForceAvD', 'geometry_msgs/Twist', 'subscriber_buoyancyForceAvD_callback')
	  subscriber7 = simROS.subscribe('/buoyancyForceArG', 'geometry_msgs/Twist', 'subscriber_buoyancyForceArG_callback')
	  subscriber8 = simROS.subscribe('/buoyancyForceArD', 'geometry_msgs/Twist', 'subscriber_buoyancyForceArD_callback')
	end

	bodyRAvG = sim.getObjectHandle("roueAvG")
	bodyRAvD = sim.getObjectHandle("roueAvD")
	bodyRArG = sim.getObjectHandle("roueArG")
	bodyRArD = sim.getObjectHandle("roueArD")
	pos = {0, 0, 0}

end

function sysCall_actuation()
	-- Send an updated simulation time message, and send the transform of the object attached to this script:
	if rosInterfacePresent then
	  -- publish time and pose topics
	  simROS.publish(publisher1, {data=sim.getSimulationTime()})
	  simROS.publish(publisher2, getPose("chassis"))
	  simROS.publish(publisher3, getPose("roueAvG"))
	  simROS.publish(publisher4, getPose("roueAvD"))
	  simROS.publish(publisher5, getPose("roueArG"))
	  simROS.publish(publisher6, getPose("roueArD"))
	  -- send a TF
	  simROS.sendTransform(getTransformStamped(objectHandle, objectName, -1, 'world'))
	  -- To send several transforms at once, use simROS.sendTransforms instead
	  sim.addForce(bodyRAvG, pos, AvG_force)
	  sim.addForce(bodyRAvD, pos, AvD_force)
	  sim.addForce(bodyRArG, pos, ArG_force)
	  sim.addForce(bodyRArD, pos, ArD_force)

	end
end

function sysCall_cleanup()
	-- Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
	if rosInterfacePresent then
		simROS.shutdownPublisher(publisher1)
		simROS.shutdownPublisher(publisher2)
		simROS.shutdownPublisher(publisher3)
		simROS.shutdownPublisher(publisher4)
		simROS.shutdownPublisher(publisher5)
		simROS.shutdownPublisher(publisher6)
		simROS.shutdownSubscriber(subscriber1)
		simROS.shutdownSubscriber(subscriber2)
		simROS.shutdownSubscriber(subscriber3)
		simROS.shutdownSubscriber(subscriber4)
		simROS.shutdownSubscriber(subscriber5)
		simROS.shutdownSubscriber(subscriber6)
		simROS.shutdownSubscriber(subscriber7)
		simROS.shutdownSubscriber(subscriber8)
	end
end
