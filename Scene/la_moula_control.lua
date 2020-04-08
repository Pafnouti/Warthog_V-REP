function subscriber_speed_motorArD_callback(msg)
	spd = msg.data
	sim.setJointTargetVelocity(motorArD, spd)
	sim.addStatusbarMessage('speed_motorArD subscriber received : spd ='..spd)
end

function subscriber_speed_motorArG_callback(msg)
	spd = msg.data
	sim.setJointTargetVelocity(motorArG, spd)
	sim.addStatusbarMessage('speed_motorArG subscriber received : spd ='..spd)
end

function subscriber_speed_motorAvD_callback(msg)
	spd = msg.data
	sim.setJointTargetVelocity(motorAvD, spd)
	sim.addStatusbarMessage('speed_motorAvD subscriber received : spd ='..spd)
end

function subscriber_speed_motorAvG_callback(msg)
	spd = msg.data
	sim.setJointTargetVelocity(motorAvG, spd)
	sim.addStatusbarMessage('speed_motorAvG subscriber received : spd ='..spd)
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
	  subscriber1 = simROS.subscribe('/vrep_speed_motorArD', 'std_msgs/Float32', 'subscriber_speed_motorArD_callback')
	  subscriber2 = simROS.subscribe('/vrep_speed_motorArG', 'std_msgs/Float32', 'subscriber_speed_motorArG_callback')
	  subscriber3 = simROS.subscribe('/vrep_speed_motorAvD', 'std_msgs/Float32', 'subscriber_speed_motorAvD_callback')
	  subscriber4 = simROS.subscribe('/vrep_speed_motorAvG', 'std_msgs/Float32', 'subscriber_speed_motorAvG_callback')
	end
end

function sysCall_actuation()
	-- Send an updated simulation time message, and send the transform of the object attached to this script:
	if rosInterfacePresent then
	  -- publish time and pose topics
	  simROS.publish(publisher1, {data=sim.getSimulationTime()})
	  simROS.publish(publisher2, getPose("chassis"))
	  -- send a TF
	  simROS.sendTransform(getTransformStamped(objectHandle, objectName, -1, 'world'))
	  -- To send several transforms at once, use simROS.sendTransforms instead
	end
end

function sysCall_cleanup()
	-- Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
	if rosInterfacePresent then
		simROS.shutdownPublisher(publisher1)
		simROS.shutdownPublisher(publisher2)
		simROS.shutdownSubscriber(subscriber1)
		simROS.shutdownSubscriber(subscriber2)
		simROS.shutdownSubscriber(subscriber3)
		simROS.shutdownSubscriber(subscriber4)
	end
end