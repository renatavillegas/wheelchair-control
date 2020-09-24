function sysCall_threadmain()
-- initialize objects
motorR = sim.getObjectHandle('motorRight')
motorL = sim.getObjectHandle('motorLeft')
robot = sim.getObjectHandle('wheelchair_shape')
path_handle = sim.getObjectHandle('Path')
path_plan_handle=simGetPathPlanningHandle('PathPlanningTask0')
start_dummy_handle = sim.getObjectHandle('wheelchairFrame')

--some constants 

MPI=math.pi
rad2deg=180/MPI
deg2rad=MPI/180
    
--model related measures and constants definitions
L = 0.6403 -- m (distance between wheels)
maxLinVel=0.5 --linear velocity in m/s
maxAngVel=220 --deg/sec (3.876?rad/sec)
    
wheelHandle=sim.getObjectHandle('bigWheelLeft_respondable')
res,zMin=sim.getObjectFloatParameter(wheelHandle,17)
res,zMax=sim.getObjectFloatParameter(wheelHandle,20)
R=(zMax-zMin)/2-- m (wheel radius)
pos_on_path =0
dist = 0 -- dist from the robot to the point on the path 
planstate=simSearchPath(path_plan_handle,5)
m= sim.buildMatrixQ(sim.getObjectPosition(robot, -1), sim.getObjectQuaternion(robot,-1))
while (sim.getSimulationState()~=sim.simulation_advancing_abouttostop) do

    robot_pos= sim.getObjectPosition(robot, -1)
    path_pos = sim.getPositionOnPath(path_handle, pos_on_path)
    sim.setObjectPosition(start_dummy_handle, -1, path_pos)
    sim.setObjectOrientation(start_dummy_handle, -1, path_pos)
    print("path_pos before: " .. path_pos[1] ..", ".. path_pos[2] .. ", " .. path_pos[3])

    m= sim.getObjectMatrix(robot, -1)
    a = sim.invertMatrix(m)
    if(a == -1)then 
        print("Could not invert matrix")
    end
    path_pos= sim.multiplyVector(m, path_pos)
    print ("Path pos after changing: " .. path_pos[1] ..", ".. path_pos[2] .. ", " .. path_pos[3])

    dist = math.sqrt((path_pos[1])^2 + (path_pos[2])^2)
    print("Dist = " ..dist)
    phi = math.atan2(path_pos[2], path_pos[1])

    v_des=  maxLinVel
    om_des= maxAngVel

    vl = v_des/10 - (L/2*om_des/10*MPI/180)/(R);
    vr = v_des/10 + (L/2*om_des/10*MPI/180)/(R);

    sim.setJointTargetVelocity(motorR,vr)
    sim.setJointTargetVelocity(motorL,vl)
    if(dist<0.1) then 
        pos_on_path =  pos_on_path +0.1
    else 
          sim.setJointTargetVelocity(motorR,0)
          sim.setJointTargetVelocity(motorL,0)
    end
    sim.wait(0.025, true)
 end

end

function sysCall_cleanup()
    -- Put some clean-up code here
end


-- ADDITIONAL DETAILS:
-- -------------------------------------------------------------------------
-- If you wish to synchronize a threaded loop with each simulation pass,
-- enable the explicit thread switching with 
--
-- sim.setThreadAutomaticSwitch(false)
--
-- then use
--
-- sim.switchThread()
--
-- When you want to resume execution in next simulation step (i.e. at t=t+dt)
--
-- sim.switchThread() can also be used normally, in order to not waste too much
-- computation time in a given simulation step
-- -------------------------------------------------------------------------