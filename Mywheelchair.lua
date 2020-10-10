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

--backup 
while (sim.getSimulationState()~=sim.simulation_advancing_abouttostop 
        and planstate ~=0) do
    -- follow the path
    robot_pos= sim.getObjectPosition(robot, -1)
    robot_ori= sim.getObjectOrientation(robot, -1)
    path_pos = sim.getPositionOnPath(path_handle, pos_on_path) -- pos_on_path [0-1]
    path_ori = sim.getObjectOrientation(path_handle, -1)
--    print("path_pos before: " .. path_pos[1] ..", ".. path_pos[2] .. ", " .. path_pos[3])
    m= sim.buildMatrix(robot_pos, robot_ori)
--    print("m before =" .. m[5])
    a = sim.invertMatrix(m)
--    print("m after"..m[5])
    if(a == -1)then 
        print("Could not invert matrix")
        break
    end
    path_pos= sim.multiplyVector(m, path_pos) -- now pos is relative to the robot. 
--    print ("Path pos after changing: " .. path_pos[1] ..", ".. path_pos[2] .. ", " .. path_pos[3])

    dist = math.sqrt((path_pos[1])^2 + (path_pos[2])^2) -- distance between the robot and the point on path
    print("Dist = " ..dist)
    phi = math.atan2(path_pos[2], path_pos[1]) -- angular position of the path in relation to the robot

    v_des=  maxLinVel
    om_des= maxAngVel
    --v_r= (v_des + L*om_des)
    --v_l = (v_des -L*om_des)

--    vl = v_des/10 - (L/15*om_des*MPI/180)/(R);
--    vr = v_des/10 + (L/15*om_des*MPI/180)/(R);

--    sim.setJointTargetVelocity(motorR,vr)
--    sim.setJointTargetVelocity(motorL,vl)
    -- Moviment control. The ideia is applying the differential robot control definition. 

    if(dist<0.1) then 
        pos_on_path =  pos_on_path +0.5
    else
    -- by knowing the angle, we know how to update the velocities. 
        a=math.atan2(path_pos[2],path_pos[1])
        print("ang = " ..a)
        if (a>=0)and(a<math.pi*0.5) then -- [0-90]: turn the robot clockwise
            vr=nominalVelocity
            vl=nominalVelocity*(1-2*a/(math.pi*0.5))
            print('[0-90]: turn the robot clockwise')
            
        end
        if (a>=math.pi*0.5) then --[90-180]: turn the robot anticlockwise
                vl=-nominalVelocity
                vr=nominalVelocity*(1-2*(a-math.pi*0.5)/(math.pi*0.5))
                print('[90-180]: turn the robot anticlockwise')
        end
        if (a<0)and(a>-math.pi*0.5) then --[-90-0]: turn the robot anticlockwise
                vl=nominalVelocity
                vr=nominalVelocity*(1+2*a/(math.pi*0.5))
                print('[-90-0]: turn the robot anticlockwise')
        end
        if (a<=-math.pi*0.5) then --[-180 -90]: turn the robot clockwise 
                vr=-nominalVelocity
                vl=nominalVelocity*(1+2*(a+math.pi*0.5)/(math.pi*0.5))
                print('[-180 -90]: turn the robot clockwise')
        end
        omega=R*(vr-vl)/L
        vel=(vr+vl)/2
        -- update velocities
        vl_new = vel - (L/2*omega*MPI/180.)/(R);
        vr_new = vel + (L/2*omega*MPI/180.)/(R);
        sim.setJointTargetVelocity(motorR,vr_new)
        sim.setJointTargetVelocity(motorL,vl_new)
  -- adjusting the orientation. Verify if we're close to the target. If we are, adjust the orientation
        reachError = 0.01
        posTg=sim.getObjectPosition(goal,-1)
        rotTg=sim.getObjectOrientation(goal,-1)
        posRobot=sim.getObjectPosition(robot,-1)
        rotRobot=sim.getObjectOrientation(robot,-1)
        if math.abs(posTg[1]-posRobot[1])< reachError and math.abs(posTg[2]-posRobot[2])< reachError then
            print ('Robot close to the target.')
        end 
    end    
    sim.wait(0.05, true)
    end