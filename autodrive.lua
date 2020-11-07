------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and later: 
if (sim_call_type==sim.syscb_init) then 
    sim.setScriptAttribute(sim.handle_self,sim.childscriptattribute_automaticcascadingcalls,false) 
end 
if (sim_call_type==sim.syscb_cleanup) then 
 
end 
if (sim_call_type==sim.syscb_sensing) then 
    sim.handleChildScripts(sim_call_type) 
end 
if (sim_call_type==sim.syscb_actuation) then 
    if not firstTimeHere93846738 then 
        firstTimeHere93846738=0 
    end 
    sim.setScriptAttribute(sim.handle_self,sim.scriptattribute_executioncount,firstTimeHere93846738) 
    firstTimeHere93846738=firstTimeHere93846738+1 
 
------------------------------------------------------------------------------ 
 
 
function setAngle(ang)     --set ang in ]-PI, PI]
                        -- input is an angle in radian and return the angle in rad between ]-pi,pi]
    while ang>math.pi do 
        ang=ang-(2*math.pi)--print("while!!")
    end
    while ang<-math.pi do 
        ang=ang+(2*math.pi)--print("while!!")
    end
    return ang
end

function normAngle(ang) --get an angle in rad and returns it in [-pi, pi]
    while ang>math.pi do ang=ang-2*math.pi  end
    while ang<=-math.pi do ang=ang+2*math.pi  end
    return ang
end

function getAngularDifference(goalAngle,startAngle)
    dx=goalAngle-startAngle
    if (dx>=0) then
        dx=math.mod(dx+math.pi,2*math.pi)-math.pi
    else
        dx=math.mod(dx-math.pi,2*math.pi)+math.pi
    end
    return(dx)
end


if (sim.getScriptExecutionCount()==0) then
    rad2deg=180/math.pi
    deg2rad=math.pi/180
    MPI=math.pi
    
    goal=sim.getObjectHandle('p3_goalPos')
    lmotorHandle=sim.getObjectHandle('motorLeft')
    rmotorHandle=sim.getObjectHandle('motorRight')
    robot=sim.getObjectHandle('wheelchairFrame')--body')
    vel=0
    omega=0
    tolHeading=5*deg2rad -- tolerance heading in deg
    tolPosition=0.2 -- reaching position tolerance in meter
    gainHeading=20--40 
    gainSpeed=1--10
    maxvel= 0.5            -- max linear velocity
    maxomega=20*deg2rad -- max angular velocity


    kro= 0.6--0.4--1 
    kalpha=345/823
    kbeta=-0.6 -- -0.6-- -2/11
    --alpha=0
    
    newTarget=false
    tgpos=sim.getObjectPosition(robot,-1)
    tgori=sim.getObjectOrientation(robot,-1)
    tgpos[3]=tgpos[3]+2.7000e-02 --shift in z to make the goal way up to the current robot pos
                          --to change if policy change i.e. goal disappear when not in auto mode
    sim.setObjectPosition(goal,-1,tgpos)
    sim.setObjectOrientation(goal,-1,tgori)
    tgpos_old=tgpos
    tgori_old=tgori

    pathHandle=sim.getObjectHandle('p3_pathPlanningPath')
    pathPlanningHandle=simGetPathPlanningHandle('PathPlanningTask')
    collidableForPathPlanning=sim.getObjectHandle('p3CollidableForPathPlanning')
    obstacles=sim.getCollectionHandle('p3ObstacleCollection')

    randomModeUntilTime=0
    pathCalculated=0 -- 0=not calculated, 1=beeing calculated, 2=calculated
    tempPathSearchObject=-1
    currentPosOnPath=0

    --model related measures and constants definitions
    wheelHandle=sim.getObjectHandle('bigWheelLeft_respondable')
    res,zMin=sim.getObjectFloatParameter(wheelHandle,17)
    res,zMax=sim.getObjectFloatParameter(wheelHandle,20)
    R=(zMax-zMin)/2-- m (wheel radius)
    local lw=sim.getObjectPosition(wheelHandle,-1)
    local rw=sim.getObjectPosition(sim.getObjectHandle('bigWheelRight_respondable'),-1)
    L = math.sqrt((lw[1]-rw[1])^2+(lw[2]-rw[2])^2) -- m (distance between wheels)

    nominalVelocity=10--0.996 --EVENTUALLY GETTING FROM THE SLIDER
    reachError=0.01

--PID rotation related init
prev_error=0
integral=0
oldtime=sim.getSimulationTime()
Kp=0.1 --mm/s
Ki=0.001--0--0.09
Kd=0.09--0.9--30

--to remove: for test
pathCalculated=0
------
end
    
sim.handleChildScripts(sim_call_type)
    
    
--mode=sim.readCustomDataBlock(robot,'mode')
mode='ptg'
if mode=='ptg' or mode =='lasernav' then

    -- main code here
    -- checking if the target position changed
    targetP=sim.getObjectPosition(goal,-1)
    vv={targetP[1]-tgpos[1],targetP[2]-tgpos[2]}
    if (math.sqrt(vv[1]*vv[1]+vv[2]*vv[2])>0.01) then
        pathCalculated=0 -- We have to recompute the path since the target position has moved
        tgpos[1]=targetP[1] ^
        tgpos[2]=targetP[2]
        --newTarget=true
        closeToTarget=false
    end
    currentTime=sim.getSimulationTime()

if closeToTarget==false then --we are far from the end or we have a new target so we create a path (if any yet) 
                              --and follow it    
    rightV=0
    leftV=0
    if (pathCalculated==0) then
        -- search for a path
        if (sim.checkCollision(obstacles,collidableForPathPlanning)~=1) then -- Make sure we are not colliding when starting to compute the path!
            if (tempPathSearchObject~=-1) then     
                simPerformPathSearchStep(tempPathSearchObject,true) -- delete any previous temporary path search object
            end
            orientation=sim.getObjectOrientation(robot,-1)
            sim.setObjectOrientation(robot,-1,{0,0,orientation[3]}) -- Temporarily set the robot's orientation to be in the plane (the robot can slightly tilt back and forth)
            tempPathSearchObject=simInitializePathSearch(pathPlanningHandle,10,0.03) -- search for a maximum of 10 seconds
            sim.setObjectOrientation(robot,-1,orientation) -- Set the previous robot's orientation
            if (tempPathSearchObject~=-1) then
                pathCalculated=1
            else sim.addStatusbarMessage('failed to find a path')
            end
        else
        if (currentTime>randomModeUntilTime) then
                randomModeUntilTime=currentTime+2 -- 2 seconds in random direction
                randomVLeft=(-1+math.random()*2)*nominalVelocity
                randomVRight=(-1+math.random()*2)*nominalVelocity
            end
        end
    else
        if (pathCalculated==1) then
            r=simPerformPathSearchStep(tempPathSearchObject,false)
            if (r<1) then
                if (r~=-2) then
                    pathCalculated=0 -- path search failed, try again from the beginning
                    tempPathSearchObject=-1
                end
            else
                pathCalculated=2 -- we found a path
                currentPosOnPath=0
                tempPathSearchObject=-1
            end
        else
            l=sim.getPathLength(pathHandle)
            r=sim.getObjectPosition(robot,-1)
            while true do
                p=sim.getPositionOnPath(pathHandle,currentPosOnPath/l)
                d=math.sqrt((p[1]-r[1])*(p[1]-r[1])+(p[2]-r[2])*(p[2]-r[2]))
                if (d>0.2)or(currentPosOnPath>=l) then
                    break
                end
                currentPosOnPath=currentPosOnPath+0.01
            end
            m=sim.getObjectMatrix(robot,-1)
            m=simGetInvertedMatrix(m)
            p=sim.multiplyVector(m,p)
            -- Now p is relative to the robot
            a=math.atan2(p[2],p[1])
            if (a>=0)and(a<math.pi*0.5) then
                rightV=nominalVelocity
                leftV=nominalVelocity*(1-2*a/(math.pi*0.5))
            
            end
            if (a>=math.pi*0.5) then
                leftV=-nominalVelocity
                rightV=nominalVelocity*(1-2*(a-math.pi*0.5)/(math.pi*0.5))
            end
            if (a<0)and(a>-math.pi*0.5) then
                leftV=nominalVelocity
                rightV=nominalVelocity*(1+2*a/(math.pi*0.5))
            end
            if (a<=-math.pi*0.5) then
                rightV=-nominalVelocity
                leftV=nominalVelocity*(1+2*(a+math.pi*0.5)/(math.pi*0.5))
            end
                    
        end
    end
    
    omega=R*(rightV-leftV)/L
    vel=(rightV+leftV)/2


    posTg=sim.getObjectPosition(goal,-1)
    rotTg=sim.getObjectOrientation(goal,-1)
    posRobot=sim.getObjectPosition(robot,-1)
    rotRobot=sim.getObjectOrientation(robot,-1)
    if math.abs(posTg[1]-posRobot[1])< reachError and math.abs(posTg[2]-posRobot[2])< reachError then
        closeToTarget=true -- we then will enter the adjusting orientation only mode
    end

print(sim.getSimulationTime(),'autodrive: following path')

else -- linear velocity =0, we just adjust the rotation 
print(sim.getSimulationTime(),'autodrive: adjusting orientation')
    vel=0

    tgori=sim.getObjectOrientation(goal,-1)
    curori=sim.getObjectOrientation(robot,-1)
    
    dt=sim.getSimulationTime()-oldtime
    dth=getAngularDifference(tgori[3],curori[3])
    
    if math.abs(dth)>0.5*deg2rad then
        
        --calculate the spinning velocity
        if dt>0.05 then
    
            error=getAngularDifference(tgori[3],curori[3])
            integral=integral+error*dt
            derivative=(error- prev_error)/dt
    
            omega = Kp*error +Ki*integral +Kd * derivative
    
            if omega> maxomega then omega=maxomega end
            if omega< -maxomega then omega=-maxomega end
            prev_error=error
            oldtime=sim.getSimulationTime()
        end
    else 
        prev_error=0
        integral=0
        omega=0
        dt=0
        startTimer=sim.getSimulationTime()
    end

end    
    

    













 

--[[ controller in Siegward

--sim.addStatusbarMessage('autodrive mode: '..mode)
    tgpos=sim.getObjectPosition(goal,-1)
    tgori=sim.getObjectOrientation(goal,-1)
    if tgpos_old[1]~=tgpos[1] or tgpos_old[2]~=tgpos[2] or  tgori_old[1]~=tgori[1] or tgori_old[2]~=tgori[2] then 
        --here a new goal has been defined
        newTarget=true
    --else
    --    newTarget=false
    end
    tgpos_old=tgpos
    tgori_old=tgori
    
    xg=tgpos[1]
    yg=tgpos[2]
    thetag=tgori[3]
    
    curpos=sim.getObjectPosition(robot,-1)
    curori=sim.getObjectOrientation(robot,-1)
    xr=curpos[1]      --current x robot
    yr=curpos[2]      --current y robot
    thetar=curori[3] --current theta robot
    
    ro=math.sqrt((xg-xr)^2+(yg-yr)^2)
    alpha=setAngle(-thetar + math.atan2(yg-yr,xg-xr))
    beta=setAngle(-math.atan2(yg-yr,xg-xr)+ thetag)
    
    
    --print('ctrl loop:',math.abs(ro)>tolPosition , math.abs(thetag-thetar)>tolHeading)
    --print('ro:',math.abs(ro),'dtheta',setAngle(math.abs(setAngle(thetag)-setAngle(thetar)))*180/math.pi)
    if math.abs(ro)>tolPosition or setAngle(math.abs(setAngle(thetag)-setAngle(thetar)))>tolHeading then
        newTarget=false
    
        --print('\n fresh alpha',alpha)
--    while alpha>math.pi do 
--            alpha=alpha-(2*math.pi) 
--            --print('alpha SUP PI')
--        end
--        while alpha<-math.pi do 
--            alpha=alpha+(2*math.pi) 
--            --print('alpha INF PI')
--        end
--

        alpha=setAngle(alpha)
    --    print(math.abs(thetag-thetar))
        --print('\n')
        --print('ro',ro,'alpha',alpha*rad2deg,'beta',beta*rad2deg,'thetar',thetar*rad2deg,'thetag',thetag*rad2deg)
    
        --print('       alpha (deg)',alpha*rad2deg, 'beta', beta*rad2deg)
    
        if alpha <-MPI/2 or alpha >=MPI/2 then
            ro=-ro
            alpha=alpha+MPI
            beta=beta+MPI
    
            alpha=setAngle(alpha)
            --while alpha>math.pi do alpha=alpha-(2*math.pi)end
            --while alpha<-math.pi do alpha=alpha+(2*math.pi)end
            --print('in I2: alpha (deg)',alpha*rad2deg, 'beta', beta*rad2deg)
    
        end
    
        vel=kro*ro
        omega=kalpha*alpha + kbeta*beta
    
        --sim.addStatusbarMessage('vel '..vel..' omega '..omega..' omega(deg/s) '..omega*rad2deg)
        --print('vel',vel,'omega',omega,'omega(deg/s)',omega*rad2deg)
    
    else
    
        if newTarget==false then --robot is paused
            --print('ROBOT IS STOPPED')
            omega=0
            vel=0
            --ro=1000
            --newTarget=true
        end
    
    end
--]]


--print('autodrive',vel, omega)
    -- send vel and omega to robot control script
    if vel>maxvel then vel=maxvel end
    if vel<-maxvel then vel=-maxvel end

    if omega>maxomega then omega=maxomega end
    if omega<-maxomega then omega=-maxomega end

    data=sim.packFloatTable({vel,omega*rad2deg})
    sim.setStringSignal('autodrive',data)

end


if (sim.getSimulationState()==sim.simulation_advancing_lastbeforestop) then
    print('\n\nEND autodrive')
end 

------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and later: 
end 
------------------------------------------------------------------------------ 


------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and later: 
if (sim_call_type==sim.syscb_init) then 
    sim.setScriptAttribute(sim.handle_self,sim.childscriptattribute_automaticcascadingcalls,false) 
end 
if (sim_call_type==sim.syscb_cleanup) then 
 
end 
if (sim_call_type==sim.syscb_sensing) then 
    sim.handleChildScripts(sim_call_type) 
end 
if (sim_call_type==sim.syscb_actuation) then 
    if not firstTimeHere93846738 then 
        firstTimeHere93846738=0 
    end 
    sim.setScriptAttribute(sim.handle_self,sim.scriptattribute_executioncount,firstTimeHere93846738) 
    firstTimeHere93846738=firstTimeHere93846738+1 
 
------------------------------------------------------------------------------ 
function goalChanged() -- return true if the goal position changed else false
    if math.abs(tgpos_old[1]-tgpos[1])>0.001 or
       math.abs(tgpos_old[2]-tgpos[2])>0.001 or
       math.abs(tgori_old[3]-tgori[3])>0.1*deg2rad then
        return true
    else
        return false
    end
end
 
function setAngle(ang)     --set ang in ]-PI, PI]

                        -- input is an angle in radian and return the angle in rad between ]-pi,pi]
    while ang>math.pi do 
        ang=ang-(2*math.pi)--print("while!!")
    end
    while ang<-math.pi do 
        ang=ang+(2*math.pi)--print("while!!")
    end
    return ang
end
    
if (sim.getScriptExecutionCount()==0) then
-- initialize objects
motorR = sim.getObjectHandle('motorRight')
motorL = sim.getObjectHandle('motorLeft')
robot = sim.getObjectHandle('wheelchair_shape')
path_handle = sim.getObjectHandle('Path')
path_plan_handle=simGetPathPlanningHandle('PathPlanningTask0')
start_dummy_handle = sim.getObjectHandle('wheelchairFrame')
goal = sim.getObjectHandle('p3_goalPos')
wheelHandle=sim.getObjectHandle('bigWheelLeft_respondable')
res,zMin=sim.getObjectFloatParameter(wheelHandle,17)
res,zMax=sim.getObjectFloatParameter(wheelHandle,20)
R=(zMax-zMin)/2-- m (wheel radius)
next_point =0
dist = 0 -- dist from the robot to the point on the path 
--some constants 

MPI=math.pi
rad2deg=180/MPI
deg2rad=MPI/180
    
--model related measures and constants definitions
L = 0.6403 -- m (distance between wheels)
maxLinVel=0.5 --linear velocity in m/s
maxAngVel=220 --deg/sec (3.876¨rad/sec)
maxomega=20*deg2rad 
nominalVelocity=2.5--0.996 --EVENTUALLY GETTING FROM THE SLIDER

-- Path panning
planstate=simSearchPath(path_plan_handle,10)
if (planstate == 0) then
    print('Could not find the path!!'..planstate)
end
-- Follow the path. 
l=sim.getPathLength(path_handle)
path_pos = sim.getPositionOnPath(path_handle, next_point) -- pos_on_path [0-1]
path_ori = sim.getOrientationOnPath(path_handle, next_point)
sim.setObjectPosition(start_dummy_handle, -1, path_pos) -- put the robot in the start of the path
sim.setObjectOrientation(start_dummy_handle, -1, path_ori) -- ajust the robot orientation on the start
next_point = next_point + 0.1
sim.setObjectPosition(goal, -1,sim.getPositionOnPath(path_handle, next_point))
sim.setObjectOrientation(goal, -1, sim.getOrientationOnPath(path_handle, next_point))

-- First, adjust the orientation of the robot to make the path follow simplier. 
--angular postion of the path. 
dist_angle = sim.getObjectOrientation(path_handle, robot)




-- PID Control linear velocity. 
prev_error=0
integral=0
oldtime=sim.getSimulationTime()
Kp=0.15 --mm/s
Ki=0.00--0--0.09
Kd=0.08--0.9--30
vcorr =0
curPos = sim.getObjectPosition(robot, -1)
targetPos = sim.getObjectPosition(goal, robot)
dist = math.sqrt(targetPos[1]*targetPos[1]+targetPos[2]*targetPos[2])

--print("dist = ".. dist)
while(next_point<1) do
    

    print("next_point = "..next_point)
-- current position/distance
    curPos = sim.getObjectPosition(robot, -1)
    curOri = sim.getObjectOrientation(robot, -1) 

    next_point = sim.getClosestPositionOnPath(path_handle, curPos)
    sim.setObjectPosition(goal, -1,sim.getPositionOnPath(path_handle, next_point))
    
    targetPos = sim.getObjectPosition(goal, robot)
    targetOri = sim.getObjectOrientation(goal, -1)
    dist = math.sqrt(targetPos[1]*targetPos[1]+targetPos[2]*targetPos[2])
    dist_ang = targetOri[3]-curOri[3]
    dt=sim.getSimulationTime()-oldtime
--    print("dt = ".. dt)
    if(dist>0.1) then
        if (dt>=0.05) then
            -- calculate the velocity 
            error= dist
            integral=integral+error*dt
            derivative=(error- prev_error)/dt    
            vcorr = Kp*error +Ki*integral +Kd*derivative
            if vcorr> nominalVelocity then vcorr=nominalVelocity end
            if vcorr< -nominalVelocity then vcorr=-nominalVelocity end
            prev_error=error
            oldtime=sim.getSimulationTime()
        else
            prev_error=0
            integral=0
            omega=0
            dt=0
            startTimer=sim.getSimulationTime()
            print(startTimer)
        end
        -- update velocity
    vr = nominalVelocity + vcorr
    vl = nominalVelocity - vcorr
    omega=R*(vr-vl)/L
    vel=(vl+vr)/2
    else 
        print("Close to the target")
        vl = nominalVelocity
        vr = nominalVelocity
    end
--    print("vr = "..vr)
--    print("vl = " ..vl)
end

velold=vel
omegaold=omega

    data=sim.packFloatTable({vel,omega*rad2deg})

    if data~=dataold then
        sim.setStringSignal('autodrive',data)
--        sim.addStatusbarMessage(sim.getSimulationTime()..' vel '..vel..' omega '..omega*rad2deg)
        dataold=data

    end

if (sim.getSimulationState()==sim.simulation_advancing_lastbeforestop) then
    print('\n\nEND autodrive')
end 
end 
------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and later: 
end 
------------------------------------------------------------------------------ 


function sysCall_threadmain()
-- initialize objects
motorR = sim.getObjectHandle('motorRight')
motorL = sim.getObjectHandle('motorLeft')
robot = sim.getObjectHandle('wheelchair_shape')
path_handle = sim.getObjectHandle('Path')
path_plan_handle=simGetPathPlanningHandle('PathPlanningTask0')
start_dummy_handle = sim.getObjectHandle('wheelchairFrame')
goal = sim.getObjectHandle('p3_goalPos')
wheelHandle=sim.getObjectHandle('bigWheelLeft_respondable')
res,zMin=sim.getObjectFloatParameter(wheelHandle,17)
res,zMax=sim.getObjectFloatParameter(wheelHandle,20)
R=(zMax-zMin)/2-- m (wheel radius)
next_point =0
dist = 0 -- dist from the robot to the point on the path 
--some constants 

MPI=math.pi
rad2deg=180/MPI
deg2rad=MPI/180
    
--model related measures and constants definitions
L = 0.6403 -- m (distance between wheels)
maxLinVel=0.5 --linear velocity in m/s
maxAngVel=220 --deg/sec (3.876¨rad/sec)
maxomega=20*deg2rad 
nominalVelocity=2.5--0.996 --EVENTUALLY GETTING FROM THE SLIDER

function setAngle(ang)     --set ang in ]-PI, PI]
                        -- input is an angle in radian and return the angle in rad between ]-pi,pi]
    while ang>math.pi do 
        ang=ang-(2*math.pi)--print("while!!")
    end
    while ang<-math.pi do 
        ang=ang+(2*math.pi)--print("while!!")
    end
    return ang
end

function normAngle(ang) --get an angle in rad and returns it in [-pi, pi]
    while ang>math.pi do ang=ang-2*math.pi  end
    while ang<=-math.pi do ang=ang+2*math.pi  end
    return ang
end

function getAngularDifference(goalAngle,startAngle)
    dx=goalAngle-startAngle
    if (dx>=0) then
        dx=math.mod(dx+math.pi,2*math.pi)-math.pi
    else
        dx=math.mod(dx-math.pi,2*math.pi)+math.pi
    end
    return(dx)
end




data=sim.getStringSignal('autodrive')
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



------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and later: 
if (sim_call_type==sim.syscb_init) then 
    sim.setScriptAttribute(sim.handle_self,sim.childscriptattribute_automaticcascadingcalls,false) 
end 
if (sim_call_type==sim.syscb_cleanup) then 
 
end 
if (sim_call_type==sim.syscb_sensing) then 
    sim.handleChildScripts(sim_call_type) 
end 
if (sim_call_type==sim.syscb_actuation) then 
    if not firstTimeHere93846738 then 
        firstTimeHere93846738=0 
    end 
    sim.setScriptAttribute(sim.handle_self,sim.scriptattribute_executioncount,firstTimeHere93846738) 
    firstTimeHere93846738=firstTimeHere93846738+1 
    
    ------------------------------------------------------------------------------ 
-- initialize objects
motorR = sim.getObjectHandle('motorRight')
motorL = sim.getObjectHandle('motorLeft')
robot = sim.getObjectHandle('wheelchair_shape')
path_handle = sim.getObjectHandle('Path')
path_plan_handle=simGetPathPlanningHandle('PathPlanningTask0')
start_dummy_handle = sim.getObjectHandle('wheelchairFrame')
goal = sim.getObjectHandle('p3_goalPos')
wheelHandle=sim.getObjectHandle('bigWheelLeft_respondable')
res,zMin=sim.getObjectFloatParameter(wheelHandle,17)
res,zMax=sim.getObjectFloatParameter(wheelHandle,20)
R=(zMax-zMin)/2-- m (wheel radius)
next_point =0
dist = 0 -- dist from the robot to the point on the path 
--some constants 

MPI=math.pi
rad2deg=180/MPI
deg2rad=MPI/180
    
--model related measures and constants definitions
L = 0.6403 -- m (distance between wheels)
maxLinVel=0.5 --linear velocity in m/s
maxAngVel=220 --deg/sec (3.876¨rad/sec)
maxomega=20*deg2rad 
nominalVelocity=2.5--0.996 --EVENTUALLY GETTING FROM THE SLIDER

-- Path panning
planstate=simSearchPath(path_plan_handle,10)
if (planstate == 0) then
    print('Could not find the path!!'..planstate)
end
-- Follow the path. 
l=sim.getPathLength(path_handle)
path_pos = sim.getPositionOnPath(path_handle, next_point) -- pos_on_path [0-1]
path_ori = sim.getOrientationOnPath(path_handle, next_point)
sim.setObjectPosition(start_dummy_handle, -1, path_pos) -- put the robot in the start of the path
sim.setObjectOrientation(start_dummy_handle, -1, path_ori) -- ajust the robot orientation on the start
next_point = next_point + 0.1
sim.setObjectPosition(goal, -1,sim.getPositionOnPath(path_handle, next_point))
sim.setObjectOrientation(goal, -1, sim.getOrientationOnPath(path_handle, next_point))


while(next_point<1) do
    
    print("next_point = "..next_point)
-- current position/distance
    curPos = sim.getObjectPosition(robot, -1)
    curOri = sim.getObjectOrientation(robot, -1) 

    next_point = sim.getClosestPositionOnPath(path_handle, curPos)
    sim.setObjectPosition(goal, -1,sim.getPositionOnPath(path_handle, next_point))
    
    targetPos = sim.getObjectPosition(goal, robot)
    targetOri = sim.getObjectOrientation(goal, -1)
    dist = math.sqrt(targetPos[1]*targetPos[1]+targetPos[2]*targetPos[2])
    dist_ang = targetOri[3]-curOri[3]
--    print("dt = ".. dt)

    -- calculate the velocity by autodrive
    data=sim.getStringSignal('autodrive')
    -- update velocity
    if data ~=nil then
        param=sim.unpackFloatTable(data)
        vel=param[1]
        omega=param[2]
    
        -- update velocities    
        vl = vel - (L/2*omega*MPI/180.)/(R);
        vr = vel + (L/2*omega*MPI/180.)/(R);
        RICR=L/2 *(vr+vl)/(vr-vl)

        sim.setJointTargetVelocity(motorR,vl)
        sim.setJointTargetVelocity(motorL,vr)
--    print("vr = "..vr)
--    print("vl = " ..vl)
    end 
end
sim.setJointTargetVelocity(motorR,0)
sim.setJointTargetVelocity(motorL,0)
print("Path is over!!!")
end

------------------------------------------------------------------------------ 

------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and later: 



    -- show target moving



function setAngle(ang)     --set ang in ]-PI, PI]
                            -- input is an angle in radian and return the angle in rad between ]-pi,pi]
        while ang>math.pi do ang=ang-(2*math.pi)print("while!!")end
        while ang<-math.pi do ang=ang+(2*math.pi)print("while!!")end
   
        return ang
    end
   
function sysCall_init()
        rad2deg=180/math.pi
        deg2rad=math.pi/180
        MPI=math.pi
   
        goal=sim.getObjectHandle('p3_goalPos')
        lmotorHandle=sim.getObjectHandle('motorLeft')
        rmotorHandle=sim.getObjectHandle('motorRight')
        roomba=sim.getObjectHandle('wheelchairFrame')--body')
        vel=0
        omega=0
        tolHeading=5*deg2rad -- tolerance heading in deg
        tolPosition=0.02 -- reaching position tolerance in meter
        gainHeading=200--40
        gainSpeed=1--10
        maxvel= 0.5
   
   
        kro=0.4-- 2/11
        kalpha=345/823
        kbeta=-0.6-- -2/11
        --alpha=0
   
        newTarget=false
        tgpos=sim.getObjectPosition(roomba,-1)
        tgori=sim.getObjectOrientation(roomba,-1)
    tgpos[3]=tgpos[3]+100 --shift in z to make the goal way up to the current robot pos
                           --to change if policy change i.e. goal disappear when not in auto mode
        sim.setObjectPosition(goal,-1,tgpos)
        sim.setObjectOrientation(goal,-1,tgori)
        tgpos_old=tgpos
        tgori_old=tgori
    end
   

function sysCall_actuation()
   
    tgpos=sim.getObjectPosition(goal,-1)
    tgori=sim.getObjectOrientation(goal,-1)
    if tgpos_old[1]~=tgpos[1] or tgpos_old[2]~=tgpos[2] or  tgori_old[1]~=tgori[1] or tgori_old[2]~=tgori[2] then
        --here a new goal has been defined
        newTarget=true
      end
    tgpos_old=tgpos
    tgori_old=tgori
   
    xg=tgpos[1]
    yg=tgpos[2]
    thetag=tgori[3]
   
        curpos=sim.getObjectPosition(roomba,-1)
        curori=sim.getObjectOrientation(roomba,-1)
        xr=curpos[1]      --current x robot
        yr=curpos[2]      --current y robot
        thetar=curori[3] --current theta robot
   
        ro=math.sqrt((xg-xr)^2+(yg-yr)^2)
        alpha=setAngle(-thetar + math.atan2(yg-yr,xg-xr))
        beta=setAngle(-math.atan2(yg-yr,xg-xr)+ thetag)
   
   
    print('ctrl loop:',math.abs(ro)>tolPosition , math.abs(thetag-thetar)>tolHeading,math.abs(ro)>tolPosition or math.abs(thetag-thetar)>tolHeading)
    print('ro:',math.abs(ro))
    if math.abs(ro)>tolPosition or setAngle(math.abs(setAngle(thetag)-setAngle(thetar)))>tolHeading then
        newTarget=false
   
        --print('\n fresh alpha',alpha)
    alpha=setAngle(alpha)
    print(math.abs(thetag-thetar))
        --print('\n')
        --print('ro',ro,'alpha',alpha*rad2deg,'beta',beta*rad2deg,'thetar',thetar*rad2deg,'thetag',thetag*rad2deg)
   
        --print('       alpha (deg)',alpha*rad2deg, 'beta', beta*rad2deg)
   
        if alpha <-MPI/2 or alpha >=MPI/2 then
            ro=-ro
            alpha=alpha+MPI
            beta=beta+MPI
   
            alpha=setAngle(alpha)
            print('in I2: alpha (deg)',alpha*rad2deg, 'beta', beta*rad2deg)
   
        end
   
        vel=kro*ro
        omega=kalpha*alpha + kbeta*beta
   
        --print('vel',vel,'omega',omega,'omega(deg/s)',omega*rad2deg)
   
    else
   
        if newTarget==false then --robot is paused
            print('ROBOT IS STOPPED')
            omega=0
            vel=0
            --ro=1000
            --newTarget=true
        end
   
    end
    -- send vel and omega to the motors here
    data=sim.packFloatTable({vel,omega*rad2deg})
    sim.setStringSignal('autodrive',data)

    end


function sysCall_cleanup()
     print('\n\nEND autodrive')
end

 
6.1755e-1
8.7352e-1
1.0234e+0




if (pathCalculated==1) then
    print("path")
    r=simPerformPathSearchStep(tempPathSearchObject,false)
    if (r<1) then
        if (r~=-2) then
            pathCalculated=0 -- path search failed, try again from the beginning
            tempPathSearchObject=-1
         end
     else
         print("We've find a path")
         c=2 -- we found a path
         currentPosOnPath=0
         tempPathSearchObject=-1
         sim.setObjectPosition(robot,-1, sim.getPositionOnPath(pathHandle,currentPosOnPath))
         sim.setObjectOrientation(robot, -1, sim.getOrientationOnPath(pathHandle,currentPosOnPath))
     end
end
