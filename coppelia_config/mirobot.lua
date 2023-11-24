function sysCall_init()
    corout=coroutine.create(coroutineMain)
end

function sysCall_actuation()
    if coroutine.status(corout)~='dead' then
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    end
end

function coroutineMain()
    -- Get handles:
    modelBase=sim.getObject('.')
    gripperHandle=sim.getObject('./vacuumGripper')
    
    local maxTorque=50
    
    -- Movement params for FK:
    -- Robot dynamics
    maxJointVel={45*math.pi/180,45*math.pi/180,45*math.pi/180,45*math.pi/180,45*math.pi/180,45*math.pi/180}
    maxJointAccel={40*math.pi/180,40*math.pi/180,40*math.pi/180,40*math.pi/180,40*math.pi/180,40*math.pi/180}
    maxJointJerk={80*math.pi/180,80*math.pi/180,80*math.pi/180,80*math.pi/180,80*math.pi/180,80*math.pi/180}

    -- Movement params for IK:
    maxVel={0.1}
    maxAccel={0.1}
    maxJerk={0.5}
    metric={1,1,1,0.1}
    
    -- Get more handles:
    jointHandles={}
    for i=1,6,1 do
        jointHandles[i]=sim.getObject('./joint'..i)
        sim.setJointTargetForce(jointHandles[i],maxTorque)
        sim.setObjectFloatParam(jointHandles[i],sim.jointfloatparam_maxvel,maxJointVel[i])
    end
end

function setGripperOn(inInts,inFloats,inStrings,inBuffer)
    if inInts[1] == 1 then
        sim.writeCustomDataBlock(gripperHandle,'gripperOn','on')
    else
        sim.writeCustomDataBlock(gripperHandle,'gripperOn','')
    end
end