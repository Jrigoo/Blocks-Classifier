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
    conveyor = sim.getObject('./conveyor')
end

function setConveyor(inInts,inFloats,inStrings,inBuffer)
    sim.writeCustomTableData(conveyor,'__ctrl__',{vel=inFloats[1]}) -- vel. ctrl
end

