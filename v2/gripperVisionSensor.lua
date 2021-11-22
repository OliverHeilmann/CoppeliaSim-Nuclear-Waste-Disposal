--[[
function sysCall_init()
end

function sysCall_vision(inData)
    -- callback function automatically added for backward compatibility
    -- (vision sensor have no filters anymore, but rather a callback function where image processing can be performed)
    local retVal={}
    retVal.trigger=false
    retVal.packedPackets={}
    simVision.sensorImgToWorkImg(inData.handle)
    -- simVision.intensityScaleOnWorkImg(inData.handle,1.000000,0.000000,true)
    simVision.selectiveColorOnWorkImg(inData.handle,{0.500000,0.900000,0.500000},{0.500000,0.100000,0.400000},false,true,false)
    local trig,packedPacket=simVision.blobDetectionOnWorkImg(inData.handle,0.150000,0.000000,false) if trig then retVal.trigger=true end if packedPacket then retVal.packedPackets[#retVal.packedPackets+1]=packedPacket end
    simVision.sensorImgToWorkImg(inData.handle)
    simVision.workImgToSensorImg(inData.handle)

    print(retVal)
    return retVal
end
--]]


function sysCall_vision(inData)
    simVision.sensorImgToWorkImg(inData.handle) -- copy the vision sensor image to the work image

    simVision.selectiveColorOnWorkImg(inData.handle,{1.0,0.0,0.0},{1,0.2,0.0},true,true,true) -- red filter
    --simVision.selectiveColorOnWorkImg(inData.handle,{0.81,1.0,0.0},{0.81,0.0,0.0},true,true,true) -- nuclear filter

    -- unpack the blob data (see below for more details on the contents)
    trigger, packedPacket=simVision.blobDetectionOnWorkImg(inData.handle, 0.1, 0.0, true, {1.0,1.0,1.0})

    -- See below on examples of packedPacket output format
    -- Unpack as {1.0, 6.0, 0.043296813964844, -0.0, 0.5, 0.5068359375, 0.232421875, 0.234375}
    -- Unpack as {2.0, 6.0, 0.015182495117188, -0.78539818525314, 0.5, 0.509765625, 0.13810682296753, 0.13810682296753, 0.02362060546875, -0.0, 0.5, 0.80859375, 0.138671875, 0.212890625}
    --[[
        a) the number of detected blobs
        b) the number of values returned for each blob
        then for each blob:
        c.1) the blob relative size
        c.2) the blob orientation
        c.3) the blob relative position X
        c.4) the blob relative position Y
        c.5) the blob bounding box relative width
        c.6) the blob bounding box relative height
    ]]

    -- before being able to access the information, we must unpack it
    data=sim.unpackFloatTable(packedPacket,0,0,0)

    -- if a detection has been made then find its corresponding dz value and then print
    if data[5] ~= nil and data[6] ~= nil then
        depthBuffer=sim.getVisionSensorDepthBuffer(inData.handle,math.floor(data[5]*512),math.floor(data[6]*512),1,1)

        local dx_pix = tonumber(string.format("%.3f", math.floor(data[5]*512)))
        local dy_pix  = tonumber(string.format("%.3f", math.floor(data[6]*512)))
        local dz_depth = tonumber(string.format("%.3f", depthBuffer[1]))

        print(dx_pix, dy_pix, dz_depth)
    end

    simVision.workImgToSensorImg(inData.handle) -- copy the work image to the vision sensor image buffer
end

function sysCall_init()
end



--[[
function sysCall_init() 
    maxScanDistance=5
    cameraAngle=60*math.pi/180
    pointsToExtract={32,32}
    
    visionSensorHandle=sim.getObjectHandle(sim.handle_self)
    pointCloudHandle=sim.getObjectHandle("PointCloud#")
    sim.removePointsFromPointCloud(pointCloudHandle,0,nil,0)

    sim.setObjectFloatParam(visionSensorHandle,sim.visionfloatparam_far_clipping,maxScanDistance)
    sim.setObjectFloatParam(visionSensorHandle,sim.visionfloatparam_perspective_angle,cameraAngle)

    showPoints=true
        
    local col={1,1,0}
    points=sim.addDrawingObject(sim.drawing_spherepoints,0.005,0,-1,100000,nil,nil,nil,col)
end

function sysCall_vision(inData)
    simVision.sensorImgToWorkImg(inData.handle)
    simVision.workImgToBuffer1(inData.handle)
    simVision.sensorDepthMapToWorkImg(inData.handle)
    local trig,packedPacket,packedCols=simVision.coordinatesFromWorkImg(inData.handle,pointsToExtract,false,true)
    if packedPacket then
        local measuredData={}
        local colors={}
        sim.addDrawingObjectItem(points,nil)
        local u1=sim.unpackFloatTable(packedPacket)
        local cols
        if packedCols then
            cols=sim.unpackUInt8Table(packedCols)
        end
        local m=sim.getObjectMatrix(visionSensorHandle,-1)
        if u1 then
            for j=0,u1[2]-1,1 do
                for i=0,u1[1]-1,1 do
                    local ind=(j*u1[1]+i)
                    local w=2+4*ind
                    v1=u1[w+1]
                    v2=u1[w+2]
                    v3=u1[w+3]
                    v4=u1[w+4]
                    if (v4<maxScanDistance*0.999) then
                        local p={v1,v2,v3}
                        p=sim.multiplyVector(m,p)
                        table.insert(measuredData,p[1])
                        table.insert(measuredData,p[2])
                        table.insert(measuredData,p[3])
                        if cols then
                            table.insert(colors,cols[3*ind+1])
                            table.insert(colors,cols[3*ind+2])
                            table.insert(colors,cols[3*ind+3])
                        end
                        if showPoints then
                            sim.addDrawingObjectItem(points,p)
                        end
                    end
                end
            end
        end
        if #colors>0 then
            sim.insertPointsIntoPointCloud(pointCloudHandle,2,measuredData,colors,0.001) -- wasn't yet supported in CoppeliaSim V4.2.0Rev2
        else
            sim.insertPointsIntoPointCloud(pointCloudHandle,0,measuredData,{255,0,255},0.001)
        end
    end
end
--]]
