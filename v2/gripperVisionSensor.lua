-- Initialize step
function sysCall_init()
end

-- Main script which returns dx,dy (in pixel values) and depth (in meters)
-- NOTE: make sure to enable 'Explicit Handling' in 'Scene Object Properties' section
-- in CoppeliaSim
function sysCall_vision(inData)
    -- NOTES ON PARSING DATA TO AND FROM VISION SENSOR:
        --> inData.handle : the handle of the vision sensor.
        --> inData.resolution : the x/y resolution of the vision sensor
        --> inData.clippingPlanes : the near and far clipping planes of the vision sensor
        --> inData.viewAngle : the view angle of the vision sensor (if in persp. proj. mode)
        --> inData.orthoSize : the ortho size of the vision sensor (if in orth. proj. mode)
        --> inData.perspectiveOperation : true if the sensor is in persp. proj. mode

    -- convert the vision sensor image to the work image
    simVision.sensorImgToWorkImg(inData.handle) 

    -- filter for the appropriate colour
    simVision.selectiveColorOnWorkImg(inData.handle,{1.0,0.0,0.0},{1,0.2,0.0},true,true,true) -- red filter
    --simVision.selectiveColorOnWorkImg(inData.handle,{0.81,1.0,0.0},{0.81,0.0,0.0},true,true,true) -- nuclear filter

    -- unpack the blob data (see below for more details on the contents)
    trigger, packedPacket=simVision.blobDetectionOnWorkImg(inData.handle, 0.1, 0.0, true, {1.0,1.0,1.0})

    -- Next we must unpack the float table in order to access information from it. See below for the
    -- format which it is returned as:
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

        See below on examples of packedPacket output format:
        --> Unpack as {1.0, 6.0, 0.043296813964844, -0.0, 0.5, 0.5068359375, 0.232421875, 0.234375}
        --> Unpack as {2.0, 6.0, 0.015182495117188, -0.78539818525314, 0.5, 0.509765625, 0.13810682296753, 0.13810682296753, 0.02362060546875, -0.0, 0.5, 0.80859375, 0.138671875, 0.212890625}
    --]]

    -- before being able to access the information, we must unpack it
    local data = sim.unpackFloatTable(packedPacket,0,0,0)

    -- set local variables to overwrite if data is found
    local dx_pix = 0
    local dy_pix = 0
    local dz_depth = 0
    
    -- if a detection has been made then find its corresponding dz value and then print
    if data[5] ~= nil and data[6] ~= nil then
        -- the lines below first convert the unpacked floating point numbers (from 0 to 1) to the
        -- corresponding pixel X and Y values. These are then parsed to the corresponding positions
        -- in the getVisionSensorDepthBuffer() script to aquire its contituent depth values...
        relativeX = math.floor(data[5]*512)
        relativeY = math.floor(data[6]*512)
        depthBuffer=sim.getVisionSensorDepthBuffer(inData.handle,relativeX,relativeY,1,1)
        
        -- set the resultant values to 3 d.p.
        dx_pix = tonumber(string.format("%.3f", math.floor(data[5]*512)))
        dy_pix  = tonumber(string.format("%.3f", math.floor(data[6]*512)))
        dz_depth = tonumber(string.format("%.3f", depthBuffer[1]))
    end

    -- convert the work image to the vision sensor image buffer
    simVision.workImgToSensorImg(inData.handle)

    -- pack float table to return to function called in main niryo script
    local packedPacket_return = sim.packFloatTable({dx_pix, dy_pix, dz_depth}, 0,0)

    local outData={}
    --outData.trigger=false -- true if the sensor should trigger
    outData.packedPackets = {packedPacket_return} -- a table of packed packets. Can be accessed via e.g. sim.readVisionSensor
    return outData
end