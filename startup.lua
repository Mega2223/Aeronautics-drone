require('other_peripherals')
require('PID')
require('matrix')
require('vector')

---@class SelfAircraft
---@field update fun()
---@field coord Vec3
---@field speed Vec3

LOC = {
    "Create_CreativeMotor_6",
    "Create_CreativeMotor_7",
    "Create_CreativeMotor_4",
    "Create_CreativeMotor_5",
    "Create_CreativeMotor_8",
    "Create_CreativeMotor_9"
}

NW = CreativeMotor(LOC[1])
NE = CreativeMotor(LOC[2])
SW = CreativeMotor(LOC[3])
SE = CreativeMotor(LOC[4])

A1 = CreativeMotor(LOC[5])
A2 = CreativeMotor(LOC[6])

ENGINES = { NW, NE, SW, SE, A1, A2 }

---@type SelfAircraft
AIRCRAFT = {
    update = function ()
        
    end,
    coord = Vec3(0,0,0),
    speed = Vec3(0,0,0)
}

print("ola mundo")

for i = 1, 6 do
    ENGINES[i].set(0)
end


local des_height = 300
local alt = AltitudeSensor()
local gim = GimbalSensor()
local nav = NavigationTable()

if not alt then error'not alt' end

local dt = .1

require('autopilot')

local autopilot = Autopilot.new(ENGINES)

local x,y,z = gps.locate(2,true)
autopilot:update(x,y,z,0,0,0,1)
-- autopilot:setDestination(Vec3(-292, 104, 230))
autopilot:setDestination(Vec3(265, 65, -80))
-- autopilot:setDestination(Vec3(-1741, 7, 536))

autopilot:startFlight()

os.sleep(4)
-- autopilot.local_goal = autopilot.destination:clone()
-- autopilot.state = AutopilotState.LANDING

while true do
    local x,y,z = gps.locate(2,false)
    y = alt.getHeight()
    local g = gim.getAnglesRad()
    autopilot:update(x,y,z,g[1],g[2],nav.getRelativeAngleRad(),dt)

    sleep(dt)
    term.clear()
    term.setCursorPos(1, 1)
end