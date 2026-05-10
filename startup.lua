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
	"Create_CreativeMotor_5"
}

NW = CreativeMotor(LOC[1])
NE = CreativeMotor(LOC[2])
SW = CreativeMotor(LOC[3])
SE = CreativeMotor(LOC[4])

ENGINES = { NW, NE, SW, SE }

---@type SelfAircraft
AIRCRAFT = {
    update = function ()
        
    end,
    coord = Vec3(0,0,0),
    speed = Vec3(0,0,0)
}

print("ola mundo")

for i = 1, 4 do
    ENGINES[i].set(0)
end


local des_height = 300
local alt = AltitudeSensor()
local gim = GimbalSensor()
local nav = NavigationTable()

if not alt then error'not alt' end
sleep(2)

--- height, gimbal[1], gimbal[2]


PIDs = {
    PID.new(0, 0, 0),
    PID.new(0, 0, 0),
    PID.new(0 ,0 ,0)
}

Desired = {
    300,
    0,
    0
}

local sys = MatricialControlSystem(3,4,PIDs)
local dt = .1

while true do
    local erros = Matrix(3, 1)
    erros.data[1][1] = Desired[1] - alt.getHeight()
    erros.data[2][1] = Desired[2] - gim.getAnglesRad()[1]
    erros.data[3][1] = Desired[3] - gim.getAnglesRad()[2]

    local outs = sys:compute(erros, dt)
    for i = 1, outs.r do
        ENGINES[i].set(outs.data[i][1])
        print("E" .. i .. ": " .. outs.data[i][1])
    end

    sleep(dt)
end