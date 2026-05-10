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
-- sleep(2)

--- height, gimbal[1], gimbal[2]


PIDs = {
    PID.new(5, 1, 2),
    PID.new(.1, .001, .1),
    PID.new(.1 ,.001 ,.1),
    PID.circular(.25,0,16, -math.pi, math.pi)
}

Desired = {
    200,
    0,
    0,
    0
}

local sys = MatricialControlSystem(#PIDs, #ENGINES, PIDs)

sys.relationship_matrix.data[1][1] = .01
sys.relationship_matrix.data[2][1] = .01
sys.relationship_matrix.data[3][1] = .01
sys.relationship_matrix.data[4][1] = .01

sys.relationship_matrix.data[1][2] = 35
sys.relationship_matrix.data[2][2] = -35
sys.relationship_matrix.data[3][2] = 35
sys.relationship_matrix.data[4][2] = -35

sys.relationship_matrix.data[1][3] = -35
sys.relationship_matrix.data[2][3] = 35
sys.relationship_matrix.data[3][3] = 35
sys.relationship_matrix.data[4][3] = -35


sys.relationship_matrix.data[6][4] = -80
sys.relationship_matrix.data[5][4] = -80

local dt = .1

while true do
    local erros = Matrix(4, 1)
    erros.data[1][1] = Desired[1] - alt.getHeight()
    erros.data[2][1] = Desired[2] - gim.getAnglesRad()[1]
    erros.data[3][1] = Desired[3] - gim.getAnglesRad()[2]
    erros.data[4][1] = Desired[4] - nav.getRelativeAngleRad()

    local outs = sys:compute(erros, dt)

    print("errors\n" .. erros:toString())
    -- print("weights\n" .. sys.relationship_matrix:toString())
    print("outs\n" .. outs:toString())

    for i = 1, outs.r do
        ENGINES[i].set(outs.data[i][1])
        --print("E" .. i .. ": " .. outs.data[i][1])
    end

    sleep(dt)
    term.clear()
    term.setCursorPos(1,1)
end