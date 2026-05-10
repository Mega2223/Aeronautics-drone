---@diagnostic disable: param-type-mismatch

require('other_peripherals')
require('PID')
require('matrix')
require('vector')

---@class SelfAircraft
---@field update fun()
---@field coord Vec3
---@field speed Vec3

LOC = {
    "Create_CreativeMotor_0",
    "Create_CreativeMotor_1",
    "Create_CreativeMotor_2",
	"Create_CreativeMotor_3"
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

MainPID = PID.new(1, .01, 2)

-- Talvez precise colocar um fator de soma

local INPUTS = 4
local OUTPUTS = 4
local weight = Matrix(OUTPUTS, INPUTS)

weight:set(1,1,0)
print(weight:toString())
-- MainPID.err_sum = 0

local des_height = 200
local alt = peripheral.find("altitude_sensor")
if not alt then error'not alt' end
sleep(1)

while true do
    local error = alt.getHeight() - des_height
    local upd = MainPID:update(error, .1)
    print(string.format("Error %.2f out %.2f", error, upd))
        
    for i = 1, 4 do
        ENGINES[i].set(upd)
    end
    sleep(.1)
end