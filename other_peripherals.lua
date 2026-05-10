---@diagnostic disable: param-type-mismatch, return-type-mismatch

---@class Motor
---@field set fun(ac: number)

---@param loc string
---@return Motor
function CreativeMotor(loc)
    local p = peripheral.wrap(loc)
    ---@type Motor
    return {
        ---@diagnostic disable-next-line: undefined-field, need-check-nil
        set = p.setGeneratedSpeed
    }
end

-- "altitude_sensor"
---@class AltitudeSensor
---@field getHeight fun(): number
---@return AltitudeSensor
function AltitudeSensor()
	return peripheral.find("altitude_sensor")
end

-- "navigation_table"
---@class NavigationTable
---@field getRelativeAngle fun(): number
---@field getRelativeAngleRad fun(): number
---@return NavigationTable
function NavigationTable()
    return peripheral.find("navigation_table")
end

-- "gimbal_sensor"
---@class GimbalSensor
---@field getAngles fun(): table<integer,number>
---@field getAnglesRad fun(): table<integer,number>
---@return GimbalSensor
function GimbalSensor()
	return peripheral.find("gimbal_sensor")
end