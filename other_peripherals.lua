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

