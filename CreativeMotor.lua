
---@param loc string
---@return Motor
function FromLoc(loc)
    local p = peripheral.wrap(loc)
	---@type Motor
    return {
		---@diagnostic disable-next-line: undefined-field, need-check-nil
		set = p.setGeneratedSpeed
	}
end