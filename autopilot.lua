require('matrix')
require('vector')

---@enum AutopilotState
AutopilotState = {
    STOP = 1,
    TAKEOFF = 2,
	HEADING = 3,
    CRUISE = 4,
	LANDING = 5
}

---@class Autopilot
---@field state AutopilotState
---@field update fun(self: Autopilot, x: number, y: number, z:number, rx: number, ry: number, rz:number, dt:number)
---@field setDestination fun(self: Autopilot, destination: Vec3)
---@field startFlight fun(self: Autopilot)
---@field control_schema MatricialControlSystem
---@field PIDs table<integer,PID>
---@field ENGINES table<integer,Motor>
---@field destination ?Vec3
---@field local_goal ?Vec3
---@field position Vec3

Autopilot = {}

---@param self Autopilot
---@param x number
---@param y number
---@param z number
---@param dt number
local function update(self, x, y, z, rx, ry, rz, dt)
    self.position.x = x
    self.position.y = y
	self.position.z = z

    if self.state == AutopilotState.TAKEOFF then
		Desired = {
			self.local_goal.y, -- height
			0, 0, 0
        }
		local errors = Matrix(4, 1)
		errors.data[1][1] = Desired[1] - y
		errors.data[2][1] = Desired[2] - rx -- gim.getAnglesRad()[1]
		errors.data[3][1] = Desired[3] - rz -- gim.getAnglesRad()[2]
        errors.data[4][1] = Desired[4] - ry --nav.getRelativeAngleRad()
		
		local outs = self.control_schema:compute(errors, dt)

		print("errors\n" .. errors:toString())
		-- print("weights\n" .. sys.relationship_matrix:toString())
		print("outs\n" .. outs:toString())

		for i = 1, outs.r do
			self.ENGINES[i].set(outs.data[i][1])
		end
	elseif self.state == AutopilotState.CRUISE then
		local desired_heading = 0
	end
end

---@param PIDs table<integer,PID>
---@param ENGINES table<integer,Motor>
local function genControlSchema(PIDs, ENGINES)
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
	
	return sys
end

---@param ENGINES table<integer,Motor>
---@return Autopilot
function Autopilot.new(ENGINES)

	PIDs = {
		PID.new(5, 1, 6),
		PID.new(.5, .001, 16),
		PID.new(.5, .001, 16),
		PID.circular(.5,0,1, -math.pi, math.pi)
	}

    local control_schema = genControlSchema(PIDs, ENGINES)
	
	---@type Autopilot
    return {
        state = AutopilotState.STOP,
        update = update,
		setDestination = function(self, destination)
			self.destination = destination
        end,
        startFlight = function(self)
			print("taking off")
            self.local_goal = self.position:clone()
			self.local_goal.y = 200
			self.state = AutopilotState.TAKEOFF
        end,
        control_schema = control_schema,
        PIDs = PIDs,
		ENGINES = ENGINES,
		position = Vec3(0,0,0)
	}
end

return Autopilot