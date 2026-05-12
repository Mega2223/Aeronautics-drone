require('matrix')
require('vector')

function AngleDifference(angle_a, angle_b)
    local interval = 2 * math.pi
    local err = angle_b - angle_a
    while err < -math.pi do
        err = err + interval
    end
    while err > math.pi do
        err = err - interval
    end
    if err > (interval / 2) then
        err = err - interval
    end
    if err < -(interval / 2) then
        err = err + interval
    end
    return err
end

function Clamp(num, min_n, max_n)
	return math.max(min_n,math.min(max_n,num))
end

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
---@field rotation Vec3

---@param self Autopilot
---@param goals table<integer, number>
---@param y number
---@param rx number
---@param heading number
---@param rz number
---@param dt number
local function computeGoals(self, goals, y, rx, rz, heading, dt)
	local errors = Matrix(4, 1)
	errors.data[1][1] = goals[1] - y
	errors.data[2][1] = goals[2] - rx -- gim.getAnglesRad()[1]
	errors.data[3][1] = goals[3] - rz -- gim.getAnglesRad()[2]
    errors.data[4][1] = goals[4] - heading --nav.getRelativeAngleRad()
		
	local outs = self.control_schema:compute(errors, dt)

	print("errors\n" .. errors:toString())
	-- print("weights\n" .. sys.relationship_matrix:toString())
	print("outs\n" .. outs:toString())

	for i = 1, outs.r do
		self.ENGINES[i].set(outs.data[i][1])
	end

end

Autopilot = {}

---@type PID
local cruise_pid = PID.new(-.003,0,-.003)

---@param self Autopilot
---@param x number
---@param y number
---@param z number
---@param dt number
local function update(self, x, y, z, rx, rz, heading, dt)
    self.position.x = x
    self.position.y = y
    self.position.z = z
	self.rotation.x = rx
    self.rotation.y = heading
    self.rotation.z = rz

    if self.state == AutopilotState.TAKEOFF then
		print("Takeoff phase\n")
		local goals = {
			self.local_goal.y, -- height
			0, 0, 0
        }
		
		computeGoals(self, goals, y, rx, rz, heading, dt)

        if math.abs(self.position.y - self.local_goal.y) <= 3 then
			local d_y = self.local_goal.y
            self.state = AutopilotState.CRUISE
			self.local_goal = self.destination:clone()
			self.local_goal.y = d_y
		end
    elseif self.state == AutopilotState.CRUISE then
		local angle = math.atan(self.position.x - self.destination.x, self.position.z - self.destination.z)
        print("Cruise Phase\nHeading:",self.rotation.y * 180 / math.pi," Wanted heading:", angle * 180 / math.pi)
		print("Current: ", self.position:toString())
		print("Goal: ", self.local_goal:toString())
		local goals = {
			self.local_goal.y, 0, 0, angle
        }
        local dif = AngleDifference(angle, self.rotation.y) * 180 / math.pi
        dif = math.abs(dif)
		local dist = math.min(
			math.sqrt((self.position.x - self.local_goal.x)^2 + (self.position.z - self.local_goal.z)),
			100
		)
        if dif <= 6 then
            goals[3] = math.min(.1, cruise_pid:update(dist, dt))
            print("Angle ", dif, " good enough to go forward with ", goals[3])
            print("distance = ", dist)
        end
        if dist < 3 then
            print("landing phase")
			self.state = AutopilotState.LANDING
			self.local_goal = self.destination
		end

        computeGoals(self, goals, y, rx, rz, heading, dt)
    elseif self.state == AutopilotState.LANDING then
        print("Landing...")
        local dif = AngleDifference(0, self.rotation.y) * 180 / math.pi
		local dist = math.sqrt((self.position.x - self.local_goal.x)^2 + (self.position.z - self.local_goal.z))
        dif = math.abs(dif)
        if dif >= 3 then
            print('Aligning north...')
            local goals = { self.position.y - 5, 0, 0, 0 }
            computeGoals(self, goals, y, rx, rz, heading, dt)
        elseif dist <= 5 then
            print('Descending...')
			local xerr, zerr = self.local_goal.x - self.position.x, self.local_goal.z - self.position.z
			local xang, zang = - Clamp(zerr * 0.005,-.025,.025), Clamp(xerr * 0.005,-.025,.025)
			local goals = {
				self.position.y - 30, zang, xang, 0
			}
            computeGoals(self, goals, y, rx, rz, heading, dt)
            if math.abs(self.position.y - self.local_goal.y) <= 5.5 then
                for i = 1, #self.ENGINES do
                    self.ENGINES[i].set(0)
                end
				self.state =AutopilotState.STOP
			end
        else
            local xerr, zerr = self.local_goal.x - self.position.x, self.local_goal.z - self.position.z
			local xang, zang = - Clamp(zerr * 0.005,-.025,.025), Clamp(xerr * 0.005,-.025,.025)
			print('Getting Closer...',xang,zang)
			local goals = {
				self.position.y - 10, zang, xang, 0
			}
			computeGoals(self, goals, y, rx, rz, heading, dt)
		end
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
		PID.circular(1,0,1, -math.pi, math.pi)
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
        position = Vec3(0, 0, 0),
		rotation = Vec3(0,0,0)
	}
end

return Autopilot