---@class PID
---@field kp number
---@field ki number
---@field kd number
---@field err_sum number
---@field prev_err number
---@field update fun(self: PID, err: number, deltaT: number): number

require('matrix')

PID = {}

---@return PID
function PID.new(kp, ki, kd)
	---@type PID
    return {
		prev_err = 0, err_sum = 0,
        kp = kp, ki = ki, kd = kd,
		update = function (self,err, deltaT)
			local p = -err
			local i = -self.err_sum
			local d = deltaT * (self.prev_err - err)

			self.err_sum = self.err_sum + err
			self.prev_err = err
			return p * self.kp + i * self.ki + d * self.kd
		end
	}
end

function PID.circular(kp, ki, kd, min, max)
	---@type PID
    return {
		prev_err = 0, err_sum = 0,
        kp = kp, ki = ki, kd = kd,
        update = function(self, err, deltaT)
            local interval = max - min
			local pure_err = err
            while err < min do
                err = err + interval
            end
            while err > max do
				err = err - interval
			end

			local p = -err
			local i = -self.err_sum
			local d = deltaT * (self.prev_err - pure_err)

			self.err_sum = self.err_sum + err
			self.prev_err = pure_err
			return p * self.kp + i * self.ki + d * self.kd
		end
	}
end

---@class MatricialControlSystem
---@field relationship_matrix Matrix
---@field inputs integer
---@field input_PIDs table<integer, PID>
---@field outputs integer
---@field compute fun(self: MatricialControlSystem, errors: Matrix, deltaT:number): Matrix

---@param inputs integer
---@param outputs integer
---@param PIDs table<integer,PID>
---@return MatricialControlSystem
function MatricialControlSystem(inputs, outputs, PIDs)
    local mat = Matrix(outputs, inputs)
    local input_pids = {}
    for i = 1, PIDs do
        input_pids = PIDs[i]
	end

	---@type MatricialControlSystem
    return {
        inputs = inputs,
        outputs = outputs,
        input_PIDs = input_pids,
        relationship_matrix = mat,
        compute = function(self, errors, deltaT)
            local error_output = Matrix(errors.r, errors.c)
			for i = 1, self.inputs do
				error_output.data[i][1] = self.input_PIDs[i]:update(errors[i][1], deltaT)
			end
			---@diagnostic disable-next-line: return-type-mismatch
			return self.relationship_matrix:times(error_output)
		end
	}
end

return PID