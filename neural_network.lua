-- require('matrix')

-- ---@class NeuralNetwork
-- ---@field entrance integer
-- ---@field exit integer
-- ---@field layers table<integer,Matrix>
-- ---@field activation_function fun(val: number): number
-- ---@field calculate fun(self: NeuralNetwork, vales: Matrix): Matrix

-- ---@class Dimention
-- ---@field x integer
-- ---@field y integer

-- ---@param x integer
-- ---@param y integer
-- ---@return Dimention
-- function Dimention(x, y)
--     return {
-- 		x = x, y = y
-- 	}	
-- end

-- ---@param self NeuralNetwork
-- ---@param values Matrix
-- local function calc(self, values)
--     local values = values:transpose():transpose()
--     for i = 1, #self.layers do
--         values = self.layers[i]:times(values)
--     end
--     return values
-- end

-- ---@param layers table<integer,Dimention>
-- ---@return NeuralNetwork
-- function NeuralNetwork(layers)
-- 	local matrices = {}
--     for i = 1, #layers do
--         matrices[i] = Matrix(layers[i].x, layers[i].y)
--     end
-- 	---@type NeuralNetwork
--     return {
--         calculate = calc,
--         matrices = matrices,
-- 		entrance = layers[1]
-- 	}
-- end