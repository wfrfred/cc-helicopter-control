---@meta

---@class vector
---@field x number
---@field y number
---@field z number
---@field length fun(self: vector): number
---@field dot fun(self: vector, other: vector): number
---@field cross fun(self: vector, other: vector): vector
---@operator add(vector): vector
---@operator sub(vector): vector
---@operator mul(number): vector

---@class quaternion
---@field a number
---@field v vector
---@field normalize fun(self: quaternion): quaternion
---@field conjugate fun(self: quaternion): quaternion
---@operator unm: quaternion
---@operator mul(quaternion): quaternion

---@class matrix
---@field from2DArray fun(rows: number[][]): matrix
---@field clone fun(self: matrix): matrix
---@operator mul(matrix): matrix
