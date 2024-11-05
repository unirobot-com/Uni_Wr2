class IncrementalPID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.prev_output = 0
        self.integral = 0

    def calculate_output(self, target, current):
        error = target - current
        self.integral += error
        derivative = error - self.prev_error
        output = self.prev_output + self.Kp * (error - self.prev_error) + self.Ki * error + self.Kd * derivative
        self.prev_error = error
        self.prev_output = output
        return output

# 设置目标值
target_value = 100

# 初始化增量式PID控制器
pid_controller = IncrementalPID(0.1, 0.01, 0.01)

# 初始数值
current_value = 0

# 模拟调节过程
for _ in range(100):
    output = pid_controller.calculate_output(target_value, current_value)
    current_value += output
    print("Current Value: {:.2f}".format(current_value))

