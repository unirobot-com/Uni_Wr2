#!/usr/bin/env python

import matplotlib.pyplot as plt
 
 
class Pid():
    def __init__(self, exp_val, kp, ki, kd):
        self.KP = kp
        self.KI = ki
        self.KD = kd
        self.exp_val = exp_val
        self.now_val = 0
        self.now_err = 0
        self.last_err = 0
        self.last_last_err = 0
        self.change_val = 0
 
    def cmd_pid(self):
        self.last_last_err = self.last_err
        self.last_err = self.now_err
        self.now_err = self.exp_val - self.now_val
        self.change_val = self.KP * (self.now_err - self.last_err) + self.KI * \
            self.now_err + self.KD * (self.now_err - 2 * self.last_err
                                      + self.last_last_err)
        self.now_val += self.change_val
        return self.now_val
 
 
pid_val = []
my_Pid = Pid(1000000, 0.1, 0.15, 0.1)
for i in range(0, 30):
    pid_val.append(my_Pid.cmd_pid())
plt.plot(pid_val)
plt.show()
