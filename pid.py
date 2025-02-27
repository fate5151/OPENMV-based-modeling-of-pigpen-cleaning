from pyb import millis
from math import pi, isnan

class PID:
    def __init__(self, p=0, i=0, d=0, imax=0, d_filter_hz=20):
        self._kp = float(p)
        self._ki = float(i)
        self._kd = float(d)
        self._imax = abs(imax)
        self._d_filter_hz = d_filter_hz
        self._RC = 1/(2 * pi * self._d_filter_hz) if self._d_filter_hz > 0 else 0
        self.reset()

    def reset(self):
        self._integrator = 0
        self._last_error = 0
        self._last_derivative = float('nan')
        self._last_t = 0

    def get_pid(self, error, scaler=1.0):
        tnow = millis()
        dt = tnow - self._last_t

        # 处理异常时间情况
        if self._last_t == 0 or dt > 1000 or dt < 0:
            dt = 0
            self.reset()
        else:
            dt = min(dt, 100)  # 限制最大有效dt为100ms

        delta_time = dt / 1000.0
        output = 0.0

        # 比例项
        output += error * self._kp

        # 微分项
        if self._kd != 0 and dt > 0:
            try:
                raw_derivative = (error - self._last_error) / delta_time
            except ZeroDivisionError:
                raw_derivative = 0

            if isnan(self._last_derivative):
                derivative = raw_derivative
            else:
                alpha = dt / (self._RC * 1000 + dt)
                derivative = self._last_derivative + alpha * (raw_derivative - self._last_derivative)

            output += self._kd * derivative
            self._last_derivative = derivative

        # 积分项（带抗饱和）
        if self._ki != 0 and dt > 0:
            if (error > 0 and self._integrator < self._imax) or \
               (error < 0 and self._integrator > -self._imax):
                self._integrator += error * self._ki * delta_time
                self._integrator = max(-self._imax, min(self._imax, self._integrator))
            output += self._integrator

        # 更新状态
        self._last_error = error
        self._last_t = tnow

        return output * scaler
