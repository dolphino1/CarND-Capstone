
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import rospy


GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self,
                 wheel_base,
                 steer_ratio,
                 min_speed,
                 max_lat_accel,
                 max_steer_angle):
        self._throttle_break_pid = PID(2, 0, 0.5, -1, 1)
        self._steering_pid = PID(0.5, 0, 1.0, -8.2, 8.2)
        self._yaw_controller = YawController(wheel_base,
                                             steer_ratio,
                                             min_speed,
                                             max_lat_accel,
                                             max_steer_angle)

        self._throttle_break_filter = LowPassFilter(0.2, 0.1)
        self._steering_filter = LowPassFilter(0.2, 0.1)

    def control(self, twist, current_velocity, dt):
        current_speed_ms = current_velocity.twist.linear.x
        target_speed_ms = twist.twist.linear.x
        target_angular_speed = twist.twist.angular.z

        rospy.logdebug("target_speed_ms: %f current_speed_ms: %f target_angular_speed: %f",
                       target_speed_ms, current_speed_ms, target_angular_speed)

        steer = self._yaw_controller.get_steering(target_speed_ms,
                                                  target_angular_speed,
                                                  current_speed_ms)
        steer = self._steering_filter.filt(steer)

        throttle_break = self._throttle_break_pid.step(target_speed_ms - current_speed_ms, dt)
        throttle_break = self._throttle_break_filter.filt(throttle_break)
        throttle = throttle_break * 1 if throttle_break > 0.1 else 0.
        break_value = (-3250.0 * throttle_break) if throttle_break < -0.1 else 0
        return throttle, break_value, steer

    def reset(self):
        self._steering_pid.reset()
        self._throttle_break_pid.reset()