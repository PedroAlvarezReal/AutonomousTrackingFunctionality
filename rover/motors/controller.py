from config import DRIVE_SPEED, TURN_SPEED


class MotorController:
    """
    Controls the rover's motors.
    Right now these are stubs — replace the print statements
    with your actual motor GPIO calls when the rover is built.
    """

    def drive_forward(self, speed=None):
        speed = speed or DRIVE_SPEED
        print(f"[MOTORS] Driving forward at speed {speed}")
        # TODO: GPIO PWM signal to left + right motors forward

    def turn_left(self, speed=None):
        speed = speed or TURN_SPEED
        print(f"[MOTORS] Turning LEFT at speed {speed}")
        # TODO: left motor backward, right motor forward

    def turn_right(self, speed=None):
        speed = speed or TURN_SPEED
        print(f"[MOTORS] Turning RIGHT at speed {speed}")
        # TODO: left motor forward, right motor backward

    def stop(self):
        print("[MOTORS] Stopped")
        # TODO: all motors off

    def close(self):
        self.stop()
