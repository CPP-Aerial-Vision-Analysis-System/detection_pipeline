from pymavlink import mavutil
import time


def set_servo(mav, channel, pwm_value):
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,
        pwm_value,
        0,
        0,
        0,
        0,
        0,
    )
    # Confirmation, channel, pwm_value, unused parameters in DO_SET_SERVO
    print(f"Servo on channel {channel} set to PWM {pwm_value}")


def main():
    # Connect to the Pixhawk
    mav = mavutil.mavlink_connection(
        "/dev/ttyUSB0", baud=115200
    )  # Update with your connection string and baud rate

    # Wait for a heartbeat before sending commands
    mav.wait_heartbeat()
    print(
        f"Heartbeat received from system (system_id: {mav.target_system} component_id: {mav.target_component})"
    )

    servo_channel = 9  # 9 corresponds to AUX1, 10 to AUX2,....14 to AUX615

    while True:
        try:
            pwm_value = int(input("Enter PWM value (500 to 2500): "))
            if pwm_value < 500 or pwm_value > 2500:
                print("Please enter a PWM value between 500 and 2500.")
                continue
            set_servo(mav, servo_channel, pwm_value)
            time.sleep(1)  # Delay for a second before taking new input
        except ValueError:
            print("Invalid input, please enter an integer value for PWM.")
        except KeyboardInterrupt:
            print("\nExiting program.")
            break


if __name__ == "__main__":
    main()
