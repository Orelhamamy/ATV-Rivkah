import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
from dynamixel_sdk_custom_interfaces.msg import SetBrake
import subprocess
import time
import serial  # Import pyserial to enable serial communication

MaxSpeed = 9  # m/s (Max speed for the vehicle)

class JoyControl(Node):
    def __init__(self):
        # Initialize the node with the name 'joy_control'
        super().__init__('joy_control')
        
        # Create a subscription to receive Joy messages from the 'joy' topic
        self.subscription = self.create_subscription(
            Joy,  # The message type we are subscribing to
            'joy',  # The topic name for joystick data
            self.joy_callback,  # The callback function to handle incoming messages
            10  # Queue size for the subscription
        )
        
        # Create a publisher for sending AckermannDrive messages to the 'ackermann_drive' topic
        self.ackermann_pub = self.create_publisher(AckermannDriveStamped, 'ackermann_drive', 10)
        self.brake_pub = self.create_publisher(SetBrake, 'set_brake', 10)
        
        # Initialize control variables for forwards, backwards, left, and right movement
        self.forwards = 0
        self.backwards = 0
        self.left = 0
        self.right = 0
        self.stop = False
        self.speed_restriction = True  # Initialize the speed_restriction state
        self.max_brake = 10
        self.brake_flag = False
        # Define the dead zone threshold for joystick axis input (ignores small input values)
        self.dead_zone = 0.05

        # Set up serial communication with both Teensy devices (Throttle and Steering)
        # self.ser_throttle = serial.Serial('/dev/ThrottleTeensy', 115200)  # Throttle Teensy serial port
        # self.ser_steering = serial.Serial('/dev/SteeringTeensy', 115200)  # Steering Teensy serial port

    def joy_callback(self, msg):
        # Create the AckermannDrive message to control the vehicle
        ackermann_msg = AckermannDriveStamped()
        brake_msg = SetBrake()
        brake_msg.id = 1

        # Axis indices for the joystick (these can vary depending on the joystick used)
        r2_axis = 5   # Right trigger (R2)
        l2_axis = 2   # Left trigger (L2)
        right_stick_horizontal_axis = 0  # Right stick horizontal axis 
        A_button = 0 # A button 
        B_button = 1
        left_stick_vertical_axis = 1 # controll the rear brake
        L1_button = 4  # L1 button
        R1_button = 5  # R1 button

        # Get the values of the joystick axes
        r2_value = msg.axes[r2_axis]  # R2 trigger value, ranges from -1 (unpressed) to 1 (fully pressed)
        l2_value = msg.axes[l2_axis]  # L2 trigger value, ranges from -1 (unpressed) to 1 (fully pressed)
        right_stick_value = msg.axes[right_stick_horizontal_axis]  # Right stick horizontal value, ranges from -1 (left) to 1 (right)
        A_button_value = msg.buttons[A_button] # A button value, 0 when released and 1 when pressed
        B_button_value = msg.buttons[B_button]
        L1_button_value = msg.buttons[L1_button] # L1 button value
        R1_button_value = msg.buttons[R1_button] # R1 button value
        
        brake_value = msg.axes[left_stick_vertical_axis]
        # Toggle speed_restriction when both L1 and R1 are pressed simultaneously
        if L1_button_value == 1 and R1_button_value == 1:
            self.speed_restriction = not self.speed_restriction  # Toggle the value

	    # -----------------THIS IS WORNG!!! -------------- START -------------------
        # Handle the forwards motion using the R2 trigger
        if abs(1 - r2_value) > self.dead_zone:  # Only consider values greater than the dead zone
            self.forwards = MaxSpeed * (1 - r2_value)  # Scale to MaxSpeed
        else:
            self.forwards = MaxSpeed  # Default to max speed if R2 is not pressed

        # Handle the backwards motion using the L2 trigger
        if l2_value > self.dead_zone:  # Only consider values greater than the dead zone
            self.backwards = MaxSpeed * (1 - l2_value)  # Scale to MaxSpeed
        else:
            self.backwards = MaxSpeed  # Default to max speed if L2 is not pressed
        # THIS IS WORNG!!! -------------- END --------------------

        
        # Handle left/right steering based on the right joystick (scaled to 0-30 for steering angle)
        if right_stick_value > 0:  # Right stick pushed right, steering left
            self.left = int(abs(right_stick_value) * 30)  # Convert to steering angle (0 to 30)
            self.right = 0  # Reset right steering
        elif right_stick_value < 0:  # Right stick pushed left, steering right
            self.right = int(abs(right_stick_value) * 30)  # Convert to steering angle (0 to 30)
            self.left = 0  # Reset left steering
        else:  # If the stick is centered, no steering
            self.left = 0
            self.right = 0

        # Handle the stop condition based on A button press
        self.stop = A_button_value == 1  # Stop if the A button is pressed

        if abs(brake_value) > self.dead_zone:
            brake_msg.brake = int(10 * abs(brake_value))
        # If stop is True, set speed to 0 (as float) and stop steering as well
        if self.stop:
            ackermann_msg.drive.speed = 0.0  # Ensure it's a float
        else:
            # Set the steering angle: negative left, positive right (adjust for steering direction)
            ackermann_msg.drive.steering_angle = float(-self.left if self.left > 0 else self.right)  # Adjust steering direction
            # Set the speed: positive for forwards, negative for backwards
            ackermann_msg.drive.speed = float(self.forwards if self.backwards == 0 else -self.backwards)  # Adjust speed

        # Log the control values to the console for debugging purposes
        self.get_logger().info(f'Forwards: {self.forwards}, Backwards: {self.backwards}, Left: {self.left}, Right: {self.right}, Stop: {self.stop}, Speed Restriction: {self.speed_restriction}')

        # Publish the AckermannDrive message to the 'ackermann_drive' topic
        self.ackermann_pub.publish(ackermann_msg)
        self.brake_pub.publish(brake_msg)

        # Prepare the Ackermann message for serial communication
        ackermann_serial = f"steering_angle:{ackermann_msg.drive.steering_angle}, speed:{ackermann_msg.drive.speed}, stop:{self.stop}, speed_restriction:{int(self.speed_restriction)}\n"

        # Send the message over the serial connection to the steering Teensy
        # self.ser_steering.write(ackermann_serial.encode())  # Encoding the string to bytes before sending

        # Send the message over the serial connection to the throttle Teensy
        # self.ser_throttle.write(ackermann_serial.encode())  # Same message for throttle Teensy (if required)

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Launch the 'joy' node using subprocess to read joystick input
    joy_node_process = subprocess.Popen(['ros2', 'run', 'joy', 'joy_node'])
    
    # Wait a short time to ensure the joy_node is up and running
    time.sleep(2)

    # Create the JoyControl node that processes joystick input and publishes drive commands
    joy_control = JoyControl()

    try:
        # Spin the node to keep the ROS 2 system running and processing events
        rclpy.spin(joy_control)
    except KeyboardInterrupt:
        pass
    finally:
        # Properly terminate the joy_node subprocess when exiting
        joy_node_process.terminate()  # Kill the joy_node process
        joy_node_process.wait()  # Wait for it to terminate completely
        
        # Destroy the JoyControl node and shut down ROS 2 client library
        joy_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    # Start the main function if this file is being executed directly
    main()
