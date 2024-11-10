from controller import Robot, DistanceSensor, Motor

# Create the robot instance
robot = Robot()

# Time step for the simulation
time_step = int(robot.getBasicTimeStep())

# Get the motors and set them to initial speed
left_motor = robot.getMotor('left wheel motor')
right_motor = robot.getMotor('right wheel motor')
base_speed = 5.0  # Base speed within the max velocity limit
max_velocity = 6.28  # Maximum motor velocity

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Get distance sensors for line following
ir_sensors = [
    robot.getDistanceSensor('ps0'),
    robot.getDistanceSensor('ps1'),
    robot.getDistanceSensor('ps2'),
    robot.getDistanceSensor('ps3'),
    robot.getDistanceSensor('ps4')
]
for sensor in ir_sensors:
    sensor.enable(time_step)

# Get proximity sensors for obstacle detection
proximity_sensors = [
    robot.getDistanceSensor('ps5'),
    robot.getDistanceSensor('ps6'),
]
for sensor in proximity_sensors:
    sensor.enable(time_step)

# PID variables
Kp = 0.5
Ki = 0.1
Kd = 0.05
integral = 0
prev_error = 0

while robot.step(time_step) != -1:
    # Read IR sensor values
    sensor_values = [sensor.getValue() for sensor in ir_sensors]
    print(sensor_values)  # Check sensor values

    # Check if the sum of sensor values is greater than a small threshold
    if sum(sensor_values) > 0.2:  # Adjust threshold as needed
        line_position = (sensor_values[0] * 0 + sensor_values[1] * 1 +
                     sensor_values[2] * 2 + sensor_values[3] * 3 + sensor_values[4] * 4) / sum(sensor_values)
    else:
        line_position = 2  # Default to center if no line detected


    target_position = 2  # Target position for center
    error = target_position - line_position

    # PID calculations
    integral += error
    derivative = error - prev_error
    correction = Kp * error + Ki * integral + Kd * derivative

    # Calculate left and right motor velocities
    left_velocity = base_speed - correction
    right_velocity = base_speed + correction

    # Obstacle avoidance logic
    obstacle_detected = any(sensor.getValue() < 0.1 for sensor in proximity_sensors)  # Adjust threshold as needed

    if obstacle_detected:
        # Reverse briefly to create space
        left_motor.setVelocity(-base_speed / 2)  # Reverse left motor
        right_motor.setVelocity(-base_speed / 2)  # Reverse right motor
        robot.step(200)  # Wait for a short time to reverse

        # Turn away from the obstacle (adjust left/right as necessary)
        left_motor.setVelocity(base_speed / 2)  # Turn left
        right_motor.setVelocity(-base_speed / 2)  # Turn right
        robot.step(1000)  # Turn for a short duration

        # Resume line following
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
    else:
        # Clamp the velocities to avoid exceeding max limits
        left_motor.setVelocity(max(min(left_velocity, max_velocity), -max_velocity))
        right_motor.setVelocity(max(min(right_velocity, max_velocity), -max_velocity))

    prev_error = error
