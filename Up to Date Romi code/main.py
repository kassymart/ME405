import task_share
import cotask
from motor import Motor
from encoder import Encoder
from imu import BNO055
from pyb import Pin, Timer, I2C
from bump import BumpSensor

def motor_task(shares):
    '''
    This motor task uses a segment-based state machine.
    Each segment is either a drive (with a target distance and desired heading)
    or a turn (with a target heading).
    '''
    state = 'INIT'
    segment_index = 0
    L1 = 30
    L2 = L1*(34.5/86.5)
    L3 = L1*(57/86.5)
    L4 = L1*(69/86.5)
    L5 = L1*(35/86.5)
    L6 = L1*(80.5/86.5)
    L7 = L1*(34.5/86.5)
    L8 = L1*(40.5/86.5)
    L9 = L1*(25.5/86.5)
    L10 = L1*(47/86.5)
    segments = [
        {"segment_num": 1, "type": "drive", "distance": L1, "heading": 0},    # Drive forward at 0°
        {"segment_num": 2, "type": "turn",  "heading": 90},                    # Turn right to 90°
        {"segment_num": 3, "type": "drive", "distance": L2, "heading": 90},    # Drive forward to checkpoint 1
        {"segment_num": 4, "type": "turn",  "heading": -36},                   # Rotate to checkpoint 2 angle
        {"segment_num": 5, "type": "drive", "distance": L3, "heading": -36},   # Drive forward to checkpoint 2
        {"segment_num": 6, "type": "turn",  "heading": 96},                    # Rotate to checkpoint 3 angle
        {"segment_num": 7, "type": "drive", "distance": L4, "heading": 96},    # Drive forward to checkpoint 3
        {"segment_num": 8, "type": "turn",  "heading": -167},                  # Rotate to point toward checkpoint 4
        {"segment_num": 9, "type": "drive", "distance": L5, "heading": -167},   # Drive forward to checkpoint 4
        {"segment_num": 10, "type": "turn",  "heading": 180},                  # Rotate to 180°
        {"segment_num": 11, "type": "drive", "distance": L6, "heading": 180},   # Drive forward a set distance
        {"segment_num": 12, "type": "turn",  "heading": -90},                  # Turn right to 90°
        {"segment_num": 13, "type": "drive", "distance": L7, "heading": -90},   # Drive straight after turn
        {"segment_num": 14, "type": "turn",  "heading": 0},                    # Turn right to 0°
        {"segment_num": 15, "type": "drive", "distance": L8, "heading": 0},     # Drive straight after turn
        {"segment_num": 16, "type": "turn",  "heading": -90},                  # Turn right to -90°
        {"segment_num": 17, "type": "drive", "distance": L9, "heading": -90},   # Drive straight after turn
        {"segment_num": 18, "type": "turn",  "heading": 180},                  # Turn right to 180°
        {"segment_num": 19, "type": "drive", "distance": L10, "heading": 180},   # Drive straight after turn
        {"segment_num": 20, "type": "END"}
    ]
    
    # Constants for motor control
    base_effort = 30
    Kp_heading = 100   # Gain for heading correction during drive
    Kp_vel = 7       # Gain for position correction during drive
    Kp_turn = 15
    Ki_turn = 2  # Integral gain for turn state
    tolerance = 0.1          # Heading error tolerance in degrees
    
    # Variables to store drive start positions
    drive_start_left = 0
    drive_start_right = 0
    turn_integral = 0
    prev_turn_error = 0

    # Motor and encoder objects
    left_motor = None
    right_motor = None
    left_encoder = None
    right_encoder = None
    bump_sensor = None

    while True:
        if state == 'INIT':
            # Initialize motors and encoders
            left_motor = Motor(Pin.board.PB4, Pin.cpu.H0, Pin.cpu.H1, 3, 1)
            right_motor = Motor(Pin.board.PB5, Pin.cpu.C10, Pin.cpu.C12, 3, 2)
            left_encoder = Encoder(Timer(1, period=0xFFFF, prescaler=0), Pin(Pin.cpu.A8), Pin(Pin.cpu.A9))
            right_encoder = Encoder(Timer(8, period=0xFFFF, prescaler=0), Pin(Pin.cpu.C6), Pin(Pin.cpu.C7))
            bump_sensor = BumpSensor(Pin.cpu.C4, debounce_time=5)
            left_motor.enable()
            right_motor.enable()
            left_encoder.reset()
            right_encoder.reset()
            state = 'WAIT_FOR_CALIBRATION'
            yield

        elif state == 'WAIT_FOR_CALIBRATION':
            # Wait until the IMU is calibrated (and other sensors if needed)
            if imu_calibration_flag.get() == 1:
                state = 'SEGMENT'
            yield

        elif state == 'SEGMENT':
            # If all segments are complete, stop.
            if segment_index >= len(segments):
                left_motor.set_effort(0)
                right_motor.set_effort(0)
                state = 'DONE'
                yield
            current_segment = segments[segment_index]
            print(f"Segment {current_segment['segment_num']}: {current_segment['type']}")
            # When starting a drive segment, record encoder positions.
            if current_segment["type"] == "drive":
                drive_start_left = left_encoder.get_position()
                left_pos.put(drive_start_left)
                drive_start_right = right_encoder.get_position()
                right_pos.put(drive_start_right)
                state = 'DRIVE'
            elif current_segment["type"] == "turn":
                state = 'TURN'
            elif current_segment["type"] == "END":
                state = 'DONE'
            yield

        elif state == 'DRIVE':
            # Update encoders to track distance traveled.
            left_motor.set_effort(base_effort)
            right_motor.set_effort(base_effort)

            # Drive with heading correction.
            desired_heading = segments[segment_index]["heading"]

            heading_error = abs(desired_heading) - (current_heading.get())

            #print(f"Heading Error: {heading_error}")

            correction = int(Kp_heading * heading_error)
            correction = max(min(correction, 15), -15)
            new_left_effort = base_effort + correction
            new_right_effort = base_effort - correction
            left_motor.set_effort(int(new_left_effort))
            right_motor.set_effort(int(new_right_effort))

            left_encoder.update()
            right_encoder.update()
            current_left = left_encoder.get_position()
            left_pos.put(current_left)
            current_right = right_encoder.get_position()
            right_pos.put(current_right)
            avg_distance.put(((current_left - drive_start_left) + (current_right - drive_start_right)) / 2)

            left_velocity.put(left_encoder.get_velocity())
            right_velocity.put(right_encoder.get_velocity())

            vel_error = left_velocity.get() - right_velocity.get()
            #print(f"Position Error: {pos_error}")

            correction = int(Kp_vel * vel_error)
            correction = max(min(correction, 5), -5)
            new_left_effort = base_effort - correction
            new_right_effort = base_effort + correction
            left_motor.set_effort(int(new_left_effort))
            right_motor.set_effort(int(new_right_effort))

            left_encoder.update()
            right_encoder.update()
            # print(f"Left Effort: {new_left_effort} Right Effort: {new_right_effort}")
            # print(f"left velocity: {left_velocity.get()} right velocity: {right_velocity.get()}")

            # If the target distance is reached, stop, reset encoders, and move to the next segment.
            if abs(avg_distance.get()) >= segments[segment_index]["distance"]:
                left_motor.set_effort(0)
                right_motor.set_effort(0)
                left_encoder.reset()   # Reset the left encoder
                right_encoder.reset()  # Reset the right encoder
                segment_index += 1
                state = 'SEGMENT'
            yield

            if bump_sensor is not None:
                bump_sensor.update()
                if bump_sensor.pressed:
                    print("Collision detected! Emergency stop.")
                    left_motor.set_effort(0)
                    right_motor.set_effort(0)
                    left_motor.set_effort(-30)
                    right_motor.set_effort(-30)
                  #  state = 'BACKING'
                  #  backup_start_time = pyb.millis()



        elif state == 'TURN':
            # In a turn segment, rotate until the desired heading is reached.
            desired_heading = segments[segment_index]["heading"]
            heading_error = desired_heading - current_heading.get()

            if heading_error < -180:
                heading_error += 360
            elif heading_error > 180:
                heading_error -= 360

            # Reset the integral if the error is near zero (deadband) or if a sign change occurs (only when prev_turn_error is non-zero)
            if abs(heading_error) < 1:
                heading_error = 0
                turn_integral = 0
            elif prev_turn_error != 0 and heading_error * prev_turn_error < 0:
                turn_integral = 0

            prev_turn_error = heading_error
            turn_integral += heading_error  # Accumulate error for integral control

            if abs(heading_error) <= tolerance:
                # Turn complete
                left_motor.set_effort(0)
                right_motor.set_effort(0)
                left_encoder.reset()   # Reset the left encoder
                right_encoder.reset()  # Reset the right encoder
                segment_index += 1
                state = 'SEGMENT'
                yield
            else:
                # Proportional control for turning: apply effort based on error.
                turn_effort = Kp_turn * heading_error + Ki_turn * turn_integral 
                # Saturate efforts between -25 and 25.
                turn_effort = max(min(turn_effort, 10), -10)
                left_motor.set_effort(int(turn_effort))
                right_motor.set_effort(int(-turn_effort))

                yield

        elif state == 'DONE':
            # All segments completed – ensure the robot is stopped.
            left_motor.set_effort(0)
            right_motor.set_effort(0)
            left_motor.disable()
            right_motor.disable()
            yield


def IMU_task(shares):
    ''' Function to be run by the IMU task. The IMU task will read the magnetometer data and determine the heading '''
    state = 'INIT'
    imu = None

    while True:
        if state == 'INIT':
            # Initialize IMU
            i2c = I2C(3, I2C.CONTROLLER, baudrate=400000)
            imu = BNO055(i2c)
            imu.set_mode(BNO055.NDOF_MODE)  # Set IMU to NDOF mode
            state = 'CALIBRATION'
            yield

        elif state == 'CALIBRATION':
            cal_status = imu.read_cal_status()
            print(f"Calibration Status: {cal_status}")
            # Check if IMU is calibrated
            if cal_status['sys'] == 3 and cal_status['mag'] == 3:
                if imu_calibration_flag.get() == 0:
                    if input("IMU is calibrated. Press enter to continue: ") == '':
                        imu_calibration_flag.put(1)
                        starting_heading.put(imu.heading())
                        state = 'READ_HEADING'
            yield

        elif state == 'READ_HEADING':
            # Read heading from the IMU
            raw_heading = imu.heading()  # Raw heading between 0-360
            # Calculate relative heading with starting heading as zero reference.
            relative_heading = normalize_angle(raw_heading - starting_heading.get())
            current_heading.put(relative_heading)
            yield

def data_task(shares):
    ''' Function to be run by the data task. The data task will read the value from the share and print it to the terminal '''
    state = 'INIT'
    while True:
        if state == 'INIT':
            state = 'PRINT_DATA'
            yield

        elif state == 'PRINT_DATA':
            # Print motor velocities and sensor values to the terminal
            #print(f"Left Motor Velocity: {left_velocity.get()} Right Motor Velocity: {right_velocity.get()}") 
            #print(f"Avg Distance: {avg_distance.get()} Left Position: {left_pos.get()} Right Position: {right_pos.get()}")
            print(f"Current Heading: {current_heading.get()}")
            yield

def normalize_angle(angle):
    """Normalize an angle (in degrees) to the range -180 to 180."""
    return (angle + 180) % 360 - 180

if __name__ == "__main__":

    # Define shared variables
    left_effort = task_share.Share('i', thread_protect=False, name="Left Motor Effort")
    right_effort = task_share.Share('i', thread_protect=False, name="Right Motor Effort")
    left_velocity = task_share.Share('f', thread_protect=False, name="Left Motor Velocity")
    right_velocity = task_share.Share('f', thread_protect=False, name="Right Motor Velocity")
    left_pos = task_share.Share('f', thread_protect=False, name="Left Position")
    right_pos = task_share.Share('f', thread_protect=False, name="Right Position")
    imu_calibration_flag = task_share.Share('i', thread_protect=False, name="IMU Calibration Flag")
    current_heading = task_share.Share('f', thread_protect=False, name="Current Heading")
    starting_heading = task_share.Share('f', thread_protect=False, name="Starting Heading")
    avg_distance = task_share.Share('f', thread_protect=False, name="Average Distance")

    # Initialize shared variableS
    imu_calibration_flag.put(0)

    # Create tasks
    motor_task = cotask.Task(motor_task, name="Motor Task", priority=1, period=50, profile=True, trace=False, shares=(left_effort, right_effort, left_velocity, right_velocity, imu_calibration_flag, current_heading, starting_heading, left_pos, right_pos, avg_distance))
    data_task = cotask.Task(data_task, name="Data Task", priority=1, period=100, profile=True, trace=False, shares=(left_effort, right_effort, left_velocity, right_velocity, current_heading, left_pos, right_pos, avg_distance))
    IMU_task = cotask.Task(IMU_task, name="IMU Task", priority=1, period=25, profile=True, trace=False, shares=(current_heading, imu_calibration_flag))
    
    # Add tasks to the task list
    cotask.task_list.append(motor_task)
    cotask.task_list.append(data_task)
    cotask.task_list.append(IMU_task)

    # Run the scheduler
    while True:
        cotask.task_list.pri_sched()  # Run the priority scheduler