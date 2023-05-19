import time
import socket
import math
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles
    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(
        rotation_quaternion)
    rotations[robot_id] = rotz


# Connect to the robot
IP_ADDRESS = '192.168.0.204'

# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')


# Move the robot to starting point
def ToStartingPoint(Point):
    try:
        while is_running:
            if robot_id in positions:
                print('Current Position: ', positions[robot_id],
                      'Current Rotation: ', rotations[robot_id])
                print("Destination: " + str(Point))

                X_position = positions[robot_id][0]
                Y_position = positions[robot_id][1]

                X_destination = Point[0]
                Y_destination = Point[1]

                X_difference = X_destination - X_position
                Y_difference = Y_destination - Y_position
                Distance = math.sqrt(X_difference**2 + Y_difference**2)

                rotation = rotations[robot_id]
                Ideal_rotation = np.arctan(Y_difference / X_difference)
                Ideal_rotation = np.rad2deg(Ideal_rotation)

                print("Ideal Rotation: " + str(Ideal_rotation))
                if (X_difference > 0):
                    rotation_diff = Ideal_rotation - rotation
                else:
                    rotation_diff = Ideal_rotation - rotation + 180
                if (rotation_diff > 180):
                    rotation_diff = rotation_diff - 360
                if (rotation_diff < -180):
                    rotation_diff = rotation_diff + 360
                # Calculate the difference between the ideal rotation and the current rotation
                print("Rotation Difference: " + str(rotation_diff))

                Gain_V = 600
                V = 600 + Distance * Gain_V
                print("Speed: " + str(V))
                Gain_Omega = 70
                Omega = rotation_diff * Gain_Omega
                print("Angular Speed: " + str(Omega))

                # Calculate the parameters for the wheels
                u = np.array([V - Omega, V + Omega])
                u[u > 1500] = 1500
                u[u < -1500] = -1500
                print("Moter Parameters: ", u[0], u[0], u[1], u[1])
                command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (u[0], u[0], u[1], u[1])
                s.send(command.encode('utf-8'))
                time.sleep(0.1)

                # Check if the robot has reached the destination
                if (Distance < 0.1):
                    print("Destination Reached")
                    command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (0, 0, 0, 0)
                    s.send(command.encode('utf-8'))
                    time.sleep(4)
                    break
            else:
                print("Robot not found")
                time.sleep(1)
    except KeyboardInterrupt:
        streaming_client.shutdown()
        command = 'CMD_MOTOR#00#00#00#00\n'
        s.send(command.encode('utf-8'))
        s.shutdown(2)
        s.close()


if __name__ == "__main__":
    clientAddress = "192.168.0.156"
    optitrackServerAddress = "192.168.0.172"
    robot_id = 204

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()

    positions = {}
    rotations = {}
    # Rescue_Center = [0.0, 0.0]
    print("Move the robot to rescue center")
    # ToStartingPoint(Rescue_Center)
    print("Robot is at rescue center")

    Duck_Counter = 0
    command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (1300, 1300, -1300, -1300)
    s.send(command.encode('utf-8'))
    time.sleep(3)

    command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (0, 0, 0, 0)
    s.send(command.encode('utf-8'))
    time.sleep(1)
    # Quit the program
    s.shutdown(2)
    quit()

    try:
        while is_running:
            if robot_id in positions:
                print('Current Position: ', positions[robot_id],
                      'Current Rotation: ', rotations[robot_id])

                X_position = positions[robot_id][0]
                Y_position = positions[robot_id][1]

                X_destination = Path[i][0]
                Y_destination = Path[i][1]

                X_difference = X_destination - X_position
                Y_difference = Y_destination - Y_position
                Distance = math.sqrt(X_difference**2 + Y_difference**2)

                rotation = rotations[robot_id]
                Ideal_rotation = np.arctan(Y_difference / X_difference)
                Ideal_rotation = np.rad2deg(Ideal_rotation)

                print("Ideal Rotation: " + str(Ideal_rotation))
                if (X_difference > 0):
                    rotation_diff = Ideal_rotation - rotation
                else:
                    rotation_diff = Ideal_rotation - rotation + 180
                if (rotation_diff > 180):
                    rotation_diff = rotation_diff - 360
                if (rotation_diff < -180):
                    rotation_diff = rotation_diff + 360
                # Calculate the difference between the ideal rotation and the current rotation
                print("Rotation Difference: " + str(rotation_diff))

                Gain_V = 600
                V = 600 + Distance * Gain_V
                print("Speed: " + str(V))
                Gain_Omega = 70
                Omega = rotation_diff * Gain_Omega
                print("Angular Speed: " + str(Omega))

                # Calculate the parameters for the wheels
                u = np.array([V - Omega, V + Omega])
                u[u > 1500] = 1500
                u[u < -1500] = -1500
                print("Moter Parameters: ", u[0], u[0], u[1], u[1])
                command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (u[0], u[0], u[1], u[1])
                s.send(command.encode('utf-8'))
                time.sleep(0.1)

                # Check if the robot has reached the destination
                if (Distance < 0.1):
                    print(Path[i], "Reached")
                    command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (0, 0, 0, 0)
                    s.send(command.encode('utf-8'))
                    time.sleep(0.2)
                    i += 1
                if (i == Length):
                    command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (0, 0, 0, 0)
                    s.send(command.encode('utf-8'))
                    time.sleep(0.2)
                    break
    except KeyboardInterrupt:
        streaming_client.shutdown()
        command = 'CMD_MOTOR#00#00#00#00\n'
        s.send(command.encode('utf-8'))
        s.shutdown(2)
        s.close()
command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (0, 0, 0, 0)
s.send(command.encode('utf-8'))
time.sleep(1)
