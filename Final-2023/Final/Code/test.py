import time
import socket
import math
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import cv2 as cv


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles
    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)
    rotations[robot_id] = rotz


# Move the robot to starting point
def ToStartingPoint(Point):
    try:
        while is_running:
            if robot_id in positions:
                print(
                    "Current Position: ",
                    positions[robot_id],
                    "Current Rotation: ",
                    rotations[robot_id],
                )
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
                if X_difference > 0:
                    rotation_diff = Ideal_rotation - rotation
                else:
                    rotation_diff = Ideal_rotation - rotation + 180
                if rotation_diff > 180:
                    rotation_diff = rotation_diff - 360
                if rotation_diff < -180:
                    rotation_diff = rotation_diff + 360
                # Calculate the difference between the ideal rotation and the current rotation
                print("Rotation Difference: " + str(rotation_diff))

                Gain_V = 100
                V = 600 + Distance * Gain_V
                print("Speed: " + str(V))
                Gain_Omega = 50
                print("rt:" + str(rotation_diff))
                Omega = rotation_diff * Gain_Omega
                if(Omega > 1500):
                    Omega = 1500
                print("Angular Speed: " + str(Omega))

                # Calculate the parameters for the wheels
                u = np.array([V - Omega, V + Omega])
                u[u > 1500] = 1500
                u[u < -1500] = -1500
                print("Moter Parameters: ", u[0], u[0], u[1], u[1])
                command = "CMD_MOTOR#%d#%d#%d#%d\n" % (u[0], u[0], u[1], u[1])
                s.send(command.encode("utf-8"))
                time.sleep(0.1)

                # Check if the robot has reached the destination
                if Distance < 0.2:
                    print("Destination Reached")
                    command = "CMD_MOTOR#%d#%d#%d#%d\n" % (0, 0, 0, 0)
                    s.send(command.encode("utf-8"))
                    time.sleep(1)
                    break
            else:
                print("Robot not found")
                time.sleep(1)
    except KeyboardInterrupt:
        streaming_client.shutdown()
        command = "CMD_MOTOR#00#00#00#00\n"
        s.send(command.encode("utf-8"))
        s.shutdown(2)
        s.close()

X = 0
size = 0
# Search for the duck in the camera
def SearchDuck():
    timer = 0
    print("Searching for duck......")
    X = 0
    size = 0
    Flag = 0
    try:
        while True:
            print("Timer: " + str(timer))
            cap = cv.VideoCapture("http://192.168.0.204:1234/stream.mjpg")
            ret, frame = cap.read()
            # Camera Part
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            # Threshold of blue in HSV space
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([30, 255, 255])
            # preparing the mask to overlay
            mask = cv.inRange(hsv, lower_yellow, upper_yellow)
            result = cv.bitwise_and(frame, frame, mask=mask)
            gray = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
            gray = cv.medianBlur(gray, 5)
            params = cv.SimpleBlobDetector_Params()
            # Change thresholds
            params.minThreshold = 60
            params.maxThreshold = 150
            # Filter by Area.
            params.filterByArea = True
            params.minArea = 200
            params.maxArea = 900000
            # color filter
            params.filterByColor = True
            params.blobColor = 255
            # Filter by Circularity
            params.filterByCircularity = True
            params.minCircularity = 0.5
            # Filter by Convexity
            params.filterByConvexity = True
            params.minConvexity = 0.5
            # Filter by Inertia
            params.filterByInertia = False
            params.minInertiaRatio = 0.01
            # Create a detector with the parameters
            detector = cv.SimpleBlobDetector_create(params)
            # set up keypoints
            keypoints = detector.detect(gray)
            print("The keypoints are: " + str(keypoints))
            # Rotate if there is no keypoints
            if not keypoints and Flag == 0:
                timer = timer + 0.23
                command = "CMD_MOTOR#%d#%d#%d#%d\n" % (1200, 1200, -1200, -1200)
                s.send(command.encode("utf-8"))
                time.sleep(0.23)
                s.send(command.encode("utf-8"))
            elif not keypoints and Flag != 0:
                Flag = Flag - 1
                command = "CMD_MOTOR#%d#%d#%d#%d\n" % (0, 0, 0, 0)
                s.send(command.encode("utf-8"))
            else:
                Flag = 2
                command = "CMD_MOTOR#%d#%d#%d#%d\n" % (0, 0, 0, 0)
                s.send(command.encode("utf-8"))
                im_with_keypoints = cv.drawKeypoints(
                    gray,
                    keypoints,
                    np.array([]),
                    (0, 0, 255),
                    cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
                )
                cv.imshow("Keypoints", im_with_keypoints)
                # Show keypoints
                if cv.waitKey(1) & 0xFF == ord("q"):
                    break
                if X not in keypoints:
                    X = keypoints[0].pt[0]
                    size = keypoints[0].size
                print("The size is: " + str(size))
                print("The X coordinate is: " + str(X))
                X_difference = X - 320
                error = 30
                if X_difference > error:
                    print("Turn Right")
                    command = "CMD_MOTOR#%d#%d#%d#%d\n" % (1100, 1100, -1100, -1100)
                    s.send(command.encode("utf-8"))
                    time.sleep(0.08)
                    command = "CMD_MOTOR#%d#%d#%d#%d\n" % (0, 0, 0, 0)
                    s.send(command.encode("utf-8"))
                if X_difference < -error:
                    print("Turn Left")
                    command = "CMD_MOTOR#%d#%d#%d#%d\n" % (-1100, -1100, 1100, 1100)
                    s.send(command.encode("utf-8"))
                    time.sleep(0.08)
                    command = "CMD_MOTOR#%d#%d#%d#%d\n" % (0, 0, 0, 0)
                    s.send(command.encode("utf-8"))
                if X_difference < error and X_difference > -error:
                    print("Go Straight")
                    speed = 1800 - 10 * size
                    if speed < 700:
                        speed = 700
                    command = "CMD_MOTOR#%d#%d#%d#%d\n" % (speed, speed, speed, speed)
                    s.send(command.encode("utf-8"))
                    time.sleep(0.5)
                    command = "CMD_MOTOR#%d#%d#%d#%d\n" % (0, 0, 0, 0)
                    s.send(command.encode("utf-8"))
            if timer >= 5:
                print("Cargo Full")
                command = "CMD_MOTOR#%d#%d#%d#%d\n" % (0, 0, 0, 0)
                s.send(command.encode("utf-8"))
                time.sleep(2)
                break
        if not cap.isOpened():
            print("Error opening video stream or file")
            exit()
    except KeyboardInterrupt:
        # Shut down the entire program
        streaming_client.shutdown()
        command = "CMD_MOTOR#00#00#00#00\n"
        s.send(command.encode("utf-8"))
        s.shutdown(2)
        s.close()


def inrange(Point, Dest):
    x = Point[0]
    y = Point[1]
    dx = Dest[0]
    dy = Dest[1]
    if abs(x - dx) < 0.6 and abs(y - dy) < 0.6:
        return True
    else:
        return False


def findlocation(Box1, Box2, Path):
    test = ([1,0],[0,1],[-1,0],[0,-1])
    for i in range(4):
        Location = Path[-1]
        Location = [Location[0] + test[i][0], Location[1] + test[i][1]]
        for p in Path:
            if inrange(Location, p):
                continue
        if inrange(Location, Box1) or inrange(Location, Box2):
            continue
        return Location
    print("No Location")
    return None

# Connect to the robot
IP_ADDRESS = "192.168.0.204"

# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print("Connected")

if __name__ == "__main__":
    clientAddress = "192.168.0.41"
    optitrackServerAddress = "192.168.0.04"
    robot_id = 304

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
    Path = []  # Poitns robots have reached
    Rescue_Center = [5.4, 0]
    P1 = [3.7, 0.1]
    P2 = [2.1, 0.1]
    P3 = [-1.3, -0.11]
    PX = [0.93, 0.72]
    print("Move the robot to rescue center")
    #ToStartingPoint([0,0])

    ToStartingPoint(Rescue_Center)
    print("Robot is at rescue center")
    print("Move the robot to P1")
    ToStartingPoint(P1)
    print("Robot is at P1")
    print("Move the robot to P2")
    ToStartingPoint(P2)
    print("Robot is at P2")
    print("Move the robot to P3")
    ToStartingPoint(P3)
    print("Robot is at P3")
    series = ([-3.39, 0.81], [-1.18, 3], [0.13, 1.42])

    for i in range(3):
        SearchDuck()
        try:
            while is_running:
                if robot_id in positions:
                    print(
                        "Current Position: ",
                        positions[robot_id],
                        "Current Rotation: ",
                        rotations[robot_id],
                    )
                    # Add position to the Path
                    Path.append(positions[robot_id])
                    break
                else:
                    print("Robot not found")
        except KeyboardInterrupt:
            streaming_client.shutdown()
            command = "CMD_MOTOR#00#00#00#00\n"
            s.send(command.encode("utf-8"))
            s.shutdown(2)
            s.close()
        print("Going to next location")
        location = series[i]
        # findlocation(Box1, Box2, Path)
        ToStartingPoint(location)
    
    ToStartingPoint(PX)
    print("Move the robot to rescue center")
    ToStartingPoint(Rescue_Center)
    print("Robot is at rescue center")
    command = "CMD_MOTOR#%d#%d#%d#%d\n" % (-1500, -1500, -1500, -1500)
    s.send(command.encode("utf-8"))
    command = "CMD_MOTOR#%d#%d#%d#%d\n" % (0, 0, 0, 0)
    s.send(command.encode("utf-8"))
    time.sleep(1)
    # Quit the program
    s.shutdown(2)
    quit()
command = "CMD_MOTOR#%d#%d#%d#%d\n" % (0, 0, 0, 0)
s.send(command.encode("utf-8"))
time.sleep(1)
