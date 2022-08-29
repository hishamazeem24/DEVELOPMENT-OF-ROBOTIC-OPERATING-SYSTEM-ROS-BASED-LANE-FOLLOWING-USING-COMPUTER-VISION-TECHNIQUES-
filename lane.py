import cv2
import numpy as np
import math
import time
import rospy
from std_msgs.msg import String, Int32

# defining the region in the image that i want capture and use
def region_of_interest(edges): 
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # only focus lower half of the screen
    polygon = np.array([[
        (0, height),
        (0,  height-175),
        (width , height-175),
        (width , height),
    ]], np.int32)
    
    cv2.fillPoly(mask, polygon, 255)
    
    cropped_edges = cv2.bitwise_and(edges, mask)
    cv2.imshow("roi",cropped_edges)
    
    return cropped_edges


def detect_edges(frame):
    # filter for blue lane lines
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow("GRAY",gray)
    
    #cv2.imshow("mask",mask)
    
    # detect edges
    edges = cv2.Canny(gray, 200, 200)
    cv2.imshow("canny edges",edges)
    
    return edges

# detecting line in the image to be use
def detect_line_segments(cropped_edges):
    rho = 1  
    theta = np.pi / 180  
    min_threshold = 10  
    
    #Use Hough transform to detect the images of the line
    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold, 
                                    np.array([]), minLineLength=5, maxLineGap=150)

    return line_segments

# developing an image of trajectory by the lane image captured
def average_slope_intercept(frame, line_segments):
    lane_lines = []
    
    if line_segments is None:
        print("no line segments detected")
        return lane_lines

    height, width,_ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/3
    left_region_boundary = width * (1 - boundary)
    right_region_boundary = width * boundary
    
    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                print("skipping vertical lines (slope = infinity")
                continue
            
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)
            
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines

#developing the line boundaries of lane image trajectory
def make_points(frame, line):
    height, width, _ = frame.shape
    
    slope, intercept = line
    
    y1 = height  # bottom of the frame
    y2 = int(y1/2)  # make points from middle of the frame down
    
    if slope == 0:
        slope = 0.1
        
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    
    return [[x1, y1, x2, y2]]

#developing an intersection line on lane image for heading line
def display_lines(frame, lines, line_color=(0, 255, 0), line_width=5 ):
    line_image = np.zeros_like(frame)
    
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
                
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    
    return line_image

#calculating the angle of heading line from the image using math.pi
def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=3):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape
    
    steering_angle_radian = steering_angle / 180.0 * math.pi
    
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height /2 )
    
    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)
    
    return heading_image

# developing an angle of heading line on line intersection
def get_steering_angle(frame, lane_lines):
    
    height,width,_ = frame.shape
    
    if len(lane_lines) == 2:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)
        
    elif len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x1 - x2
        y_offset = int(height / 2)
        
    elif len(lane_lines) == 0:
        x_offset = 0
        y_offset = int(height / 2)
        
    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian*180 / math.pi)  
    steering_angle = angle_to_mid_deg + 90
    
    return steering_angle

#Open Cv library to capture image using a webcam/camera
video = cv2.VideoCapture(0)
video.set(cv2.CAP_PROP_FRAME_WIDTH,320)
video.set(cv2.CAP_PROP_FRAME_HEIGHT,240)

time.sleep(1)

##fourcc = cv2.VideoWriter_fourcc(*'XVID')
##out = cv2.VideoWriter('Original15.avi',fourcc,10,(320,240))
speed = 8
lastTime = 0
lastError = 0

kp = 0.4
kd = kp * 0.65

# will display multiple video from capture image depending on the filter used
while True:
    ret,frame = video.read()
    cv2.imshow("original",frame)
    edges = detect_edges(frame)
    roi = region_of_interest(edges)
    line_segments = detect_line_segments(roi)
    lane_lines = average_slope_intercept(frame,line_segments)
    lane_lines_image = display_lines(frame,lane_lines)
    steering_angle = get_steering_angle(frame, lane_lines)
    heading_image = display_heading_line(lane_lines_image,steering_angle)
    # the final image capture with heading display of mobile robot
    cv2.imshow("heading line",heading_image)
    
    #node used to communicate within ROS platform.
    rospy.init_node('direction_display')
    # to send data to subscriber.
    pub=rospy.Publisher('direction', Int32, queue_size=1)
    rate=rospy.Rate(5)

    now = time.time()
    dt = now - lastTime
    deviation = steering_angle - 90
    error = abs(deviation)
    
    #defining the heading direction of the mobile robot
    if deviation < 7.5 and deviation > -7.5:
        deviation = 0
        error = 0
        print("straight")
        directionH= 1
        rospy.loginfo("The output: %s", deviation)
          
    elif deviation > 2:
        print("slight right")
        directionH= 2
        rospy.loginfo("The output: %s", deviation)
    
        if deviation > 5:
            print("right")
            directionH=3
            rospy.loginfo("The output: %s", deviation)

    elif deviation < -2:
        print("slight left")
        directionH= 4
        rospy.loginfo("The output: %s", deviation)
   
        if deviation < -5:
            print("left")
            directionH= 5
            rospy.loginfo("The output: %s", deviation)

    derivative = kd * (error - lastError) / dt
    proportional = kp * error
    PD = int(speed + derivative + proportional)
        
    
    lastError = error
    lastTime = time.time()
    #(directionH) is the topic of the communication
    pub.publish(directionH)    
    rate.sleep()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
   
video.release()
cv2.destroyAllWindows()
