
import cv2
import numpy as np
from picamera2 import Picamera2
import myserial # conected to arduino to send and receive data for controling motors and strings

dw = 320
dh = 240

right_val = 0
left_val = 0
stop_val = 0 
strt_val = 0 
cros_val = 0
walk_val = 0
tunel_val = 0

roi_vertices = np.array([[(0, 240), (160, 170), (320, 240)]], dtype=np.int32)

right = cv2.CascadeClassifier('right.xml') 
left = cv2.CascadeClassifier('turnleft_sign.xml')
stop = cv2.CascadeClassifier('stop_sign1.xml')

pic_stop = cv2.imread('stop.png', cv2.IMREAD_COLOR)
pic_stop = cv2.resize(pic_stop, (50, 50))

pic_strt = cv2.imread('straight.png', cv2.IMREAD_COLOR)
pic_strt = cv2.resize(pic_strt, (40, 40))

pic_right = cv2.imread('right.png', cv2.IMREAD_COLOR)
pic_right = cv2.resize(pic_right, (40, 40))

pic_tunel = cv2.imread('tunnel.jpg', cv2.IMREAD_COLOR)
pic_tunel = cv2.resize(pic_tunel, (50, 50))

pic_walk = cv2.imread('crosswalk.png', cv2.IMREAD_COLOR)
# pic_walk = cv2.resize(pic_walk, (50, 50))

# PID parameters
Kp = 0.012 
Ki = 0.01  
Kd = 0.05  

previous_error = 0
integral = 0

def pid_control(error):
    global previous_error, integral
    # Proportional term
    proportional = error * Kp
    
    # Integral term
    integral += error * Ki
    
    # Derivative term
    derivative = (error - previous_error) * Kd
    previous_error = error
    
    # Total PID output
    output = proportional + integral + derivative
    return output

# Function to detect horizontal lines (used for crosswalk detection)
def detect_horizontal_lines(frame): 
    gray_image = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    edges = cv2.Canny(gray_image, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=100, maxLineGap=10)
    if lines is None:
        return False
    for line in lines:
        x1, y1, x2, y2 = line[0]
        angle = np.arctan2(y2 - y1, x2 - x1) * 180.0 / np.pi
        if abs(angle) < 5 or abs(angle - 180) < 5:
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            return True
    return False

# Initialize camera in raspberypi
camera = Picamera2()
camera.preview_configuration.main.size=(dw,dh)
camera.preview_configuration.main.format="RGB888"
camera.preview_configuration.align()
camera.configure("preview")
camera.start()

target_position = 160  # Center of the image (horizontal axis)
mask_top = 200
mask_bottom = 300

# Main loop
while True:
    frame = camera.capture_array()

    # Mask the region of interest
    mask = np.zeros_like(frame[:, :, 0])
    cv2.fillPoly(mask, roi_vertices, 255)
    masked_image = cv2.bitwise_and(frame, frame, mask=mask)
    mycross = detect_horizontal_lines(masked_image)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Template matching for different signs
    R_strt = cv2.matchTemplate(gray, cv2.cvtColor(pic_strt, cv2.COLOR_BGR2GRAY), cv2.TM_CCOEFF_NORMED)
    min_val_strt, max_val_strt, min_loc_strt, max_loc_strt = cv2.minMaxLoc(R_strt)

    R_tunel = cv2.matchTemplate(gray, cv2.cvtColor(pic_tunel, cv2.COLOR_BGR2GRAY), cv2.TM_CCOEFF_NORMED)
    min_val_tunel, max_val_tunel, min_loc_tunel, max_loc_tunel = cv2.minMaxLoc(R_tunel)

    R_walk = cv2.matchTemplate(gray, cv2.cvtColor(pic_walk, cv2.COLOR_BGR2GRAY), cv2.TM_CCOEFF_NORMED)
    min_val_walk, max_val_walk, min_loc_walk, max_loc_walk = cv2.minMaxLoc(R_walk)

    if max_val_strt > 0.5:
        strt_val = 1
    if max_val_tunel > 0.4:
        tunel_val = 1
    if max_val_walk > 0.5:
        walk_val = 1

    lefts = left.detectMultiScale(frame, scaleFactor=1.1, minNeighbors=4)
    rights = right.detectMultiScale(frame, scaleFactor=1.1, minNeighbors=3)
    stops = stop.detectMultiScale(frame, scaleFactor=1.1, minNeighbors=10)
    if len(rights) > 0:
        right_val = 1
        left_val = 0
    if len(lefts) > 0:
        left_val = 1
        right_val = 0
    if len(stops) > 0:
        stop_val = 1

    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 150, 150)

    h, w  = edges.shape
    trsh = 50
    myserial.send("f") # f means go forward

    mymask = np.zeros_like(edges)
    polygon = np.array([[
        (0, 240), (0,  200), (w, 200), (w , 240)
    ]], np.int32)
        
    cv2.fillPoly(mymask, polygon, 255)
    masked_frame = cv2.bitwise_and(edges, mymask)
    contours, _ = cv2.findContours(masked_frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        max_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(max_contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cv2.circle(frame, (cX, int((mask_bottom - mask_top) / 2) + mask_top), 5, (0, 0, 255), -1)
            cv2.drawContours(frame, [max_contour], -1, (0, 255, 0), 2)
            
            error = cX - target_position

            # PID control
            correction = pid_control(error)
            if correction > 0:
                myserial.send("r")  # Turn right
            elif correction < 0:
                myserial.send("l")  # Turn left
            else:
                myserial.send("f")  # Move forward

    if mycross:
        # handle crosswalk or other detected signs
        pass

    # Stop sign
    if stop_val == 1:
        myserial.send("s")
        stop_val = 0

    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) == ord('q'):
        myserial.send("s")
        break

cv2.destroyAllWindows()
