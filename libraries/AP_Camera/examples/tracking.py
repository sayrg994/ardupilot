import cv2
import threading
from pymavlink import mavutil
import math 
from ultralytics import YOLO

master = mavutil.mavlink_connection('127.0.0.1:14560')
master.wait_heartbeat()
print("Heartbeat from the system (system %u component %u)" % 
     (master.target_system, master.target_component))
def send_gimbal_manager_pitch_yaw_angles(target_system, target_component, flags, gimbal_device_id, pitch, yaw, pitch_rate, yaw_rate):
    """
    Send gimbal manager pitch and yaw angles message.

    :param target_system: System ID
    :param target_component: Component ID
    :param flags: High level gimbal manager flags to use
    :param gimbal_device_id: Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components
    :param pitch: Pitch angle (positive: up, negative: down, NaN to be ignored)
    :param yaw: Yaw angle (positive: to the right, negative: to the left, NaN to be ignored)
    :param pitch_rate: Pitch angular rate (positive: up, negative: down, NaN to be ignored)
    :param yaw_rate: Yaw angular rate (positive: to the right, negative: to the left, NaN to be ignored)
    """
    msg = master.mav.gimbal_manager_set_pitchyaw_encode(
        target_system,
        target_component,
        flags,
        gimbal_device_id,
        pitch,
        yaw,
        pitch_rate,
        yaw_rate
    )
    master.mav.send(msg)
    #print("Gimbal manager pitch and yaw angles message sent.")
send_gimbal_manager_pitch_yaw_angles(0, 0, 0, 0, 0, 0, float("NaN"),float("NaN"))
def send_command():
    global centre_x, center_y
    while(1):
        centre_x_copy = int(center_x)
        centre_y_copy = int(center_y)
        if ((centre_x_copy == 0) and (centre_y_copy ==0)):
            diff_x = 0
            diff_y = 0
        else:
            diff_x = (centre_x_copy - (640/2))/2
            diff_y = -(centre_y_copy - (480/2))/2
        #print(diff_x,diff_y)

        send_gimbal_manager_pitch_yaw_angles(0, 0, 0, 0, float("NaN"), float("NaN"), math.radians(diff_y), math.radians(diff_x))

#########################################################
# Initialize the VideoCapture object

# gst-launch-1.0 -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false

gst_pipeline = (
    "udpsrc port=5600 ! application/x-rtp,encoding-name=H264,payload=96 ! "
    "rtph264depay ! h264parse ! queue ! avdec_h264 ! videoconvert ! appsink"
)

cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
# cap = cv2.VideoCapture('rtsp://192.168.144.25:8554/main.264')

# Load the pre-trained face detection model
#face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
model = YOLO("yolov8n.pt") #yolo-Weights/yolov8n.pt

# Define a lock for accessing the latest frame
frame_lock = threading.Lock()
latest_frame = None
center_y = 0
center_x = 0
# Function to continuously update the latest frame
def update_frame():
    global latest_frame
    while True:
        ret, frame = cap.read()
        if ret:
            with frame_lock:
                latest_frame = frame

# Function to perform face detection on the latest frame
def detect_faces():
    global latest_frame, center_x, center_y, show_frame, frame, classf, final_list 
    final_list = []
    while True:
        
        if latest_frame is not None:
            with frame_lock:
                frame = latest_frame
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # Your face detection code here
            # For example:

            results = model.track(frame,stream=True)
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    #bounding box
                    classf = int(box.cls)
                    name = model.names[classf]
                    print("classification is:", name)
                    if (name != "person" or box.conf < 0.7):
                        continue
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values

                    # height, width = img.shape[:2]

                    #Calculate the center coordinates
                    # img_center_x = width // 2
                    # img_center_y = height // 2
                    
                    #put box in cam
                    
                    final_list = (x1, y1, x2, y2)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 3)

            # faces = face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5, minSize=(30, 30))
            faces= [final_list]
            if len(faces) == 0:
                # Do something if no faces were detected
                #print("No faces detected!")
                center_x = 640/2
                center_y = 480/2
                continue
            try:
                for (x, y, w, h) in faces:
                    cv2.rectangle(frame, (x, y), (w, h), (255, 0, 0), 2)
                    center_x = (x + w) // 2
                    center_y = (y + h) // 2
                    cv2.circle(frame, (center_x, center_y), 3, (0, 255, 0), -1)
                    #print("Center of the face: ({}, {})".format(center_x, center_y))
            except:
                pass
                

# Start the threads
update_thread = threading.Thread(target=update_frame)
update_thread.start()

detect_thread = threading.Thread(target=detect_faces)
detect_thread.start()

control_thread = threading.Thread(target=send_command)
control_thread.start()


# Main thread to display the latest frame
while True:
    with frame_lock:
        if latest_frame is not None:
            cv2.imshow('Frame', latest_frame)
            # sprint(center_x,center_y)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the VideoCapture object
cap.release()
cv2.destroyAllWindows()
