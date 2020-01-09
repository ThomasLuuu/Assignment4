import numpy as np
import cv2
import WebcamMultithreading
from pushbullet import Pushbullet
import time
import RPi.GPIO as GPIO
import Adafruit_DHT
from multiprocessing import Process

def motionDetection():
    # calling WebcamMultithreading class to initialize webcam
    # and read frames
    cam = WebcamMultithreading.WebcamMultithreading().start()

    # OpenCV's adaptive background substractor with configurable values
    BSubstractor = cv2.createBackgroundSubtractorMOG2(history=100, varThreshold=35, detectShadows=False)

    # loop for processing
    while True:
        motion_text = "None" # set up a string variable to later display on cam feed
        _, frame = cam.read()  # return a single frame from webcam
        substracted = BSubstractor.apply(frame) # apply the Background Subtractor method to retrieved frames

        # apply thresholding to grayscaled frames
        # this will assign a white pixel to any pixel meeting the defined threshold,
        # otherwise it's assigned a black pixel
        # main purpose is for image segmentation to separate objects/subjects
        thresh = cv2.threshold(substracted, 25, 255, cv2.THRESH_BINARY)[1]

        # dilate above thresholded frames to fill holes,
        # creating a more solid shape for finding contours/edges
        thresh = cv2.dilate(thresh, None, iterations=2)

        # draws contours aka edge detection around detected motion
        (_, edges, _) = cv2.findContours(thresh.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    #     edges = imutils.grab_contours(edges)

        # initialize a container to put all bounding boxes
        # later to be grouped in
        rects = []
        global motion_counter

        for e in edges:
            # minimum size to draw a bounding box, otherwise ignore
            if cv2.contourArea(e) < 1000:
                continue
            #create a bounding box
            (x, y, w, h) = cv2.boundingRect(e)
            # duplicate to container in case same box will
            # be grouped as well
            rects.append([x, y, w, h])
            rects.append([x, y, w, h])
            motion_text = "Detected"
        # motion counter algorithm
        if motion_text == "Detected":
            # increment by 1 if detected
            motion_counter += 1
            # x amount of consecutive detected motion
            # before sending an alert
            if motion_counter >= 8:
                push = pb.push_note("Motion Alert", "Motion detected!")
                motion_counter = 0
        # if there's no motion, reset counter to 0
        else:
            motion_counter = 0

        # function to actually draw "bounding boxes"
        def draw(rects,color):
            for r in rects:
              p1 = (r[0], r[1])
              p2 = (r[0]+r[2], r[1]+r[3])
              cv2.rectangle(frame, p1,p2, color,2)

        # call groupRectangles method to group overlapping bounding boxes
        rects,weights = cv2.groupRectangles(rects, 1, 1.5)

        # draw grouped bounding boxes from container in green color
        draw(rects, (0, 255, 0))

        #draw text to indicate motion detection status
        cv2.putText(frame, "Motion: {}".format(motion_text), (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(frame, "Motion counter: {}".format(motion_counter), (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # dispay a normal feed and motion feed
        cv2.imshow("Normal Feed", frame)
#         cv2.imshow("Motion Feed", thresh)

        # pressing Esc to stop
        key = cv2.waitKey(1)
        if key == 27:
            break
    cam.stop()
    cv2.destroyAllWindows()
    exit(1)

# function to check when sensor's pin goes HIGH
def callback(channel):
    if GPIO.input(channel):
        print("Sound Detected!")
        push = pb.push_note("Noise Alert", "Sound detected!")

# function to retrieve temperature readings
def readTemp():
    while True:
        humidity, temperature = Adafruit_DHT.read_retry(11, 17)
        print("Temp: {0} C  Humidity: {1} %".format(temperature, humidity))
        try:
            if temperature > 28:
                print("Too hot")
                push = pb.push_note("Temperature Alert", "Abnormal reading of {0} C detected!".format(temperature))
        except TypeError:
            pass

if __name__ == '__main__':
    # initialize a global motion counter variable to
    # access from inside the function
    global motion_counter
    motion_counter = 0
    # setting up GPIO for sound sensor
    channel = 4
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(channel, GPIO.IN)
    GPIO.add_event_detect(channel, GPIO.RISING, bouncetime=300)  # let us know when the pin goes HIGH
    GPIO.add_event_callback(channel, callback)  # assign function to GPIO PIN, Run function on change

    # Pushbullet API key for sending push notifications
    pb = Pushbullet("o.e00CRKrMXGlOQ0VvtEUxId4yeGXTIgV1")

    # reading is I/O heavy so we spawn a separate process
    # for each function to run in parallel which
    # alleviated webcam being laggy problem further
    process_motion = Process(target=motionDetection)
    process_motion.daemon = True
    process_motion.start()

    process_temp = Process(target=readTemp)
    process_temp.daemon = True
    process_temp.start()