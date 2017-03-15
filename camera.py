# -*- coding: utf-8 -*-
# Tested with Python 2.7.9 on raspberry pi

from picamera.array import PiRGBArray
import time
import cv2
import datetime

import numpy as np
from imutils.video import FPS

#########################################
# Multi-threaded camera streaming class

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import threading
import cv2

class PiVideoStream:
    def __init__(self, resolution=(320, 240), framerate=32, hflip=False, vflip=False):
        # initialize the camera and stream
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.camera.hflip = hflip
        self.camera.vflip = vflip
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture,
            format="bgr", use_video_port=True)

        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.stopped = False
        self.frameEvent = threading.Event()
        self.dirty = False
        self.res = resolution

    def start(self):
        # start the thread to read frames from the video stream
        t = Thread(target=self.update, args=())
        t.daemon = True # why do we need a stop method then? daeomons quit automatically..?
        t.start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        for f in self.stream:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = f.array
            self.rawCapture.truncate(0)
            self.frameEvent.set()
            self.dirty = True

            # if the thread indicator variable is set, stop the thread
            # and resource camera resources
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return

    def read(self):
        # return the frame most recently read
        self.frameEvent.clear()
        self.dirty = False
        return self.frame

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

    def waitForFrame(self):
        self.frameEvent.wait()
##        while not self.dirty:
##            time.sleep(0.01)

    def getResolution(self):
        return self.res

######################################################################
# Servo control

import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()

# Set frequency to 60hz, good for servos.
# The min/max limits below depend on this
pwm.set_pwm_freq(60)

TILT_CHANNEL = 0
TILT_MIN = 175# 180 # Min pulse length out of 4096
TILT_MAX = 250 # 350 # Max pulse length out of 4096
TILT_MID = (TILT_MIN+TILT_MAX)/2

PAN_CHANNEL = 1
PAN_MIN = 385 #300 #350 # 300 # Min pulse length out of 4096
PAN_MAX = 535 # 470 # 550 # Max pulse length out of 4096
PAN_MID = (PAN_MIN+PAN_MAX)/2

# jiggle = False

def set_tilt(x,force=False):
    #assert TILT_MIN <= x <= TILT_MAX
    if not force:
        x = int(np.clip(x,TILT_MIN,TILT_MAX))
    pwm.set_pwm(TILT_CHANNEL,0,x)
    
def set_pan(x,force=False):
    #assert PAN_MIN <= x <= PAN_MAX
    if not force:
        x = int(np.clip(x,PAN_MIN,PAN_MAX))
    pwm.set_pwm(PAN_CHANNEL,1,x)


######################################################################
# Laser control

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(7, GPIO.OUT) ## Setup GPIO Pin 7 to OUT
def laser_on():
    GPIO.output(7, True) ## Turn on GPIO pin 7
def laser_off():
    GPIO.output(7, False) ## Turn off GPIO pin 7


######################################################################
# Servo moving program, helps with checking bounds

def servo_manual(vs):
    laser_on()
    pan_value = PAN_MID
    tilt_value = TILT_MID

    cv2.namedWindow('Image')
    while True:
        vs.waitForFrame()
        imageColor = np.array(vs.read(),dtype=np.uint8)
        cv2.imshow("Image", imageColor)
        key = cv2.waitKey(20) & 0xFF
        if key == ord("a"):
            pan_value += 10
        elif key == ord("d"):
            pan_value -= 10
        elif key == ord("w"):
            tilt_value += 10
        elif key == ord("s"):
            tilt_value -= 10
        set_pan(pan_value, force=True)
        set_tilt(tilt_value, force=True)
        
        sys.stdout.write("Pan: {0} Tilt: {1}".format(pan_value,tilt_value)+'\r')
        sys.stdout.flush()

######################################################################
# Calibration program

# Create calibration map from coordinate -> servo positions
def calibrate(vs, new_calibration=False):

    width,height = vs.getResolution()
    print "w" + str(width)
    print "h" + str(height)
        
    if new_calibration:
        print "Calibrating"
        cv2.namedWindow('Image')
        cv2.namedWindow('Delta')
        
        ker = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))

        x_points = []
        y_points = []
        pan_values = []
        tilt_values = []
        parity = 1
        for pan in [int(x) for x in np.linspace(PAN_MIN, PAN_MAX, num=8)]:
            parity *= -1
            for tilt in [int(x) for x in np.linspace(TILT_MIN, TILT_MAX, num=6)[::parity]]:

                
                # move the laser
##                set_pan(PAN_MID)
##                set_tilt(TILT_MID)
##                cv2.waitKey(1000)
                set_pan(pan)
                set_tilt(tilt)
                cv2.waitKey(1000)

                # get frame with laser off
                # (the videostream gives us the most recent frame)
                laser_off()
                for i in range(3):
                    vs.waitForFrame()
                    vs.read()
                prevColor = np.array(vs.read(),dtype=np.uint8)
                prevGray = cv2.cvtColor(prevColor, cv2.COLOR_BGR2GRAY) # convert to greyscale
                
                # now get frame with laser on
                laser_on()
                for i in range(3):
                    vs.waitForFrame()
                    vs.read()
                imageColor = np.array(vs.read(),dtype=np.uint8)
                imageGray = cv2.cvtColor(imageColor, cv2.COLOR_BGR2GRAY) # convert to greyscale

                # we don't care about pixels that haven't changed
                delta = (imageGray - prevGray)*(imageGray>prevGray)

                # erode 
                cv2.erode(delta, ker, delta)

                _, max_val, _, max_loc = cv2.minMaxLoc(delta)
                if max_val > 20:
                    cv2.circle(imageColor, max_loc, 10, (255,0,0), 2)

                    # add pair image coord -> servo coord
                    x_points.append(float(max_loc[0]))
                    y_points.append(float(max_loc[1]))
                    pan_values.append(float(pan))
                    tilt_values.append(float(tilt))
                    
                cv2.imshow("Image", imageColor)
                cv2.imshow("Delta", delta)
                    
                cv2.waitKey(30)

        np.save('calib/x_points.npy',x_points)
        np.save('calib/y_points.npy',y_points)
        np.save('calib/pan_values.npy',pan_values)
        np.save('calib/tilt_values.npy',tilt_values)
        
    else:
        print "Loading calibration data from file"
        x_points = np.load('calib/x_points.npy')
        y_points = np.load('calib/y_points.npy')
        pan_values = np.load('calib/pan_values.npy')
        tilt_values = np.load('calib/tilt_values.npy')
    
    print "Interpolating..."
    
    from scipy.interpolate import griddata
    grid_x, grid_y = np.mgrid[0:width, 0:height]
    points = np.array(zip(x_points,y_points))
    pan_values = np.array(pan_values)
    tilt_values = np.array(tilt_values)
    
    grid_pan = griddata(points, pan_values, (grid_x, grid_y), method='linear')
    grid_tilt = griddata(points, tilt_values, (grid_x, grid_y), method='linear')
    
    print "Showing calibration plots (exit window to continue)..."
    if show_local:
        import matplotlib.pyplot as plt
        plt.subplot(224)
        plt.imshow(grid_pan.T, extent=(0,width,0,height), origin='lower')
        plt.title('pan')
        plt.subplot(222)
        plt.imshow(grid_tilt.T, extent=(0,width,0,height), origin='lower')
        plt.title('tilt')

        plt.gcf().set_size_inches(6, 6)
        plt.show()

    def check_and_return(x,y,map_image):
        x = np.clip(x, 0, map_image.shape[0]-1)
        y = np.clip(y, 0, map_image.shape[1]-1)
        r = map_image[x,y]
        if np.isfinite(r):
            return r
        raise ValueError("No value here")
    def crd_to_pan(x,y):
        return check_and_return(x,y,grid_pan)
    def crd_to_tilt(x,y):
        return check_and_return(x,y,grid_tilt)
    
    return (crd_to_pan, crd_to_tilt)

######################################################################
# Web server
from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
import cgi

PORT_NUMBER = 8081

class ImageServer:
    
    def __init__(self):
        self.image = None

    def start(self):
            
        #This class will handles any incoming request from the browser
        class myHandler(BaseHTTPRequestHandler):
            imageServer = self
            #Handler for the GET requests
            def do_GET(self):
                if self.path.endswith('.html'):
                    with open('camera_index.html') as fin:
                        self.send_response(200)
                        self.send_header('Content-type','text/html')
                        self.end_headers()
                        self.wfile.write(fin.read())
                elif self.path.endswith('.jpg'):
                    if imageServer.image != None:
                        ret,imbuf = cv2.imencode('.jpg', imageServer.image)
                        self.send_response(200)
                        self.send_header('Content-type','image/jpg')
                        self.end_headers()
                        self.wfile.write(imbuf.tostring())
                    else:
                        self.send_error(404, "Image does not exist")
                else:
                    self.send_error(404, "Does not exist")

            def do_POST(self):
                form = cgi.FieldStorage(fp=self.rfile,headers =self.headers, environ={"REQUEST_METHOD":"POST"})
                # simulate mouse click
                mouse_callback(cv2.EVENT_LBUTTONDOWN, int(form["left"].value), int(form["top"].value), None, None)
                # and mouse move (if it's just been changed toauto, won't make a difference. manual doesnt matter)
                mouse_callback(cv2.EVENT_MOUSEMOVE, int(form["left"].value), int(form["top"].value), None, None)
                
                self.send_response(200)
                self.send_header('Content-type','text/html')
                self.end_headers()

        try:
            #Create a web server and define the handler to manage the
            #incoming request
            server = HTTPServer(('', PORT_NUMBER), myHandler)
            print 'Started httpserver on port ' , PORT_NUMBER
            
            t = Thread(target=server.serve_forever, args=())
            t.daemon = True # will quit automatically
            t.start()

        except KeyboardInterrupt:
            print '^C received, shutting down the web server'
            server.socket.close()
            
    def updateImage(self, image):
        self.image = image

######################################################################
# Main tracking program

import sys
import random
import datetime
import commands
 

show_local = False # show opencv windows locally? (won't work over ssh)
if len(sys.argv) >= 2:
    show_local = True
new_calibration = False
if len(sys.argv) >= 3:
    new_calibration = True

wiggle_laser = False


# initialize the camera stream and start its thread
width = 360*2
height = 240*2
videoStream = PiVideoStream(resolution=(width,height), hflip=True, vflip=True)
videoStream.start()
time.sleep(2.0)

#servo_manual(videoStream)

fps = FPS().start()
fps_value = 0

crd_to_pan, crd_to_tilt = calibrate(videoStream, new_calibration)

prevImage = np.zeros((height,width,1), np.uint8)
image = np.zeros((height,width,1), np.uint8)
delta = np.zeros((height,width,1), np.uint8)

erodeKernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))

# state
enable = False
auto_control = False
trackPoint = (0,0)

# start up the web server
imageServer = ImageServer()
imageServer.start()

def get_cpu_temp():
    tempFile = open("/sys/class/thermal/thermal_zone0/temp")
    cpu_temp = tempFile.read()
    tempFile.close()
    return float(cpu_temp)/1000
def get_gpu_temp():
    gpu_temp = commands.getoutput("/opt/vc/bin/vcgencmd measure_temp").replace("temp=","").replace("'C","")
    return float(gpu_temp)

# mouse callback function
WINDOW_BORDER = 20
def mouse_callback(event,x,y,flags,param):
    global enable
    global auto_control
    global trackPoint
    if event == cv2.EVENT_MOUSEMOVE:
        if not auto_control:
            trackPoint = (x,y)
    elif event == cv2.EVENT_LBUTTONDOWN:
        if 0<=x<=20 and 0<=y<=20:
            enable = not enable
        elif 0<=x<=20 and 20<=y<=40:
            auto_control = not auto_control
        
# Create a black image, a window and bind the function to window
if show_local:
    cv2.namedWindow('Image')
    cv2.setMouseCallback('Image',mouse_callback)

frameCnt = 0

# capture frames from the camera
while True:
        videoStream.waitForFrame()

        # grab the raw NumPy array representing the image
        imageColor = videoStream.read()
        cv2.cvtColor(imageColor, cv2.COLOR_BGR2GRAY, image) # convert to greyscale

        if auto_control and frameCnt != 0:
            cv2.absdiff(image,prevImage,delta)
            cv2.erode(delta, erodeKernel, delta)

            nonz = np.where(delta > 10)
            if len(nonz[0]) > 0: #imsize*0.002:
                    trackPoint = (nonz[1][0],nonz[0][0]+10)

        np.copyto(prevImage,image)

        if enable:
            offset_x = 0
            offset_y = 0
            if wiggle_laser:
                offset_x = (random.random()-0.5)*16
                offset_y = (random.random()-0.5)*16
            try:
                pan_value = int(crd_to_pan(trackPoint[0],trackPoint[1]) + offset_x)
                tilt_value = int(crd_to_tilt(trackPoint[0],trackPoint[1]) + offset_y)
                set_pan(pan_value)
                set_tilt(tilt_value)
                laser_on()
            except ValueError:
                pan_value = -1
                tilt_value = -1
                laser_off()
        else:
            pan_value = -1
            tilt_value = -1
            laser_off()
            

        cv2.circle(imageColor, trackPoint, 10, (255,0,0), 2)
        
        if enable:
            cv2.circle(imageColor, (10,10), 10, (0,255,0), -1)
        else:
            cv2.circle(imageColor, (10,10), 10, (0,0,255), -1)
        if auto_control:
            cv2.circle(imageColor, (10,30), 10, (0,255,0), -1)
        else:
            cv2.circle(imageColor, (10,30), 10, (0,0,255), -1)
        
        cv2.putText(imageColor, "Enable", (20, 20), 0, 0.5, (255,255,255), 1)
        cv2.putText(imageColor, "Auto", (20, 40), 0, 0.5, (255,255,255), 1)
        
        txt = str(datetime.datetime.now())
        cv2.putText(imageColor, txt, (0, height-5), 0, 0.5, (255,255,255), 1)
        gpu_temp = int(get_gpu_temp())
        cpu_temp = int(get_cpu_temp())
        cv2.putText(imageColor, "GPU: " + str(gpu_temp) + " C", (0, height-25), 0, 0.5, (255,255,255), 1)
        cv2.putText(imageColor, "CPU: " + str(cpu_temp) + " C", (0, height-45), 0, 0.5, (255,255,255), 1)
        
        if show_local:
            cv2.imshow("Image", imageColor)
            key = cv2.waitKey(2) & 0xFF
            if key == ord("q"):
                 break
        imageServer.updateImage(imageColor) # should really copy
        

        frameCnt += 1

        if frameCnt % 10 == 0:
            fps.stop()
            fps_value = int(fps.fps())
        fps.update()
        
        sys.stdout.write("FPS: {0} Pan: {1} Tilt: {2} CPU: {3} GPU: {4}".format(fps_value,pan_value,tilt_value,cpu_temp,gpu_temp)+'\r')
        sys.stdout.flush()

if show_local:
    cv2.destroyAllWindows()
    cv2.waitKey(0)
videoStream.stop()
time.sleep(1)
