import cv2
import numpy
import math
import threading
from networktables import NetworkTables
from enum import Enum
import time
from flask import Flask, flash, redirect, render_template, request, session, abort, send_file
from gevent.pywsgi import WSGIServer
from io import BytesIO
from PIL import Image
import ruamel.yaml
import os
import logging

# ----------Settings----------

yaml_settings = []
yaml = ruamel.yaml.YAML()
with open(r'settings.yaml') as file:
    yaml_settings = yaml.load(file)
#Threshold
minH = yaml_settings['minH']
maxH = yaml_settings['maxH']
minS = yaml_settings['minS']
maxS = yaml_settings['maxS']
minV = yaml_settings['minV']
maxV = yaml_settings['maxV']

#Contours
minArea = yaml_settings['minArea']
minPerimeter = yaml_settings['minPerimeter']
minVertices = yaml_settings['minVertices']
makeConvex = yaml_settings['makeConvex']
targetSides = yaml_settings['targetSides']
targetPointCoords = numpy.array(yaml_settings['targetPointCoords'], dtype=numpy.float32)
firstPointAngle = math.radians(yaml_settings['firstPointAngle'])
firstPointX = math.cos(firstPointAngle)
firstPointY = math.sin(firstPointAngle)
ransac = yaml_settings['ransac']
minSideLength = yaml_settings['minSideLength']

#Camera
imgWidth = yaml_settings['imgWidth'] #width (suggested=640)
imgHeight = yaml_settings['imgHeight'] #height (suggested=480)
framerate = yaml_settings['framerate'] #framerate (suggested=90)
exposure = yaml_settings['exposure'] #exposure (suggested=20)
cameraK = numpy.array(yaml_settings['cameraK'])
cameraD = numpy.array(yaml_settings['cameraD'])
fisheye = yaml_settings['fisheye']

#Misc
roborioAddress = yaml_settings['roborioAddress']
networkTable = yaml_settings['networkTable']
webserverPort = yaml_settings['webserverPort']
x11Stream = yaml_settings['x11Stream']
webStream = yaml_settings['webStream']
streamFramesToSkip = yaml_settings['streamFramesToSkip']
connectToRobot = yaml_settings['connectToRobot']


# ----------Start Web Server----------
print("Starting webserver...")
flaskapp = Flask(__name__)
def web():
    http_server = WSGIServer(('0.0.0.0', webserverPort), flaskapp, log=None)
    print("Webserver started")
    http_server.serve_forever()

webThread = threading.Thread(target = web)
webThread.start()

@flaskapp.route("/")
def homepage():
    return render_template('index.html',**globals())

@flaskapp.route("/settings")
def settingspage():
    return render_template('settings.html',**globals())

@flaskapp.route("/updatesetting")
def updateSetting():
    settingToUpdate = request.args.get('s').strip()
    value = request.args.get('v')
    saveVal = None
    try:
        saveVal = float(value)
    except ValueError:
        if value == "true":
            saveVal = True
        elif value == "false":
            saveVal = False
        else:
            saveVal = value

    print("updating ", settingToUpdate, " to ", saveVal)
    globals()[settingToUpdate] = saveVal
    yaml_settings[settingToUpdate] = saveVal
    return str(settingToUpdate + " = " + str(saveVal))

@flaskapp.route("/savesettings")
def saveSettings():
    global yaml_settings, yaml
    with open(r'settings.yaml', 'w') as file:
        yaml.dump(yaml_settings, file)
    return ""

@flaskapp.route("/getstat")
def getStat():
    stat = request.args.get('s').strip()
    return str(globals()[stat])

webCamPreview = None
webCamPreview2 = None
@flaskapp.route("/preview.jpg")
def previewpage():
    if webCamPreview != None:
        img_io = BytesIO()
        webCamPreview.save(img_io, 'JPEG', quality=90)
        img_io.seek(0)
        return send_file(img_io, mimetype='image/jpeg')
    return "No Preview"

@flaskapp.route("/preview.png")
def previewpagepng():
    if webCamPreview != None:
        img_io = BytesIO()
        webCamPreview.save(img_io, 'PNG')
        img_io.seek(0)
        return send_file(img_io, mimetype='image/png')
    return "No Preview"

@flaskapp.route("/preview2.jpg")
def previewpage2():
    if webCamPreview2 != None:
        img_io = BytesIO()
        webCamPreview2.save(img_io, 'JPEG', quality=70)
        img_io.seek(0)
        return send_file(img_io, mimetype='image/jpeg')
    return "No Preview"

@flaskapp.route("/primeTarget")
def primeTargetPage():
    global primeTarget
    if primeTarget == None:
        return "-,-,-,-,-,-"
    return f"{primeTarget[0][0][0]:.2f},{primeTarget[0][1][0]:.2f},{primeTarget[0][2][0]:.2f},{(primeTarget[1][0][0]*57.2958):.2f},{(primeTarget[1][1][0]*57.2958):.2f},{(primeTarget[1][2][0]*57.2958):.2f}"
# ----------Connect to Robot----------
if (connectToRobot):
    print("Looking for robot...")
    cond = threading.Condition()
    notified = [False]

    def connectionListener(connected, info):
        print(info, '; Connected=%s' % connected)
        with cond:
            notified[0] = True
            cond.notify()

    NetworkTables.initialize(server=roborioAddress)
    NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)
    table = NetworkTables.getTable(networkTable)
    inst = NetworkTables.getDefault()


# ----------Start Camera----------
print("Setting up camera")
os.system('./setupCamera.sh')
cap = cv2.VideoCapture(0)
cap.set(3,imgWidth)
cap.set(4,imgHeight)
cap.set(5,framerate)
cap.set(15,exposure)
print("Camera resolution: ", cap.get(3), "x", cap.get(4));
print("Framerate: ", cap.get(5))
print("Exposure: ", cap.get(15))
print("Running...")

# ----------Main Loop----------
sidesOfPrimeTarget = 0
pipelineTime = 0
pipelineTimeCounter = 0
targets = []
primeTarget = None
def run():
    global webCamPreview, webCamPreview2, targets, pipelineTimeCounter, pipelineTime, primeTarget
    i = 0
    while (True):
        _, frame = cap.read()
        pipelineStart = time.time()
        thresholdImage = hsvThreshold(frame)
        targets = findTargets(thresholdImage)
        primeTarget = calculatePrimeTarget(targets)
        isTarget = primeTarget != None
        if connectToRobot:
            table.putBoolean("isTarget", isTarget)
            if (isTarget):
                table.putNumberArray("tvec", primeTarget[0])
                table.putNumberArray("rvec", primeTarget[1])
            inst.flush()
        i+=1
        if i%streamFramesToSkip == 0:
            if x11Stream or webStream:
                cv2.drawContours(frame, targets, -1, (0, 0, 255), 2)
                if primeTarget != None:
                    cv2.circle(frame, (primeTarget[3][0], primeTarget[3][1]), 4, (255,0,0), 2)
                    for i in range(0, len(primeTarget[2])):
                        cv2.putText(frame,str(i),(primeTarget[2][i][0][0], primeTarget[2][i][0][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,150,150)),
            if x11Stream:
                cv2.imshow('Input',frame)
                cv2.imshow('HSV Threshold',thresholdImage)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            if webStream:
                webCamPreview = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

                webCamPreview2 = Image.fromarray(cv2.cvtColor(thresholdImage, cv2.COLOR_GRAY2RGB))
        pipelineTimeCounter += time.time() - pipelineStart
        if i%framerate == 0:
            pipelineTime = pipelineTimeCounter/framerate
            pipelineTimeCounter = 0


# ----------Helper Functions----------
def findTargets(input):
    contours = convexHull(filterContours(findContours(input)))
    return contours

def hsvThreshold(input):
    out = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)
    return cv2.inRange(out, (minH, minS, minV),  (maxH, maxS, maxV))

def findContours(input, internal=False):
    '''
    Set internal to true if you want to also get internal contours
    '''
    if(internal):
        mode = cv2.RETR_LIST
    else:
        mode = cv2.RETR_EXTERNAL
    method = cv2.CHAIN_APPROX_SIMPLE
    contours,_ =cv2.findContours(input, mode=mode, method=method)
    return contours

def convexHull(input):
    if not makeConvex:
        return input
    output = []
    for contour in input:
        #clockwise=true means counter-clockwise (It's weird, I know)
        output.append(cv2.convexHull(contour, clockwise=True))
    return output

def filterContours(input):
    output = []
    for contour in input:
        area = cv2.contourArea(contour)
        if (area < minArea):
            continue
        if (cv2.arcLength(contour, True) < minPerimeter):
            continue
        if (len(contour) < minVertices):
            continue
        output.append(contour)
    return output

def calculatePrimeTarget(targets):
    global targetSides, fisheye, minSideLength, sidesOfPrimeTarget

    if len(targets) == 0:
        return None

    #step 1 find the largest contour
    largest = 0
    largestVal = 0
    for i in range(len(targets)):
        size = cv2.contourArea(targets[i])
        epsilon = minSideLength * cv2.arcLength(targets[i], True)
        approx = cv2.approxPolyDP(targets[i], epsilon, True)
        if size > largestVal and (len(approx) == targetSides or largest == 0):
            largestVal = size
            largest = i
            sidesOfPrimeTarget = len(approx)

    #step 2 transform this contour into rectilinear
    if fisheye:
        target_undistorted = undistortPoints(numpy.array(targets[largest], numpy.float32))
    else:
        target_undistorted = numpy.array(targets[largest], numpy.float32)
    approx2 = cv2.approxPolyDP(target_undistorted, epsilon, True)
    M = cv2.moments(target_undistorted)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    point_angles = []
    dst = numpy.squeeze(approx2)
    dstLen = len(dst)
    if dstLen != targetSides:
        return None

    #So we have these points. The goal now is to find the proper starting point. We must project
    #the points onto the line of angle theta and determine their distances from the CoM.
    #Convert to polar with origin at CoM and subtract line angle
    startPoint = 0
    maxDist = 0
    for i in range(0,dstLen):
        pt_x = dst[i][0]-cX
        pt_y = cY-dst[i][1]
        scalar = firstPointX * pt_x + firstPointY * pt_y
        if scalar > maxDist:
            maxDist = scalar
            startPoint = i

    output_array = []
    for i in range(startPoint,dstLen):
        output_array.append([dst[i]])

    for i in range(0,startPoint):
        output_array.append([dst[i]])
    target_array = numpy.array(output_array, numpy.float32)
    (success, rvec, tvec) = findPose(target_array, targetPointCoords)[0:3]

    if success:
        return (tvec,rvec, output_array, (cX,cY))
    else:
        return None


def undistortPoints(points_distorted):
    global cameraK, cameraD
    cameraMatrix=cameraK
    distCoeffs=cameraD
    Knew = cameraMatrix.copy()
    Knew[(0,1),(0,1)] = Knew[(0,1),(0,1)]
    return cv2.fisheye.undistortPoints(points_distorted, K=cameraMatrix, D=distCoeffs, P=Knew)

def findPose(imagepts, objectpts):
    global cameraK, ransac, fisheye, cameraD
    cameraMatrix = cameraK
    if fisheye:
        distCoeff = numpy.zeros((4, 1))
    else:
        distCoeff = cameraD
    if ransac:
        return cv2.solvePnPRansac(objectpts, imagepts, cameraMatrix, distCoeff)
    return cv2.solvePnP(objectpts, imagepts, cameraMatrix, distCoeff)

run()
