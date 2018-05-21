#!/usr/bin/env python3
"""
Example demonstrating how to communicate with Microsoft Robotic Developer
Studio 4 via the Lokarria http interface. 

Author: Erik Billing (billing@cs.umu.se)

Updated by Ola Ringdahl 2014-09-11
Updated by Lennart Jern 2016-09-06 (converted to Python 3)
Updated by Filip Allberg and Daniel Harr 2017-08-30 (actually converted to Python 3)
Modifyed by Martin Willma(id14mwn) and Amine Balta(id14aba) 2017-09-27.
"""

MRDS_URL = 'localhost:50000'

import http.client, json, time, sys
from math import *

HEADERS = {"Content-type": "application/json", "Accept": "text/json"}

class UnexpectedResponse(Exception): pass

def postSpeed(angularSpeed,linearSpeed):
    """Sends a speed command to the MRDS server"""
    mrds = http.client.HTTPConnection(MRDS_URL)
    params = json.dumps({'TargetAngularSpeed':angularSpeed,'TargetLinearSpeed':linearSpeed})
    mrds.request('POST','/lokarria/differentialdrive',params,HEADERS)
    response = mrds.getresponse()
    status = response.status
    #response.close()
    if status == 204:
        return response
    else:
        raise UnexpectedResponse(response)

def getLaser():
    """Requests the current laser scan from the MRDS server and parses it into a dict"""
    mrds = http.client.HTTPConnection(MRDS_URL)
    mrds.request('GET','/lokarria/laser/echoes')
    response = mrds.getresponse()
    if (response.status == 200):
        laserData = response.read()
        response.close()
        return json.loads(laserData.decode())
    else:
        return response
    
def getLaserAngles():
    """Requests the current laser properties from the MRDS server and parses it into a dict"""
    mrds = http.client.HTTPConnection(MRDS_URL)
    mrds.request('GET','/lokarria/laser/properties')
    response = mrds.getresponse()
    if (response.status == 200):
        laserData = response.read()
        response.close()
        properties = json.loads(laserData.decode())
        beamCount = int((properties['EndAngle']-properties['StartAngle'])/properties['AngleIncrement'])
        a = properties['StartAngle']#+properties['AngleIncrement']
        angles = []
        while a <= properties['EndAngle']:
            angles.append(a)
            a+=pi/180 #properties['AngleIncrement']
        #angles.append(properties['EndAngle']-properties['AngleIncrement']/2)
        return angles
    else:
        raise UnexpectedResponse(response)

def getPose():
    """Reads the current position and orientation from the MRDS"""
    mrds = http.client.HTTPConnection(MRDS_URL)
    mrds.request('GET','/lokarria/localization')
    response = mrds.getresponse()
    if (response.status == 200):
        poseData = response.read()
        response.close()
        return json.loads(poseData.decode())
    else:
        return UnexpectedResponse(response)

def heading(q):
    return rotate(q,{'X':1.0,'Y':0.0,"Z":0.0})

def rotate(q,v):
    return vector(qmult(qmult(q,quaternion(v)),conjugate(q)))

def quaternion(v):
    q=v.copy()
    q['W']=0.0;
    return q

def vector(q):
    v={}
    v["X"]=q["X"]
    v["Y"]=q["Y"]
    v["Z"]=q["Z"]
    return v

def conjugate(q):
    qc=q.copy()
    qc["X"]=-q["X"]
    qc["Y"]=-q["Y"]
    qc["Z"]=-q["Z"]
    return qc

def qmult(q1,q2):
    q={}
    q["W"]=q1["W"]*q2["W"]-q1["X"]*q2["X"]-q1["Y"]*q2["Y"]-q1["Z"]*q2["Z"]
    q["X"]=q1["W"]*q2["X"]+q1["X"]*q2["W"]+q1["Y"]*q2["Z"]-q1["Z"]*q2["Y"]
    q["Y"]=q1["W"]*q2["Y"]-q1["X"]*q2["Z"]+q1["Y"]*q2["W"]+q1["Z"]*q2["X"]
    q["Z"]=q1["W"]*q2["Z"]+q1["X"]*q2["Y"]-q1["Y"]*q2["X"]+q1["Z"]*q2["W"]
    return q
    
def getHeading():
    """Returns the XY Orientation as a heading unit vector"""
    return heading(getPose()['Pose']['Orientation'])

def getPosition():
    """Returns the XYZ position"""
    return getPose()['Pose']['Position']

def getDistance(x, y):
    """Returns the distance between currentPosition and nextpos"""
    return sqrt((x ** 2) + (y ** 2))

def getBearingAngle(currentPosition, PositionP):
    """Returns the bearing angle between currentPosition and nextpos"""
    x1, y1 = currentPosition
    x2, y2 = PositionP
    bearingAngle = atan2(y2-y1, x2-x1)
    return bearingAngle

def getHeadingAngle(x, y):
    """Returns the heading angle"""
    angleX = atan2(y, x)
    return(angleX)

def errorAngle(angleHeading, angleBearing):
    """Returns the difference between bearing angle and heading angle"""
    errorA = angleBearing - angleHeading
    
    """Make sure that the robot does not circulate when angle is
    bigger/smaller than +-pi"""
    if (errorA < -pi):
        errorA = errorA + 2*pi 
        return(errorA)
    elif (errorA > pi):
        errorA = errorA - 2*pi
        return(errorA)
    else:
        return(errorA)    
    
def createPath():
    """Puts the positions of the path in a reversed list and returns the list"""
    vecArray = []
    
    with open('D:\exam2017.json') as path_file:
        data = json.load(path_file)
        
        for i in range (len(data)):
            vecArray.append(data[i]['Pose']['Position'])
        vecArray.reverse()
        return vecArray


def carrotPoint(path, pose, lookAhead):
    """Finds the Carrotpoint and returns it"""
    
    if path:
    
        for i in range (len(path)):
            nextPos = path[len(path)-1]
    
            
            distX = nextPos['X'] - pose['X']
            distY = nextPos['Y'] - pose['Y']
            
            dist = getDistance(distX, distY)
            
            """If the carrotPoint is smaller than the distance to the next
            point the list is poped and new carrotpoint is searched"""
            if dist < lookAhead:
                path.pop()
            else:
                return nextPos
        
if __name__ == '__main__':
    print('Sending commands to MRDS server', MRDS_URL)
    
    
    """"""
    path = createPath()
    """Fixed stats for finished all paths"""
    speed = 0.7
    aSpeed = 1.3
    lookAhead = 1
    """Start the timer"""
    startTime = time.time()
    
    try:
        
        while path:
            pose = getPosition()
            head = getHeading()
            nextPos = carrotPoint(path, pose, lookAhead)
            
            """Checks if the robots have a next position, new carrotpoint,
            else it stops until the robot finds one"""
            if(nextPos):
                
                """Calculate the headingAngle and bearingAngle"""
                dist = getDistance((nextPos['X'] - pose['X']),
                                   (nextPos['Y'] - pose['Y']))
                headingAngle = getHeadingAngle(head['X'], head['Y'])
                bearingAngle = getBearingAngle((pose['X'], pose['Y']),
                                               (nextPos['X'],nextPos['Y']))
                angleError = errorAngle(headingAngle, bearingAngle)
                response = postSpeed(angleError*aSpeed, speed)
                time.sleep(0.1)
                
            response = postSpeed(0, 0)
            endTime = time.time()
        """Calculates the runtime by take endtime - startime"""
        runTime = endTime - startTime
        print ('Runtime was:', runTime)
               
        response = postSpeed(0,0)
 
    except UnexpectedResponse as ex:
        print ('Error')