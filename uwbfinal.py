#!/usr/bin/env python3
import rospy

import cmath
import numpy as np
from serial import Serial
import time
import matplotlib.pyplot as plt
import json
import re
import numpy as np
from geometry_msgs.msg import Vector3

# USB PORT
portName='/dev/ttyUSB1'
# Create Serial object:
ser = Serial(portName) # Also opens the port during object creation
ser.close() # To set parameters

# Change settings for Arduino default Serial.begin:
ser.baudrate=115200
ser.port=portName
ser.bytesize=8
ser.parity='N'
ser.stopbits=1
ser.timeout=0.1
# Open port
ser.open()
ser.reset_input_buffer()
L = 31.6
B = 20.6
c1 = 2.5
# L = 2.24
# B = 1.12
# c1 = 0.66

def P123(r1,r2,r3):
    f = cmath.sqrt(-B**4*L**2 - B**2*L**4 + 2*B**2*L**2*r1**2 + 2*B**2*L**2*r3**2 - B**2*r2**4 + 2*B**2*r2**2*r3**2 - B**2*r3**4 - L**2*r1**4 + 2*L**2*r1**2*r2**2 - L**2*r2**4)
    if f.real == 0:
        return None
    if f.real != 0: 
        s = ((L**2 + r2**2 - r3**2)/(2*L), (B**2 + r1**2 - r2**2)/(2*B), c1 - np.sqrt(-B**4*L**2 - B**2*L**4 + 2*B**2*L**2*r1**2 + 2*B**2*L**2*r3**2 - B**2*r2**4 + 2*B**2*r2**2*r3**2 - B**2*r3**4 - L**2*r1**4 + 2*L**2*r1**2*r2**2 - L**2*r2**4)/(2*B*L))
        return [s[0],s[1]]
    
def P124(r1,r2,r4):
    f = cmath.sqrt(-B**4*L**2 - B**2*L**4 + 2*B**2*L**2*r2**2 + 2*B**2*L**2*r4**2 - B**2*r1**4 + 2*B**2*r1**2*r4**2 - B**2*r4**4 - L**2*r1**4 + 2*L**2*r1**2*r2**2 - L**2*r2**4)
    if f.real == 0:
        return None
    if f.real != 0: 
        s = ((L**2 + r1**2 - r4**2)/(2*L), (B**2 + r1**2 - r2**2)/(2*B), c1 - np.sqrt(-B**4*L**2 - B**2*L**4 + 2*B**2*L**2*r2**2 + 2*B**2*L**2*r4**2 - B**2*r1**4 + 2*B**2*r1**2*r4**2 - B**2*r4**4 - L**2*r1**4 + 2*L**2*r1**2*r2**2 - L**2*r2**4)/(2*B*L))
        return [s[0],s[1]]
    
def P143(r1,r4,r3):
    f = cmath.sqrt(-B**4*L**2 - B**2*L**4 + 2*B**2*L**2*r1**2 + 2*B**2*L**2*r3**2 - B**2*r1**4 + 2*B**2*r1**2*r4**2 - B**2*r4**4 - L**2*r3**4 + 2*L**2*r3**2*r4**2 - L**2*r4**4)
    if f.real == 0:
        return None
    if f.real != 0:
        s = ((L**2 + r1**2 - r4**2)/(2*L), -(-B**2 + r3**2 - r4**2)/(2*B), c1 - np.sqrt(-B**4*L**2 - B**2*L**4 + 2*B**2*L**2*r1**2 + 2*B**2*L**2*r3**2 - B**2*r1**4 + 2*B**2*r1**2*r4**2 - B**2*r4**4 - L**2*r3**4 + 2*L**2*r3**2*r4**2 - L**2*r4**4)/(2*B*L))
        return [s[0],s[1]]
    
def P243(r2,r4,r3):
    f = cmath.sqrt(-B**4*L**2 - B**2*L**4 + 2*B**2*L**2*r2**2 + 2*B**2*L**2*r4**2 - B**2*r2**4 + 2*B**2*r2**2*r3**2 - B**2*r3**4 - L**2*r3**4 + 2*L**2*r3**2*r4**2 - L**2*r4**4)
    if f.real == 0:
        return None
    if f.real != 0: 
        s = ((L**2 + r2**2 - r3**2)/(2*L), -(-B**2 + r3**2 - r4**2)/(2*B), c1 - np.sqrt(-B**4*L**2 - B**2*L**4 + 2*B**2*L**2*r2**2 + 2*B**2*L**2*r4**2 - B**2*r2**4 + 2*B**2*r2**2*r3**2 - B**2*r3**4 - L**2*r3**4 + 2*L**2*r3**2*r4**2 - L**2*r4**4)/(2*B*L))
        return [s[0],s[1]]


while True:
    rospy.init_node('uwb',anonymous=True)
    rate = rospy.Rate(5)
    uwbpub = rospy.Publisher('/uwb/loc', Vector3, queue_size=10)
    while not rospy.is_shutdown():
        try:
            
            N=ser.in_waiting        
            data=ser.readline() 
            data=data.decode('utf-8')
            f = json.loads(data)
            dis = {}
            for i in f['links']:
                if int(i["A"])==1786:
                    dis["r1"]=float(i["R"])
                if int(i["A"])==1787:
                    dis["r2"]=float(i["R"])
                if int(i["A"])==1788:
                    dis["r3"]=float(i["R"])
                if int(i["A"])==1789:
                    dis["r4"]=float(i["R"])
            # print(dis)
            if len(dis)<3:
                print(f"No Fix.  {len(dis)} anchors detected")
            elif len(dis)==3:
                print("3d fix")
                try:
                    if set(dis.keys()) == {'r1','r2','r3'}:
                        fs = P123(dis['r1'],dis['r2'],dis['r3'])
                    if set(dis.keys()) == {'r1','r2','r4'}:
                        fs = P124(dis['r1'],dis['r2'],dis['r4'])
                    if set(dis.keys()) == {'r1','r4','r3'}:
                        fs = P143(dis['r1'],dis['r4'],dis['r3'])
                    if set(dis.keys()) == {'r2','r4','r3'}:
                        fs = P243(dis['r2'],dis['r4'],dis['r3'])
                    print(fs)
                    uwbmsg = Vector3()
                    uwbmsg.x = fs[0]
                    uwbmsg.y = fs[1]
                    uwbpub.publish(uwbmsg)
                    
                except RuntimeError:
                    print("value error")
            elif len(dis)==4:
                try:
                    print("4d Fix")
                    
                    fs1 = P123(dis['r1'],dis['r2'],dis['r3'])
                    fs2 = P124(dis['r1'],dis['r2'],dis['r4'])
                    fs3 = P143(dis['r1'],dis['r4'],dis['r3'])
                    fs4 = P243(dis['r2'],dis['r4'],dis['r3'])
                    fsx = np.mean([fs1[0],fs2[0],fs3[0],fs4[0]])
                    fsy = np.mean([fs1[1],fs2[1],fs3[1],fs4[1]])
                    fs = [fsx,fsy]
                    print(fs)
                
                    uwbmsg = Vector3()
                    uwbmsg.x = fs[0]
                    uwbmsg.y = fs[1]
                    uwbpub.publish(uwbmsg)
                   
                except RuntimeError:
                    print("value error")
            # print(dis)
        except KeyboardInterrupt:
            break
        except Exception as e:
            pass
ser.close()



















