#!/usr/bin/env python2

from RFM69 import RFM69
from RFM69.RFM69registers import *
import paho.mqtt.client as mqtt
import struct
import sys
import signal
import time
import Queue

VERSION = "2.1"
NETWORKID = 100
KEY = "blanked"
FREQ = RF69_868MHZ #options are RF69_915MHZ, RF69_868MHZ, RF69_433MHZ, RF69_315MHZ
writeQ = Queue.Queue()

class Message(object):
    def __init__(self, message = None):
        self.nodeID = 0
        self.sensorID = 0
        self.uptime = 0L
        self.data = 0.0
        self.battery = 0.0
        self.s = struct.Struct('hhLff')
        self.message = message
        if message:
            self.getMessage()
    
    def setMessage(self, nodeID = None, sensorID = None, uptime = 0, data = 0.0, battery = 0.0):
        if nodeID:
            self.message = self.s.pack(nodeID, sensorID, uptime, data, battery)
        else:
            self.message = self.s.pack(self.nodeID, self.sensorID, self.uptime, self.data, self.battery)
        self.getMessage()
        
    def getMessage(self):

        try:
            self.nodeID, self.sensorID, self.uptime, self.data, self.battery = \
               self.s.unpack_from(buffer(self.message))
        except:
            print "could not extract message"

class Gateway(object):
    def __init__(self, freq, networkID, key):
        self.mqttc = mqtt.Client()
        self.mqttc.on_connect = self.mqttConnect
        self.mqttc.on_message = self.mqttMessage
        self.mqttc.connect("127.0.0.1", 1883, 60)
        self.mqttc.loop_start()
        print "mqtt init complete"
        
        self.radio = RFM69.RFM69(freq, 1, networkID, True)
        self.radio.rcCalibration()
        self.radio.encrypt(key)
        print "radio init complete"
    
    def receiveBegin(self):
        self.radio.receiveBegin()
    
    def receiveDone(self):
        return self.radio.receiveDone()
    
    def mqttConnect(self, client, userdata, flags, rc):
        self.mqttc.subscribe("home/rfm_gw/sb/#")
    
    def mqttMessage(self, client, userdata, msg):
        message = Message()
        if len(msg.topic) == 27:
            message.nodeID = int(msg.topic[19:21])
            message.devID = int(msg.topic[25:27])
            message.payload = str(msg.payload)
            if message.payload == "READ":
                message.cmd = 1
            
            statMess = message.devID in [5, 6, 8] + range(16, 31)
            realMess = message.devID in [0, 2, 3, 4] + range(40, 71) and message.cmd == 1
            intMess = message.devID in [1, 7] + range(32, 39)
            strMess = message.devID == 72
            
            if message.nodeID == 1:
                if message.devID == 0:
                    try:
                        with open('/proc/uptime', 'r') as uptime_file:
                            uptime = int(float(uptime_file.readline().split()[0]) / 60)
                    except:
                        uptime = 0
                    self.mqttc.publish("home/rfm_gw/nb/node01/dev00", uptime)
                elif message.devID == 3:
                    self.mqttc.publish("home/rfm_gw/nb/node01/dev03", VERSION)
                return
            else:
                if statMess:
                    if message.payload == "ON":
                        message.intVal = 1
                    elif message.payload == "OFF":
                        message.intVal = 0
                    else:
                        #invalid status command
                        self.error(3, message.nodeID)
                        return
                elif realMess:
                    try:
                        message.fltVal = float(message.payload)
                    except:
                        pass
                elif intMess:
                    if message.cmd == 0:
                        message.intVal = int(message.payload)
                elif strMess:
                    pass
                else:
                    #invalid devID
                    self.error(4, message.nodeID)
                    return
            
            message.setMessage()
            writeQ.put(message)
        
    def processPacket(self, packet):
		message = Message(packet)
              
		print time.strftime("[%Y-%m-%d %H:%M] ") + "Message from node %d, sensorID %d, uptime %u, data %e, battery %e" % (message.nodeID, message.sensorID, message.uptime, message.data, message.battery);        

        # send sensor data
		self.mqttc.publish("home/rfm_gw/nb/node%02d/dev%02d/data" % (message.nodeID, message.sensorID), message.data)
		# send uptime
		self.mqttc.publish("home/rfm_gw/nb/node%02d/dev%02d/uptime" % (message.nodeID, message.sensorID), message.uptime)
		# send battery state
		self.mqttc.publish("home/rfm_gw/nb/node%02d/battery" % (message.nodeID), message.battery)
		# send rssi
		self.mqttc.publish("home/rfm_gw/nb/node%02d/rssi" % (message.nodeID), gw.radio.RSSI)

    def sendMessage(self, message):
        if not self.radio.sendWithRetry(message.nodeID, message.message, 5, 30):
            self.mqttc.publish("home/rfm_gw/nb/node%02d/dev90" % (message.nodeID, ), 
                               "connection lost node %d" % (message.nodeID))
    
    def error(self, code, dest):
        self.mqttc.publish("home/rfm_gw/nb/node01/dev91", "syntax error %d for node %d" % (code, dest))

    def stop(self):
        print "shutting down mqqt"
        self.mqttc.loop_stop()
        print "shutting down radio"
        self.radio.shutdown()

def handler(signum, frame):
    print "\nExiting..."
    gw.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, handler)

gw = Gateway(FREQ, NETWORKID, KEY)

if __name__ == "__main__":
    while True:
        gw.receiveBegin()
        while not gw.receiveDone():
            try:
                message = writeQ.get(block = False)
                gw.sendMessage(message)
            except Queue.Empty:
                pass
            time.sleep(.1)
        if gw.radio.ACK_RECEIVED:
            continue
        packet = bytearray(gw.radio.DATA)
        if gw.radio.ACKRequested():
            gw.radio.sendACK(gw.radio.SENDERID)
        gw.processPacket(packet)
