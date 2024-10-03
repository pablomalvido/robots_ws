#! /usr/bin/env python
import struct
import socket
import rospy
from norbdo_force_sensor.msg import forces
from std_srvs.srv import Trigger, TriggerResponse

rospy.init_node('norbdo_fs', anonymous=True)
tare_srv = str(rospy.get_namespace())+'tare'
abs_srv = str(rospy.get_namespace())+'absolute'
forces_pub = rospy.Publisher(str(rospy.get_namespace())+"/forces", forces, queue_size=1)
tare_signal = True
abs_signal = False
tare_forces = [0,0,0,0,0,0]

IP_ADDR	= str(rospy.get_param('~ip', ""));
PORT	= int(rospy.get_param('~port', ""));

print("Connection stablished with " + IP_ADDR + ":" + str(PORT))

CMD_TYPE_SENSOR_TRANSMIT 	= '07'
SENSOR_TRANSMIT_TYPE_START 	= '01'
SENSOR_TRANSMIT_TYPE_STOP 	= '00'

CMD_TYPE_SET_CURRENT_TARE 	= '15'
SET_CURRENT_TARE_TYPE_NEGATIVE	= '01'

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)	

def main():
	global s
	global tare_signal
	global abs_signal
	global tare_forces

	s.settimeout(2.0)
	s.connect((IP_ADDR, PORT))

	sendData = '03' + CMD_TYPE_SET_CURRENT_TARE + SET_CURRENT_TARE_TYPE_NEGATIVE
	sendData = bytearray.fromhex(sendData)
	s.send(sendData)
	recvData = recvMsg()

	sendData = '03' + CMD_TYPE_SENSOR_TRANSMIT + SENSOR_TRANSMIT_TYPE_START
	sendData = bytearray.fromhex(sendData)
	s.send(sendData)
	recvData = recvMsg()

	i=0
	while not rospy.is_shutdown():
		recvData = recvMsg()
		Fx = struct.unpack('!d', recvData[2:10])[0]
		Fy = struct.unpack('!d', recvData[10:18])[0]
		Fz = struct.unpack('!d', recvData[18:26])[0]
		Tx = struct.unpack('!d', recvData[26:34])[0]
		Ty = struct.unpack('!d', recvData[34:42])[0]
		Tz = struct.unpack('!d', recvData[42:50])[0]
		if tare_signal:
			tare_forces = [float(Fx),float(Fy),float(Fz),float(Tx),float(Ty),float(Tz)]
			tare_signal = False
		if abs_signal:
			tare_forces = [0,0,0,0,0,0]
			abs_signal = False
		if i>=50: #Freq 1000/50 = 20Hz
			#print('Fx: '+str(Fx) + ", Fy: " + str(Fy) + ", Fz: " + str(Fz) + ", Tx: " + str(Tx) + ", Ty: " + str(Ty) + ", Tz: " + str(Tz))
			force_msg = forces()
			force_msg.Fx = float(Fx) - tare_forces[0]
			force_msg.Fy = float(Fy) - tare_forces[1]
			force_msg.Fz = float(Fz) - tare_forces[2]
			force_msg.Tx = float(Tx) - tare_forces[3]
			force_msg.Ty = float(Ty) - tare_forces[4]
			force_msg.Tz = float(Tz) - tare_forces[5]
			forces_pub.publish(force_msg)
			i=0
		else:
			i+=1

	sendData = '03' + CMD_TYPE_SENSOR_TRANSMIT + SENSOR_TRANSMIT_TYPE_STOP
	sendData = bytearray.fromhex(sendData)
	s.send(sendData)
	recvData = recvMsg()

	#Wait until and ACK msg is send back for the stop command. 
	while recvData[0] != 3 and recvData[1] != CMD_TYPE_SENSOR_TRANSMIT:
		recvData = recvMsg()

	s.close()

def recvMsg():
	recvData = bytearray(s.recv(2))

	while len(recvData) < recvData[0] :
		recvData += bytearray(s.recv(recvData[0] - len(recvData)))

	#printMsg(recvData)

	return recvData

def printMsg(msg):
	print("Msg len: " + str(msg[0]) + " Msg type: " + str(msg[1]) + "")
	
	dataStr = "DATA: "
	for i in range(msg[0] - 2):
		dataStr += str(msg[i + 2]) + " "

	print(dataStr)

def tare_callback(req): 
	"""
	Tares the sensor values
	"""
	global tare_signal
	print("Taring force sensor")
	tare_signal = True
	resp = TriggerResponse()
	resp.success = True
	return resp

rospy.Service(tare_srv, Trigger, tare_callback)

def abs_callback(req): 
	"""
	Tares the sensor values
	"""
	global abs_signal
	print("Reseting force sensor values")
	abs_signal = True
	resp = TriggerResponse()
	resp.success = True
	return resp         

rospy.Service(abs_srv, Trigger, abs_callback)

if __name__ == "__main__":
	main()