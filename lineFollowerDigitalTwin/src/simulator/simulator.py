#!/usr/bin/env python3
from __future__ import print_function
import struct
import sys
import argparse
import math

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)

import VsiCommonPythonApi as vsiCommonPythonApi
import VsiTcpUdpPythonGateway as vsiEthernetPythonGateway


class MySignals:
	def __init__(self):
		# Inputs
		self.v = 0
		self.omega = 0
		self.t_cmd = 0

		# Outputs
		self.x = 0
		self.y = 0
		self.theta = 0
		self.t = 0
		self.pathX = [0] * 500
		self.pathY = [0] * 500
		self.pathLen = 0



srcMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0x01]
controllerMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0x02]
visualizerMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0x03]
srcIpAddress = [192, 168, 1, 10]
controllerIpAddress = [192, 168, 1, 11]
visualizerIpAddress = [192, 168, 1, 12]

ControllerSocketPortNumber0 = 8070
SimulatorSocketPortNumber1 = 8080

Simulator0 = 0
Visualizer1 = 1


# Start of user custom code region. Please apply edits only within these regions:  Global Variables & Definitions
import random
import numpy as np



# -- helper functions

def make_bezier_chain(control_points, resolution=0.02):
    cps = np.array(control_points)
    if cps.shape[0] < 4:
        raise ValueError("Need at least 4 control points")
    pts = []
    def cubic_bezier(p0,p1,p2,p3,t):
        return ((1-t)**3)*p0 + 3*((1-t)**2)*t*p1 + 3*(1-t)*(t**2)*p2 + (t**3)*p3
    for i in range(0, cps.shape[0]-3, 3):
        for t in np.arange(0, 1+resolution, resolution):
            pt = cubic_bezier(cps[i], cps[i+1], cps[i+2], cps[i+3], t)
            pts.append(pt)
    return np.array(pts).tolist()

# noise parameters
noise_std_pos = 0.01   # meters (std dev for x, y)
noise_std_theta = 0.005  # radians

# robot state
state = {"x": 0.5, "y": 0.3, "theta": 0.0}

# parameters
v_nominal = 0.6   # nominal forward velocity
dt = 0.02         # integration timestep (will be synced to VSI step)
L = 50.0           # path length in meters

# custom_x = random.randint(0, 500)/ 100.0
# custom_y = random.randint(-100, 100)/ 100.0
# while custom_y < 0.05:
# 	custom_y = random.randint(-100, 100)/ 100.0

custom_x = 1.0
custom_y = 0.7

cps = [[0,0],[2,0],[4,4],[6,0],[8,-4],[10,0],[12,0]]
path = make_bezier_chain(cps, 0.01)
# path points (straight along x)
path_points = path
# path_points = [(i*0.1, 0.0) for i in range(int(L/0.1))]

# fill into MySignals initial
# note: MySignals has fixed size 500 arrays
# we populate the first N entries
for i, (px, py) in enumerate(path_points):
    MySignals().pathX[i] = px
    MySignals().pathY[i] = py
MySignals().pathLen = len(path_points)


# End of user custom code region. Please don't edit beyond this point.
class Simulator:

	def __init__(self, args):
		self.componentId = 0
		self.localHost = args.server_url
		self.domain = args.domain
		self.portNum = 50101
        
		self.simulationStep = 0
		self.stopRequested = False
		self.totalSimulationTime = 0
        
		self.receivedNumberOfBytes = 0
		self.receivedPayload = []

		self.numberOfPorts = 2
		self.clientPortNum = [0] * self.numberOfPorts
		self.receivedDestPortNumber = 0
		self.receivedSrcPortNumber = 0
		self.expectedNumberOfBytes = 0
		self.mySignals = MySignals()

		# Start of user custom code region. Please apply edits only within these regions:  Constructor

		# End of user custom code region. Please don't edit beyond this point.



	def mainThread(self):
		dSession = vsiCommonPythonApi.connectToServer(self.localHost, self.domain, self.portNum, self.componentId)
		vsiEthernetPythonGateway.initialize(dSession, self.componentId, bytes(srcMacAddress), bytes(srcIpAddress))
		try:
			vsiCommonPythonApi.waitForReset()

			# Start of user custom code region. Please apply edits only within these regions:  After Reset

			state["x"] = custom_x
			state["y"] = custom_y
			# state["x"] = 2.5
			# state["y"] = 0.8
			state["theta"] = 0.0
			self.mySignals.t = 0.0
			self.mySignals.x = state["x"]
			self.mySignals.y = state["y"]
			self.mySignals.theta = state["theta"]


			# End of user custom code region. Please don't edit beyond this point.
			self.updateInternalVariables()

			if(vsiCommonPythonApi.isStopRequested()):
				raise Exception("stopRequested")
			self.establishTcpUdpConnection()
			nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()
			while(vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime):

				# Start of user custom code region. Please apply edits only within these regions:  Inside the while loop

				# simple unicycle model update
				v = self.mySignals.v if self.mySignals.v != 0 else v_nominal
				omega = self.mySignals.omega

				# integrate pose
				state["x"] += v * math.cos(state["theta"]) * dt
				state["y"] += v * math.sin(state["theta"]) * dt
				state["theta"] += omega * dt

				# add Gaussian noise
				noisy_x = state["x"] + np.random.normal(0, noise_std_pos)
				noisy_y = state["y"] + np.random.normal(0, noise_std_pos)
				noisy_theta = state["theta"] + np.random.normal(0, noise_std_theta)

				# update outputs with noisy values
				self.mySignals.x = noisy_x
				self.mySignals.y = noisy_y
				self.mySignals.theta = noisy_theta

				
				
				sim_time_ns = vsiCommonPythonApi.getSimulationTimeInNs()
				if sim_time_ns is None:
					sim_time_ns = 0
				self.mySignals.t = sim_time_ns * 1e-9  # ns → s
				self.mySignals.pathLen = len(path_points)
				for i, (px, py) in enumerate(path_points):
					self.mySignals.pathX[i] = px
					self.mySignals.pathY[i] = py


				# End of user custom code region. Please don't edit beyond this point.

				self.updateInternalVariables()

				if(vsiCommonPythonApi.isStopRequested()):
					raise Exception("stopRequested")

				if(vsiEthernetPythonGateway.isTerminationOnGoing()):
					print("Termination is on going")
					break

				if(vsiEthernetPythonGateway.isTerminated()):
					print("Application terminated")
					break

				receivedData = vsiEthernetPythonGateway.recvEthernetPacket(ControllerSocketPortNumber0)
				if(receivedData[3] != 0):
					self.decapsulateReceivedData(receivedData)

				receivedData = vsiEthernetPythonGateway.recvEthernetPacket(self.clientPortNum[Visualizer1])
				if(receivedData[3] != 0):
					self.decapsulateReceivedData(receivedData)

				# Start of user custom code region. Please apply edits only within these regions:  Before sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				#Send ethernet packet to controller
				self.sendEthernetPacketTocontroller()

				#Send ethernet packet to visualizer
				self.sendEthernetPacketTovisualizer()

				# Start of user custom code region. Please apply edits only within these regions:  After sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				print("\n+=simulator+=")
				print("  VSI time:", end = " ")
				print(vsiCommonPythonApi.getSimulationTimeInNs(), end = " ")
				print("ns")
				print("  Inputs:")
				print("\tv =", end = " ")
				print(self.mySignals.v)
				print("\tomega =", end = " ")
				print(self.mySignals.omega)
				print("\tt_cmd =", end = " ")
				print(self.mySignals.t_cmd)
				print("  Outputs:")
				print("\tx =", end = " ")
				print(self.mySignals.x)
				print("\ty =", end = " ")
				print(self.mySignals.y)
				print("\ttheta =", end = " ")
				print(self.mySignals.theta)
				print("\tt =", end = " ")
				print(self.mySignals.t)
				print("\tpathX =", end = " ")
				print("[", *self.mySignals.pathX[:10], "...", *self.mySignals.pathX[-10:], "]")
				print("\tpathY =", end = " ")
				print("[", *self.mySignals.pathY[:10], "...", *self.mySignals.pathY[-10:], "]")
				print("\tpathLen =", end = " ")
				print(self.mySignals.pathLen)
				print("\n\n")

				self.updateInternalVariables()

				if(vsiCommonPythonApi.isStopRequested()):
					raise Exception("stopRequested")
				nextExpectedTime += self.simulationStep

				if(vsiCommonPythonApi.getSimulationTimeInNs() >= nextExpectedTime):
					continue

				if(nextExpectedTime > self.totalSimulationTime):
					remainingTime = self.totalSimulationTime - vsiCommonPythonApi.getSimulationTimeInNs()
					vsiCommonPythonApi.advanceSimulation(remainingTime)
					break

				vsiCommonPythonApi.advanceSimulation(nextExpectedTime - vsiCommonPythonApi.getSimulationTimeInNs())

			if(vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime):
				vsiEthernetPythonGateway.terminate()
		except Exception as e:
			if str(e) == "stopRequested":
				print("Terminate signal has been received from one of the VSI clients")
				# Advance time with a step that is equal to "simulationStep + 1" so that all other clients
				# receive the terminate packet before terminating this client
				vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)
			else:
				print(f"An error occurred: {str(e)}")
		except:
			# Advance time with a step that is equal to "simulationStep + 1" so that all other clients
			# receive the terminate packet before terminating this client
			vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)



	def establishTcpUdpConnection(self):
		if(self.clientPortNum[Simulator0] == 0):
			self.clientPortNum[Simulator0] = vsiEthernetPythonGateway.tcpConnect(bytes(controllerIpAddress), ControllerSocketPortNumber0)

		if(self.clientPortNum[Visualizer1] == 0):
			self.clientPortNum[Visualizer1] = vsiEthernetPythonGateway.tcpListen(SimulatorSocketPortNumber1)

		if(self.clientPortNum[Visualizer1] == 0):
			print("Error: Failed to connect to port: Controller on TCP port: ") 
			print(ControllerSocketPortNumber0)
			exit()

		if(self.clientPortNum[Visualizer1] == 0):
			print("Error: Failed to connect to port: Simulator on TCP port: ") 
			print(SimulatorSocketPortNumber1)
			exit()



	def decapsulateReceivedData(self, receivedData):
		self.receivedDestPortNumber = receivedData[0]
		self.receivedSrcPortNumber = receivedData[1]
		self.receivedNumberOfBytes = receivedData[3]
		self.receivedPayload = [0] * (self.receivedNumberOfBytes)

		for i in range(self.receivedNumberOfBytes):
			self.receivedPayload[i] = receivedData[2][i]

		if(self.receivedSrcPortNumber == ControllerSocketPortNumber0):
			print("Received packet from controller")
			receivedPayload = bytes(self.receivedPayload)
			self.mySignals.v, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.omega, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.t_cmd, receivedPayload = self.unpackBytes('d', receivedPayload)


	def sendEthernetPacketTocontroller(self):
		bytesToSend = bytes()

		bytesToSend += self.packBytes('d', self.mySignals.x)

		bytesToSend += self.packBytes('d', self.mySignals.y)

		bytesToSend += self.packBytes('d', self.mySignals.theta)

		bytesToSend += self.packBytes('d', self.mySignals.t)

		bytesToSend += self.packBytes('d', self.mySignals.pathX)

		bytesToSend += self.packBytes('d', self.mySignals.pathY)

		bytesToSend += self.packBytes('i', self.mySignals.pathLen)

		#Send ethernet packet to controller
		vsiEthernetPythonGateway.sendEthernetPacket(ControllerSocketPortNumber0, bytes(bytesToSend))

	def sendEthernetPacketTovisualizer(self):
		bytesToSend = bytes()

		bytesToSend += self.packBytes('d', self.mySignals.x)

		bytesToSend += self.packBytes('d', self.mySignals.y)

		bytesToSend += self.packBytes('d', self.mySignals.theta)

		bytesToSend += self.packBytes('d', self.mySignals.t)

		bytesToSend += self.packBytes('d', self.mySignals.pathX)

		bytesToSend += self.packBytes('d', self.mySignals.pathY)

		bytesToSend += self.packBytes('i', self.mySignals.pathLen)

		#Send ethernet packet to visualizer
		vsiEthernetPythonGateway.sendEthernetPacket(self.clientPortNum[Visualizer1], bytes(bytesToSend))

		# Start of user custom code region. Please apply edits only within these regions:  Protocol's callback function

		# End of user custom code region. Please don't edit beyond this point.



	def packBytes(self, signalType, signal):
		if isinstance(signal, list):
			if signalType == 's':
				packedData = b''
				for str in signal:
					str += '\0'
					str = str.encode('utf-8')
					packedData += struct.pack(f'={len(str)}s', str)
				return packedData
			else:
				return struct.pack(f'={len(signal)}{signalType}', *signal)
		else:
			if signalType == 's':
				signal += '\0'
				signal = signal.encode('utf-8')
				return struct.pack(f'={len(signal)}s', signal)
			else:
				return struct.pack(f'={signalType}', signal)



	def unpackBytes(self, signalType, packedBytes, signal = ""):
		if isinstance(signal, list):
			if signalType == 's':
				unpackedStrings = [''] * len(signal)
				for i in range(len(signal)):
					nullCharacterIndex = packedBytes.find(b'\0')
					if nullCharacterIndex == -1:
						break
					unpackedString = struct.unpack(f'={nullCharacterIndex}s', packedBytes[:nullCharacterIndex])[0].decode('utf-8')
					unpackedStrings[i] = unpackedString
					packedBytes = packedBytes[nullCharacterIndex + 1:]
				return unpackedStrings, packedBytes
			else:
				unpackedVariable = struct.unpack(f'={len(signal)}{signalType}', packedBytes[:len(signal)*struct.calcsize(f'={signalType}')])
				packedBytes = packedBytes[len(unpackedVariable)*struct.calcsize(f'={signalType}'):]
				return list(unpackedVariable), packedBytes
		elif signalType == 's':
			nullCharacterIndex = packedBytes.find(b'\0')
			unpackedVariable = struct.unpack(f'={nullCharacterIndex}s', packedBytes[:nullCharacterIndex])[0].decode('utf-8')
			packedBytes = packedBytes[nullCharacterIndex + 1:]
			return unpackedVariable, packedBytes
		else:
			numBytes = 0
			if signalType in ['?', 'b', 'B']:
				numBytes = 1
			elif signalType in ['h', 'H']:
				numBytes = 2
			elif signalType in ['f', 'i', 'I', 'L', 'l']:
				numBytes = 4
			elif signalType in ['q', 'Q', 'd']:
				numBytes = 8
			else:
				raise Exception('received an invalid signal type in unpackBytes()')
			unpackedVariable = struct.unpack(f'={signalType}', packedBytes[0:numBytes])[0]
			packedBytes = packedBytes[numBytes:]
			return unpackedVariable, packedBytes

	def updateInternalVariables(self):
		self.totalSimulationTime = vsiCommonPythonApi.getTotalSimulationTime()
		self.stopRequested = vsiCommonPythonApi.isStopRequested()
		self.simulationStep = vsiCommonPythonApi.getSimulationStep()



def main():
	inputArgs = argparse.ArgumentParser(" ")
	inputArgs.add_argument('--domain', metavar='D', default='AF_UNIX', help='Socket domain for connection with the VSI TLM fabric server')
	inputArgs.add_argument('--server-url', metavar='CO', default='localhost', help='server URL of the VSI TLM Fabric Server')

	# Start of user custom code region. Please apply edits only within these regions:  Main method

	# End of user custom code region. Please don't edit beyond this point.

	args = inputArgs.parse_args()
                      
	simulator = Simulator(args)
	simulator.mainThread()



if __name__ == '__main__':
    main()
