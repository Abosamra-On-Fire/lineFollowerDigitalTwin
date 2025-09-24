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
		self.x = 0
		self.y = 0
		self.theta = 0
		self.t = 0
		self.pathX = [0] * 500
		self.pathY = [0] * 500
		self.pathLen = 0

		# Outputs
		self.v = 0
		self.omega = 0
		self.t_cmd = 0



srcMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0x02]
simulatorMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0x01]
srcIpAddress = [192, 168, 1, 11]
simulatorIpAddress = [192, 168, 1, 10]

ControllerSocketPortNumber0 = 8070

Simulator0 = 0

# Start of user custom code region. Please apply edits only within these regions:  Global Variables & Definitions

import numpy as np

# Controller parameters
lookahead_distance = 0.5   # meters
v_nominal = 2            # m/s

# PID gains
Kp = 11.2
Ki = 0.8
Kd = 3.2
dt = 0.02

class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.i = 0.0
        self.prev = 0.0
    def step(self, e):
        self.i += e * self.dt
        d = (e - self.prev) / self.dt
        u = self.kp * e + self.ki * self.i + self.kd * d
        self.prev = e
        return u

def lateral_and_heading_error(pose, pathX, pathY, pathLen):
    if pathLen <= 1:
        return 0.0, 0.0, 0
    p = np.array(list(zip(pathX[:pathLen], pathY[:pathLen])))
    pt = np.array([pose["x"], pose["y"]])
    dists = np.sqrt(((p - pt) ** 2).sum(axis=1))
    idx = dists.argmin()
    if idx == 0:
        tangent = p[1] - p[0]
    elif idx == len(p) - 1:
        tangent = p[-1] - p[-2]
    else:
        tangent = p[idx + 1] - p[idx - 1]
    path_theta = math.atan2(tangent[1], tangent[0])
    normal = np.array([-math.sin(path_theta), math.cos(path_theta)])
    lat_err = float(np.dot(pt - p[idx], normal))
    head_err = float(((pose["theta"] - path_theta + math.pi) % (2 * math.pi)) - math.pi)
    return lat_err, head_err, idx

# Initialize PID
pid = PID(Kp, Ki, Kd, dt)

# End of user custom code region. Please don't edit beyond this point.

class Controller:

	def __init__(self, args):
		self.componentId = 1
		self.localHost = args.server_url
		self.domain = args.domain
		self.portNum = 50102
        
		self.simulationStep = 0
		self.stopRequested = False
		self.totalSimulationTime = 0
        
		self.receivedNumberOfBytes = 0
		self.receivedPayload = []

		self.numberOfPorts = 1
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

			# End of user custom code region. Please don't edit beyond this point.
			self.updateInternalVariables()

			if(vsiCommonPythonApi.isStopRequested()):
				raise Exception("stopRequested")
			self.establishTcpUdpConnection()
			nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()
			while(vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime):

				# Start of user custom code region. Please apply edits only within these regions:  Inside the while loop
				# Start of user custom code region. Please apply edits only within these regions:  Inside the while loop
				
				# Receive latest packet from simulator
				receivedData = vsiEthernetPythonGateway.recvEthernetPacket(self.clientPortNum[Simulator0])
				if(receivedData[3] != 0):
				    self.decapsulateReceivedData(receivedData)
				
				# Compute control if we have a path
				if self.mySignals.pathLen > 1:
				    pose = {"x": self.mySignals.x,
				            "y": self.mySignals.y,
				            "theta": self.mySignals.theta,
				            "t": self.mySignals.t}
				
				    lat, head, idx = lateral_and_heading_error(
				        pose, self.mySignals.pathX, self.mySignals.pathY, self.mySignals.pathLen)
				
				    err = lat + 0.2 * head
				    omega = -pid.step(err)
				    omega = max(min(omega, 2.5), -2.5)
				
				    v = v_nominal * (1.0 - min(abs(lat) / 0.6, 0.6))
				    if idx > self.mySignals.pathLen - 20:
				        v *= 0.5
				
				    self.mySignals.v = v
				    self.mySignals.omega = omega
				    self.mySignals.t_cmd = self.mySignals.t
				
				    if int(self.mySignals.t) % 1 == 0:
				        print(f"[ctrl] t={self.mySignals.t:.2f} pos=({pose['x']:.3f},{pose['y']:.3f}) "
				              f"lat_err={lat:.3f} head_err={head:.3f} v={v:.3f} omega={omega:.3f}")
				else:
				    self.mySignals.v = 0.0
				    self.mySignals.omega = 0.0
				    self.mySignals.t_cmd = self.mySignals.t
				
				# End of user custom code region. Please don't edit beyond this point.

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

				receivedData = vsiEthernetPythonGateway.recvEthernetPacket(self.clientPortNum[Simulator0])
				if(receivedData[3] != 0):
					self.decapsulateReceivedData(receivedData)

				# Start of user custom code region. Please apply edits only within these regions:  Before sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				#Send ethernet packet to simulator
				self.sendEthernetPacketTosimulator()

				# Start of user custom code region. Please apply edits only within these regions:  After sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				print("\n+=controller+=")
				print("  VSI time:", end = " ")
				print(vsiCommonPythonApi.getSimulationTimeInNs(), end = " ")
				print("ns")
				print("  Inputs:")
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
				print("  Outputs:")
				print("\tv =", end = " ")
				print(self.mySignals.v)
				print("\tomega =", end = " ")
				print(self.mySignals.omega)
				print("\tt_cmd =", end = " ")
				print(self.mySignals.t_cmd)
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
			self.clientPortNum[Simulator0] = vsiEthernetPythonGateway.tcpListen(ControllerSocketPortNumber0)

		if(self.clientPortNum[Simulator0] == 0):
			print("Error: Failed to connect to port: Controller on TCP port: ") 
			print(ControllerSocketPortNumber0)
			exit()



	def decapsulateReceivedData(self, receivedData):
		self.receivedDestPortNumber = receivedData[0]
		self.receivedSrcPortNumber = receivedData[1]
		self.receivedNumberOfBytes = receivedData[3]
		self.receivedPayload = [0] * (self.receivedNumberOfBytes)

		for i in range(self.receivedNumberOfBytes):
			self.receivedPayload[i] = receivedData[2][i]

		if(self.receivedSrcPortNumber == self.clientPortNum[Simulator0]):
			print("Received packet from simulator")
			receivedPayload = bytes(self.receivedPayload)
			self.mySignals.x, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.y, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.theta, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.t, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.pathX, receivedPayload = self.unpackBytes('d', receivedPayload, signal = self.mySignals.pathX)

			self.mySignals.pathY, receivedPayload = self.unpackBytes('d', receivedPayload, signal = self.mySignals.pathY)

			self.mySignals.pathLen, receivedPayload = self.unpackBytes('i', receivedPayload)


	def sendEthernetPacketTosimulator(self):
		bytesToSend = bytes()

		bytesToSend += self.packBytes('d', self.mySignals.v)

		bytesToSend += self.packBytes('d', self.mySignals.omega)

		bytesToSend += self.packBytes('d', self.mySignals.t_cmd)

		#Send ethernet packet to simulator
		vsiEthernetPythonGateway.sendEthernetPacket(self.clientPortNum[Simulator0], bytes(bytesToSend))

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
                      
	controller = Controller(args)
	controller.mainThread()



if __name__ == '__main__':
    main()
