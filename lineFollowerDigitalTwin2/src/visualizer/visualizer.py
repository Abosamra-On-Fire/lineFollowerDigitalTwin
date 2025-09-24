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
import matplotlib.pyplot as plt
import numpy as np


class MySignals:
	def __init__(self):
		# Inputs
		self.v = 0
		self.omega = 0
		self.t_cmd = 0
		self.x = 0
		self.y = 0
		self.theta = 0
		self.t = 0
		self.pathX = [0] * 500
		self.pathY = [0] * 500
		self.pathLen = 0



srcMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0x03]
simulatorMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0x01]
srcIpAddress = [192, 168, 1, 12]
simulatorIpAddress = [192, 168, 1, 10]

SimulatorSocketPortNumber0 = 8080

Visualizer0 = 0


# Start of user custom code region. Please apply edits only within these regions:  Global Variables & Definitions

plt.ion()
fig, ax = plt.subplots()
path_line, = ax.plot([], [], 'r--', label='Path')
traj_line, = ax.plot([], [], 'b-', label='Trajectory')
robot_dot, = ax.plot([], [], 'go', markersize=8, label='Robot') 
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_title("Trajectory vs Path")
ax.legend()
traj_x = []
traj_y = []

# KPI storage
errors = []         # lateral error history
times = []          # time history
settling_tol = 0.05 # meters tolerance for settling time
overshoot = 0.0
settling_time = None
steady_state_error = None



# End of user custom code region. Please don't edit beyond this point.
class Visualizer:

	def __init__(self, args):
		self.componentId = 2
		self.localHost = args.server_url
		self.domain = args.domain
		self.portNum = 50103
        
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
			print("Visualizer ready. Plotting trajectory...")
			errors = []         # lateral error history
			times = []          # time history
			settling_tol = 0.05 # meters tolerance for settling time
			overshoot = 0.0
			settling_time = None
			steady_state_error = None
			# End of user custom code region. Please don't edit beyond this point.
			self.updateInternalVariables()

			if(vsiCommonPythonApi.isStopRequested()):
				raise Exception("stopRequested")
			self.establishTcpUdpConnection()
			nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()
			while(vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime):

				# Start of user custom code region. Please apply edits only within these regions:  Inside the while loop
				# Store trajectory point
				traj_x.append(self.mySignals.x)
				traj_y.append(self.mySignals.y)

				# Update path line if valid
				if self.mySignals.pathLen > 0:
					path_line.set_data(self.mySignals.pathX[:self.mySignals.pathLen],
									self.mySignals.pathY[:self.mySignals.pathLen])
					px = np.array(self.mySignals.pathX[:self.mySignals.pathLen])
					py = np.array(self.mySignals.pathY[:self.mySignals.pathLen])
					path = np.column_stack((px, py))
					pose = np.array([self.mySignals.x, self.mySignals.y])

					# find nearest point on path
					dists = np.linalg.norm(path - pose, axis=1)
					idx = np.argmin(dists)
					error = dists[idx]

					# log error and time
					errors.append(error)
					times.append(self.mySignals.t)

				# Update trajectory line
				traj_line.set_data(traj_x, traj_y)
				robot_dot.set_data([self.mySignals.x], [self.mySignals.y])

				# Autoscale axes
				ax.relim()
				ax.autoscale_view()

				plt.pause(0.001)


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

				receivedData = vsiEthernetPythonGateway.recvEthernetPacket(SimulatorSocketPortNumber0)
				if(receivedData[3] != 0):
					self.decapsulateReceivedData(receivedData)

				# Start of user custom code region. Please apply edits only within these regions:  Before sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				# Start of user custom code region. Please apply edits only within these regions:  After sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				print("\n+=visualizer+=")
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
				# reporting errors
				if errors:
					errors = np.array(errors)
					times = np.array(times)

					overshoot = np.max(errors)
					steady_state_error = errors[-1]

					# find settling time
					tol = settling_tol
					within_tol = np.abs(errors) <= tol
					for i in range(len(errors)-1, -1, -1):
						if not within_tol[i]:
							settling_time = times[i+1] if i+1 < len(times) else None
							break

					print("\n=== KPIs ===")
					print(f"Overshoot: {overshoot:.3f} m")
					print(f"Settling time: {settling_time:.2f} s" if settling_time else "Settling time: not settled")
					print(f"Steady-state error: {steady_state_error:.3f} m")

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
		if(self.clientPortNum[Visualizer0] == 0):
			self.clientPortNum[Visualizer0] = vsiEthernetPythonGateway.tcpConnect(bytes(simulatorIpAddress), SimulatorSocketPortNumber0)

		if(self.clientPortNum[Visualizer0] == 0):
			print("Error: Failed to connect to port: Simulator on TCP port: ") 
			print(SimulatorSocketPortNumber0)
			exit()



	def decapsulateReceivedData(self, receivedData):
		self.receivedDestPortNumber = receivedData[0]
		self.receivedSrcPortNumber = receivedData[1]
		self.receivedNumberOfBytes = receivedData[3]
		self.receivedPayload = [0] * (self.receivedNumberOfBytes)

		for i in range(self.receivedNumberOfBytes):
			self.receivedPayload[i] = receivedData[2][i]

		if(self.receivedSrcPortNumber == SimulatorSocketPortNumber0):
			print("Received packet from simulator")
			receivedPayload = bytes(self.receivedPayload)
			self.mySignals.x, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.y, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.theta, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.t, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.pathX, receivedPayload = self.unpackBytes('d', receivedPayload, signal = self.mySignals.pathX)

			self.mySignals.pathY, receivedPayload = self.unpackBytes('d', receivedPayload, signal = self.mySignals.pathY)

			self.mySignals.pathLen, receivedPayload = self.unpackBytes('i', receivedPayload)


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
                      
	visualizer = Visualizer(args)
	visualizer.mainThread()



if __name__ == '__main__':
    main()
