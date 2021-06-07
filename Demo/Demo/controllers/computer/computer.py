# computer.py
from controller import *


receiverRobot = Robot()
TIME_STEP = int(receiverRobot.getBasicTimeStep())
computer_receiver = receiverRobot.getDevice('computer_receiver')
computer_emitter = receiverRobot.getDevice('computer_emitter')

computer_receiver.enable(TIME_STEP)

t = 0
while(t<1000):
	receiverRobot.step(TIME_STEP)
	if computer_receiver.getQueueLength() > 0:
		A = computer_receiver.getData()
		A_decoded = int.from_bytes(A, byteorder='big')
		computer_receiver.nextPacket()
	t = t+TIME_STEP

# Next function
receiverRobot.step(TIME_STEP)
computer_emitter.send(bytes([2]))
computer_emitter.send(bytes([1]))
computer_emitter.send(bytes([4]))
computer_emitter.send(bytes([3]))



while t<10000:
	receiverRobot.step(TIME_STEP)