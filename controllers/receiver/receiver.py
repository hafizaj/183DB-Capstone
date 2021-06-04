# receiver.py
from controller import *


receiverRobot = Robot()
TIME_STEP = int(receiverRobot.getBasicTimeStep())
b_receiver = receiverRobot.getDevice('test_receiver')
emitter = receiverRobot.getDevice('test_emitter')

b_receiver.enable(TIME_STEP)

t = 0
while(t<1000):
	receiverRobot.step(TIME_STEP)
	if b_receiver.getQueueLength() > 0:
		A = b_receiver.getData()
		A_decoded = int.from_bytes(A, byteorder='big')
		b_receiver.nextPacket()
	t = t+TIME_STEP

# Next function
receiverRobot.step(TIME_STEP)

# emitter.send(bytes([3]))
while t<10000:
	receiverRobot.step(TIME_STEP)
