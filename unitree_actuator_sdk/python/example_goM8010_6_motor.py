import time
import sys
import math
sys.path.append('/home/crazydog/Desktop/unitree_actuator_sdk/lib')
from unitree_actuator_sdk import *
import sys



cmd = MotorCmd()
data = MotorData()
data.motorType = MotorType.GO_M8010_6
cmd.motorType = MotorType.GO_M8010_6
# motor=int(input("Which motor?:"))
# if motor == 1 or motor == 2:
#     serial = SerialPort('/dev/ttyUSB0')
# else:
#     serial = SerialPort('/dev/ttyUSB1')
while True:
    # data.motorType = MotorType.GO_M8010_6
    # cmd.motorType = MotorType.GO_M8010_6
    motor=int(input("Which motor?:"))
    if motor == 1 or motor == 2:
        serial = SerialPort('/dev/unitree-l')
    else:
        serial = SerialPort('/dev/unitree-r')
    cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.FOC)
    cmd.id   = motor
    cmd.q    = 0.0
    cmd.dq   = 0*queryGearRatio(MotorType.GO_M8010_6)
    cmd.kp   = 0.0
    cmd.kd   = 0.0
    cmd.tau  = 0.0
    serial.sendRecv(cmd, data)
    print('\n')
    print("q: " + str(data.q))
    print("dq: " + str(data.dq))
    print("temp: " + str(data.temp))
    print("merror: " + str(data.merror))
    print('\n')

    time.sleep(0.0002) # 200 us

