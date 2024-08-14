import time
import math
import unitree_motor_command as um

unitree = um.unitree_communication('/dev/unitree-l')
MOTOR1 = unitree.createMotor(motor_number = 1,initalposition = 0.669,MAX=8.475,MIN=-5.364)
MOTOR2 = unitree.createMotor(motor_number = 2,initalposition = 3.815,MAX=26.801,MIN=-1)
unitree2 = um.unitree_communication('/dev/unitree-r')
MOTOR4 = unitree2.createMotor(motor_number = 4,initalposition = 1.247,MAX=5.364,MIN=-8.475)
MOTOR5 = unitree2.createMotor(motor_number = 5,initalposition = 5.046,MAX=1,MIN=-26.801)

def test():
    try:

        unitree.inital_all_motor()
        unitree2.inital_all_motor()
        
        CMD = input('CMD:')

        if CMD == "s":
            while MOTOR1.data.q >= MOTOR1.inital_position + 0.33*6.33 and MOTOR4.data.q  <= MOTOR4.inital_position  :
                unitree.position_force_velocity_cmd(motor_number = 1,kp = 0,kd = 0.1, position = 0 ,torque = 0, velocity = 0.01)
                unitree2.position_force_velocity_cmd(motor_number = 4 ,kp = 0,kd = 0.1, position = 0 ,torque = 0, velocity=-0.01)
            time.sleep(0.01)
            for i in range(36):                        
                unitree.position_force_velocity_cmd(motor_number = 1,kp = i,kd = 0.12, position = MOTOR1.inital_position + 0.33*6.33)
                unitree2.position_force_velocity_cmd(motor_number = 4 ,kp = i,kd = 0.12, position = MOTOR4.inital_position - 0.33*6.33)
                time.sleep(0.1)
            while MOTOR2.data.q >= MOTOR2.inital_position + 0.33*6.33*1.6 and MOTOR5.data.q  <= MOTOR5.inital_position - 0.33*6.33*1.6:
                unitree.position_force_velocity_cmd(motor_number = 2,kp = 0,kd = 0.16, position = 0 ,torque = 0, velocity = 0.01)
                unitree2.position_force_velocity_cmd(motor_number = 5 ,kp = 0,kd = 0.16, position = 0 ,torque = 0, velocity=-0.01)
            time.sleep(0.01)
            for i in range(36):                        
                unitree.position_force_velocity_cmd(motor_number = 2,kp = i,kd = 0.15, position = MOTOR2.inital_position + 0.6*6.33*1.6)
                unitree2.position_force_velocity_cmd(motor_number = 5 ,kp = i,kd = 0.15, position = MOTOR5.inital_position - 0.6*6.33*1.6)
                time.sleep(0.1)
     
            while True:
                pass
    
        else:
            unitree.disableallmotor()
            unitree2.disableallmotor()

    except KeyboardInterrupt:
        unitree.disableallmotor()
        unitree2.disableallmotor()
        time.sleep(0.1)

def test1():
    
    while True:

        if input('cmd')== 'up':
            
            unitree.inital_all_motor()
            unitree2.inital_all_motor()
            thigh1 = MOTOR1.data.q
            thigh4 = MOTOR4.data.q
            calf2 = MOTOR2.data.q
            calf5 = MOTOR5.data.q
            print("thigh1: "+ str(thigh1/6.33))
            print("thigh4: "+ str(thigh4/6.33))
            print("calf2: "+ str(calf2/(6.33*1.6)))
            print("calf5: "+ str(calf5/(6.33*1.6)))

        elif input('cmd')== 'down':

            unitree.inital_all_motor()
            unitree2.inital_all_motor()
            thigh01 = MOTOR1.data.q
            thigh04 = MOTOR4.data.q
            calf02 = MOTOR2.data.q
            calf05 = MOTOR5.data.q
            print("thigh01: "+ str(thigh01/6.33))
            print("thigh04: "+ str(thigh04/6.33))
            print("calf02: "+ str(calf02/(6.33*1.6)))
            print("calf05: "+ str(calf05/(6.33*1.6)))
        
        elif input("cmd")== 'r':

            print("thigh01-1: "+ str((thigh01-thigh1)/6.33))
            print("thigh04-4: "+ str((thigh04-thigh4)/6.33))
            print("calf02-2: "+ str((calf02-calf2)/(6.33*1.6)))
            print("calf05-5: "+ str((calf05-calf5)/(6.33*1.6)))

def ik_solver(l1=0 ,l2 = 0, x= 0, y=0):

    r = math.sqrt(x**2, y**2)

    if r > (l1 + l2) or r < abs(l1 - l2):
        raise ValueError("Target point is out of reach.")

    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    theta2 = math.acos(cos_theta2)
    k1 = l1 + l2 * cos_theta2
    k2 = l2 * math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)
    return theta1, theta2

def fk_solver(l1 =0, l2=0, theta1=0, theta2=0):

    x=l1*math.cos(theta1)+l2*math.cos(theta2+theta1)
    y=l1*math.sinsin(theta1)+l2*math.sin(theta2+theta1)
    return x,y

def ik_test(x_position = 0,y_position = 0):

    unitree.position_force_velocity_cmd(motor_number=1,torque=MOTOR1.cmd.tau,kp=MOTOR1.cmd.kp,kd=MOTOR1.cmd.kd,position=MOTOR1.cmd.q,velocity=MOTOR1.cmd.dq)
    unitree.position_force_velocity_cmd(motor_number=2,torque=MOTOR2.cmd.tau,kp=MOTOR2.cmd.kp,kd=MOTOR2.cmd.kd,position=MOTOR2.cmd.q,velocity=MOTOR2.cmd.dq)
    unitree2.position_force_velocity_cmd(motor_number=4,torque=MOTOR4.cmd.tau,kp=MOTOR4.cmd.kp,kd=MOTOR4.cmd.kd,position=MOTOR4.cmd.q,velocity=MOTOR4.cmd.dq)
    unitree2.position_force_velocity_cmd(motor_number=5,torque=MOTOR5.cmd.tau,kp=MOTOR5.cmd.kp,kd=MOTOR5.cmd.kd,position=MOTOR5.cmd.q,velocity=MOTOR5.cmd.dq)

    theta1,theta2 = ik_solver(l1 =0.215,l2 =0.215,x=x_position,y=y_position)


if __name__ == "__main__":

    test()