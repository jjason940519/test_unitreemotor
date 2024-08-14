import time
import sys
import matplotlib.pyplot as plt
import numpy as np
sys.path.append('/home/crazydog/Desktop/unitree_actuator_sdk/lib')
from unitree_actuator_sdk import * # type: ignorei
import  math
from unitree_motor_command import unitree_communication,unitree_motor
import traceback

unitree = unitree_communication('/dev/unitree-l')
MOTOR1 = unitree.createMotor(motor_number = 1)
MOTOR2 = unitree.createMotor(motor_number = 2)
unitree2 = unitree_communication('/dev/unitree-r')
MOTOR4 = unitree2.createMotor(motor_number = 4)
MOTOR5 = unitree2.createMotor(motor_number= 5)
wheel_x=-0.05
wheel_y=-0.12
def disableAllMotor():
    unitree.angular_velocity_cmd(motor_number=1, velocity=0, kd=0)
    unitree.angular_velocity_cmd(motor_number=2, velocity=0, kd=0)
    unitree2.angular_velocity_cmd(motor_number=4, velocity=0, kd=0)
    unitree2.angular_velocity_cmd(motor_number=5, velocity=0, kd=0)
def lockleg():
    unitree.position_force_cmd(motor_number=1, torque=0, kp=0.3, kd=0, position = MOTOR1.inital_position+(0.7*6.33))
    unitree.position_force_cmd(motor_number=2, torque=0, kp=0.3, kd=0, position = MOTOR2.inital_position)
    unitree2.position_force_cmd(motor_number=4, torque=0, kp=0.3, kd=0, position = MOTOR4.inital_position-(0.7*6.33))
    unitree2.position_force_cmd(motor_number=5, torque=0, kp=0.3, kd=0, position = MOTOR5.inital_position)
    time.sleep(1)
    unitree.position_force_cmd(motor_number=1, torque=0, kp=5, kd=0, position = MOTOR1.inital_position+(0.7*6.33))
    unitree.position_force_cmd(motor_number=2, torque=0, kp=5, kd=0, position = MOTOR2.inital_position)
    unitree2.position_force_cmd(motor_number=4, torque=0, kp=5, kd=0, position = MOTOR4.inital_position-(0.7*6.33))
    unitree2.position_force_cmd(motor_number=5, torque=0, kp=5, kd=0, position = MOTOR5.inital_position)
def test():
    pass
    # unitree.position_force_cmd(motor_number=1, torque=0, kp=20, kd=0, position= MOTOR1.data.q)
    # unitree.position_force_velocity_cmd()
def inverse_kinematics(x, y, L1=0.215, L2=0.215):
    # 計算 d    
    d = np.sqrt(x**2 + y**2)
    # 計算 theta2
    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = np.arccos(cos_theta2)
    # 計算 theta1
    theta1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))

    return theta1, theta2
def changeHeight(dx,dy):
    global wheel_x, wheel_y
    pi = math.pi
    theta1,theta2=inverse_kinematics(wheel_x+dx,wheel_y+dy)
    print(f'theta1: {theta1} theta2: {theta2}')
    motor1_output_end = MOTOR1.inital_position/6.33
    motor2_output_end = MOTOR2.inital_position/6.33
    motor4_output_end = MOTOR4.inital_position/6.33
    motor5_output_end = MOTOR5.inital_position/6.33
    theta1_dxl_L = motor1_output_end+(pi+theta1)
    theta1_dxl_R = motor4_output_end-(pi+theta1)
    theta2_dxl_L = motor2_output_end+((pi-pi/6)+theta2)
    theta2_dxl_R = motor5_output_end-((pi-pi/5)+theta2)

    #TODO grow height
    if theta1_dxl_L < motor1_output_end:
        print(f'theta1{theta1_dxl_L} motor1 at {motor1_output_end}out of constrain')
    elif theta1_dxl_R > motor4_output_end:
        print(f'theta1{theta1_dxl_R} motor4 at {motor4_output_end}out of constrain')
    elif theta2_dxl_L<motor2_output_end:
        print(f'theta2{theta2_dxl_L} motor2 at {motor2_output_end}out of constrain')
    elif theta2_dxl_R>motor5_output_end:
        print(f'theta2{theta2_dxl_R} motor5 at {motor5_output_end}out of constrain')
    else:
        print(f'motor1 at {motor1_output_end} going to theta1{theta1_dxl_L}')
        print(f'motor4 at {motor4_output_end} going to theta1{theta1_dxl_R}')
        print(f'motor2 at {motor2_output_end} going to theta1{theta1_dxl_L}')
        print(f'motor5 at {motor5_output_end} going to theta1{theta1_dxl_R}')

        # unitree.position_force_velocity_cmd(motor_number=1, torque=0, kp=0.05, kd=0, position = theta1_dxl_L*6.33)
        # unitree.position_force_velocity_cmd(motor_number=2, toropque=0, kp=0.05, kd=0, position = theta2_dxl_L*6.33)
        # unitree2.position_force_velocity_cmd(motor_number=4, torque=0, kp=0.05, kd=0, position = theta1_dxl_R*6.33)
        # unitree2.position_force_velocity_cmd(motor_number=5, torque=0, kp=0.05, kd=0, position = theta2_dxl_R*6.33)
        wheel_x+=dx
        wheel_y+=dy
        pass

def main():

    if input("cmd") == "start":   
        unitree.inital_all_motor()
        unitree2.inital_all_motor()
        print("inital_positon1:"+str(MOTOR1.inital_position/6.33*180/math.pi))
        print("inital_positon2:"+str(MOTOR2.inital_position/6.33*180/math.pi))
        print("inital_positon4:"+str(MOTOR4.inital_position/6.33*180/math.pi))
        print("inital_positon5:"+str(MOTOR5.inital_position/6.33*180/math.pi))
        try:
            lockleg()
            MOTOR1.inital_position=MOTOR1.inital_position-(0.7*6.33)
            MOTOR4.inital_position=MOTOR4.inital_position+(0.7*6.33)
            while True:
                updown = input("u or d")
                if updown == "u":
                    changeHeight(dx=0,dy=-0.01)
                elif updown == "d":
                    changeHeight(dx=0,dy=0.01)

        except KeyboardInterrupt:
            disableAllMotor()
        except Exception as e:
            print(str(e))
            traceback.print_exc()
            disableAllMotor()
    else:
        disableAllMotor()
    disableAllMotor()

    # unitree.inital_all_motor()
    # print("inital_positon1:"+str(MOTOR1.inital_position))
    # print("inital_positon2:"+str(MOTOR2.inital_position))
    # unitree.position_force_cmd(motor_number=1, torque=0, kp=20, kd=0, position= MOTOR1.data.q)

    # bottom = 3.5 
    # top = -6.38
    # angular_velocitgy = 50
    # step_size = angular_velocitgy/1000

    # _dir = 1

    # # 开始动态更新
    # start_time = time.time()
    # while True:
    #     # range top~bottom := -6.38~3 -> +offset := 0~9.38 -> 6.38
    #     while _dir == 1:
    #         unitree.position_force_cmd(motor_number=2, torque=0, kp=20, kd=0.16, position=current_position)
    #         if MOTOR2.data.q <= top:    
    #             _dir = 0
    #         time.sleep(0.0002)

    #     while _dir == 0:
    #         current_time = time.time()-start_time
    #         current_position = MOTOR2.data.q
    #         current_position += step_size
    #         unitree.position_force_cmd(motor_number=2, torque=0, kp=20, kd=0.16, position=current_position)
    #         if MOTOR2.data.q >= bottom:
    #             _dir = 1
    #         time.sleep(0.0002)

def profile_velocity_control():
    
    bottom = 6
    top = -6.38
    angular_velocity=math.pi/5
  
    plt.ion()
    fig,  (ax1,ax2,ax3) = plt.subplots(3,1, sharex=True)
    x_data, velocity_data, refvelocity_data, position_data , tau_data= [], [], [], [], []
    unitree.inital_all_motor()
    print("inital_positon1:"+str(MOTOR1.inital_position))
    print("inital_positon2:"+str(MOTOR2.inital_position))
    unitree.position_force_cmd(motor_number=1, torque=0, kp=0.8, kd=0, position= MOTOR1.inital_position)

    _dir = 1

    start_time = time.time()
    try:
        while True:
            
            while _dir == 1:

                current_time = time.time()-start_time
                unitree.angular_velocity_cmd(motor_number=2, kd = 0.1, velocity = -angular_velocity)
                x_data.append(current_time)
                velocity_data.append(MOTOR2.data.dq)
                refvelocity_data.append(MOTOR2.cmd.dq)
                position_data.append(MOTOR2.data.q)
                tau_data.append(MOTOR2.data.tau)
                
                # 更新数据
                
                if MOTOR2.data.q <= top:    
                    _dir = 0

                time.sleep(0.0001)

            while _dir == 0:
                
                current_time = time.time()-start_time
                unitree.angular_velocity_cmd(motor_number=2, kd = 0.1, velocity = angular_velocity)
                x_data.append(current_time)
                velocity_data.append(MOTOR2.data.dq*1.6)
                refvelocity_data.append(MOTOR2.cmd.dq)
                position_data.append(MOTOR2.data.q)
                tau_data.append(MOTOR2.data.tau)
                if MOTOR2.data.q >= bottom:
                    _dir = 1
        
                time.sleep(0.0001)
    except KeyboardInterrupt:
        
        unitree.position_force_cmd(motor_number=2, torque=0, kp=0, kd=0, position= 0)

if __name__ == "__main__":

    #profile_velocity_control()
    main()