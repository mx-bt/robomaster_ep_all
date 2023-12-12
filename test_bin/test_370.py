import numpy as np
import pandas as pd

ail = np.empty((0, 6), dtype=int) # all information list
t_v = np.array([])
x_list, y_list = np.array([]),np.array([])

# Subscribe to chassis location information

def sub_position_handler(position_info):
    global x_list
    global y_list
    global t_v

    x, y, z = position_info
    x, y, z = round(x,2), round(y,2), round(z,2)

    t_v = np.array([])
    t_v = np.append(t_v,[x,y,z])
    # print(f"chassis position: x:{x}, y:{y}, z:{z}")

def sub_attitude_info_handler(attitude_info):

    global ail
    global t_v

    yaw, pitch, roll = attitude_info

   
    yaw, pitch, roll = round(yaw,2), round(pitch,2), round(roll,2)
    t_v = np.append(t_v,[yaw, pitch, roll])
    
    print(ail[:, 0])
    print(ail[:, 1])

    ail = np.append(ail, [t_v], axis=0)


position_info1 = (1.213,2.12355,3.76334)
attitude_info1 = (11.23467,22.3423223,33.2132122)
sub_position_handler(position_info1)
sub_attitude_info_handler(attitude_info1)

position_info2 = (4,5,6)
attitude_info2 = (44,55,66)
sub_position_handler(position_info2)
sub_attitude_info_handler(attitude_info2)

position_info3 = (7,8,9)
attitude_info3 = (77,88,99)
sub_position_handler(position_info3)
sub_attitude_info_handler(attitude_info3)

main_df = pd.DataFrame(ail, columns = ['x', 'y', 'z', 'yaw', 'pitch', 'roll'])
print(main_df)
main_df.to_csv('test_pos_data.csv', index=False)

