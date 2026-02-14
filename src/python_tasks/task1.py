from robot_control_class import RobotControl
import math

def get_highest_lowest():

    rc = RobotControl()
    
    laser_readings = rc.get_laser_full()

    laser_dict = {}

    output = []

    for i, elem in enumerate(laser_readings):

        if not math.isinf(elem):
            laser_dict[i] = elem

    max_val = max(laser_dict.values())
    min_val = min(laser_dict.values())

    for key, val in laser_dict.items():  

        if val == max_val:
            output.append(key)

        if val == min_val:
            output.append(key)
            break

    return output

#max_index, min_index = get_highest_lowest()    #It has to be comment for correction.
#print(max_index, min_index)                    #It has to be comment for correction.
