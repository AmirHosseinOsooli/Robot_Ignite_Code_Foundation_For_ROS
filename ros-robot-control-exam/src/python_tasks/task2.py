from robot_control_class import RobotControl

rc = RobotControl()

get_laser = rc.get_laser(360)

while get_laser > 1:

    rc.move_straight()

    get_laser = rc.get_laser(360)

    if get_laser <= 1:

        rc.stop_robot()

rc.turn('clockwise', 0.5, 3.1415926)
    
