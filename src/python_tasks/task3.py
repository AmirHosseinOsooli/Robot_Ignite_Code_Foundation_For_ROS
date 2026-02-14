from robot_control_class import RobotControl

class ExamControl():

    def __init__(self):
        self.robotcontrol = RobotControl()
        
    def get_laser_readings(self):
        left = self.robotcontrol.get_laser(719)
        right = self.robotcontrol.get_laser(0) 
        return left, right

    def main(self):  
        while True:
            self.robotcontrol.move_straight()
            left, right = self.get_laser_readings()
            if left == float('inf') and right == float('inf'):      
                self.robotcontrol.stop_robot() 
                break  

#mv = ExamControl()    #It has to be comment for correction.
#mv.main()             #It has to be comment for correction.
