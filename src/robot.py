from navigation import NavNode

class Robot():

    def __init__(self):
        self.navigator = NavNode()
        self.homeXY = [0,0,0] #how to get the robots current location

    def move_to_goal(self, x, y):
        self.navigator.send_goal(float(x), y)
    
    def take_soil_sample(self):
        '''
        fucntion that triggers the colleciton of a soil sample

        returns a float value of the moister level
        
        '''
        raise NotImplementedError("INTERFACE DATA COLLECTION INTO HERE")

    def move_to_home(self):
        self.navigator.send_goal(self.homeXY[0], self.homeXY[1])

        
