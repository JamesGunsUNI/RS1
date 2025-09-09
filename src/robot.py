from navigation import NavNode

class Robot:
    def __init__(self, name='robot'):
        self.navigator = NavNode(name=f'{name}_navigator')
        self.homeXY = (0.0, 0.0, 0.0)

        # Register the post-goal callback to take a soil sample
        self.navigator.set_post_goal_callback(self.take_soil_sample)

    def move_to_goal(self, x, y, yaw=0.0):
        self.navigator.add_goal(x, y, yaw)

    def take_soil_sample(self):
        """Simulate taking a soil sample after reaching a goal"""
        self.navigator.get_logger().info("Taking soil sample...")
        moisture = 0.42  # placeholder
        self.navigator.get_logger().info(f"Soil moisture: {moisture}")
        return moisture

    def move_to_home(self):
        self.navigator.add_goal(*self.homeXY)


