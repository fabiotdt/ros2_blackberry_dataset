import numpy as np
from scipy.spatial.transform import Rotation as R

class CollisionFreeGridGenerator:
    def __init__(self, filename='pose_grid.npy'):

        self.filename=filename

        # Workspace bounds (in meters)
        self.xmin, self.xmax = -0.350, 0.350
        self.ymin, self.ymax =  0.400, 0.600
        self.zmin, self.zmax =  0.100, 0.500

        # Discretization steps
        self.grid_step = 0.05  # 5 cm grid spacing

        # Orientation combinations (in degrees)
        roll_disc = [-45.0, 0.0, 45.0]
        pitch_disc = [-45.0, 0.0, 45.0]
        yaw = -90.0

        # Generate meshgrid of roll/pitch and add yaw
        self.rotation_combinations = np.array(np.meshgrid(roll_disc, pitch_disc)).T.reshape(-1, 2)
        self.rotation_combinations = np.hstack((
            self.rotation_combinations[:, 0:1] + yaw,  # yaw + roll
            np.zeros((self.rotation_combinations.shape[0], 1)),  # dummy Y for conversion
            self.rotation_combinations[:, 1:2]  # pitch
        ))

    def generate_cartesian_grid(self):
        x_vals = np.arange(self.xmin, self.xmax + self.grid_step, self.grid_step)
        y_vals = np.arange(self.ymin, self.ymax + self.grid_step, self.grid_step)
        z_vals = np.arange(self.zmin, self.zmax + self.grid_step, self.grid_step)

        cartesian_points = np.array(np.meshgrid(x_vals, y_vals, z_vals)).T.reshape(-1, 3)
        return cartesian_points

    def generate_pose_grid(self):
        poses = []
        positions = self.generate_cartesian_grid()
        for pos in positions:
            for rpy in self.rotation_combinations:
                rot = R.from_euler('xyz', np.deg2rad(rpy))
                pose = {'position': pos, 'orientation': rot.as_matrix(), 'completed': False, 'is_safe': None}
                poses.append(pose)
        return poses
    
    def save_pose_grid(self):
        poses = self.generate_pose_grid()
        #np.save(filename, poses)
        print(f"Pose grid saved to {self.filename}")

        print(f"Pose grid NOT saved for testing purposes")

    def load_pose_grid(self, filename='pose_grid.npy'):
        poses = np.load(filename, allow_pickle=True)
        print(f"Pose grid loaded from {filename}")
        return poses
    
    def modify_pose_grid(self, poses, el):
        # Example modification: Mark every 10th pose as 'completed'

        for i in el:
            poses[i]['completed'] = True
            poses[i]['is_safe'] = False

            np.save(self.filename, poses)
  
# Example usage
if __name__ == "__main__":
    generator = CollisionFreeGridGenerator()
    
    #generator.save_pose_grid()  # Save the generated pose grid to a file
    
    grid = generator.load_pose_grid()  # Load the pose grid from the file

    #print(f'The last pose in the grid is:\n{grid[486]}')  # Print the last pose for verification

    modify_list = [1437]

    generator.modify_pose_grid(grid, modify_list)  # Modify the pose grid based on the list

    print(f'Overall poses: {len(grid)}')