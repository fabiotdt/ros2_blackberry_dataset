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

    def split_and_save_filtered_poses(self, dx_filename='dx_poses.npy', sx_filename='sx_poses.npy'):

        eliminated_ids = [
            8, 278, 325, 324, 331, 332, 341, 342, 368, 385,
            393, 412, 413, 438, 447, 457, 458, 459, 466,
            476, 485, 486, 487, 488, 591, 636, 676, 677,
            953, 998, 1006,
            1312, 1313, 1319, 1321, 1322, 1328, 1331, 1337,
            1351, 1352, 1357, 1359, 1362, 1365, 1366, 1372,
            1375, 1376, 1380, 1384, 1390, 1392, 1398, 1401,
            1402, 1407, 1410, 1417, 1419, 1425, 1428, 1435,
            1437, 1673
        ]
        
        poses = self.load_pose_grid(self.filename)

        # Filter completed and safe poses
        completed_and_safe = [
            pose for idx, pose in enumerate(poses)
            if pose['completed'] is True and idx not in eliminated_ids
        ]
        
        print(f'the number of completed poses is: {len(completed_and_safe)}')

        # Separate into dx (x > 0) and sx (x < 0)
        dx_poses = [pose for pose in completed_and_safe if pose['position'][0] > 0]
        sx_poses = [pose for pose in completed_and_safe if pose['position'][0] < 0]

        dx_poses = self.reset_completed_flag(dx_poses)
        sx_poses = self.reset_completed_flag(sx_poses)

        # Save them to specified filenames
        np.save(dx_filename, dx_poses)
        np.save(sx_filename, sx_poses)

        print(f"Saved {len(dx_poses)} dx poses to {dx_filename}")
        print(f"Saved {len(sx_poses)} sx poses to {sx_filename}")

    def reset_completed_flag(self, poses, save=False):

        for pose in poses:
            pose['completed'] = False

        if save:
            np.save(self.filename, poses)
            print(f"All 'completed' flags set to False and saved to {self.filename}")
        else:
            print("All 'completed' flags set to False (not saved)")

        return poses
        
  
# Example usage
if __name__ == "__main__":
    generator = CollisionFreeGridGenerator()
    
    #generator.save_pose_grid()  # Save the generated pose grid to a file
    
    #grid = generator.load_pose_grid()  # Load the pose grid from the file

    #print(f'The last pose in the grid is:\n{grid[486]}')  # Print the last pose for verification

    #modify_list = [1673]

    #generator.modify_pose_grid(grid, modify_list)  # Modify the pose grid based on the list

    generator.split_and_save_filtered_poses()

    #print(f'Overall poses: {len(grid)}')




