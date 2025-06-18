
# berry_dataset/berry_dataset/berry_dataset.py


import pandas as pd
import os
import cv2
import open3d as o3d 
import numpy as np


class BerryDataset:
    
    def __init__(self, data_root, dataset_name = 'img_ply_dataset.csv'):
        
        # Base root to store the dataset information
        self.base_root = data_root
        self.dataset_name = dataset_name
               
        # Derectories for pointcloud and images
        self.img_dir = os.path.join(self.base_root, 'images')
        self.ply_dir = os.path.join(self.base_root, 'clouds')
        self.depth_dir = os.path.join(self.base_root, 'depth_frame')
        self.depth_IMG_dir = os.path.join(self.base_root, 'depth_image')
        
        # Create the directories and check whether they exist or not
        os.makedirs(self.img_dir, exist_ok=True)
        os.makedirs(self.ply_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)
        os.makedirs(self.depth_IMG_dir, exist_ok=True)
        
        # Check if the dataset does exist or not
        if self.dataset_name in os.listdir(self.base_root):
            self.dataset = pd.read_csv(os.path.join(self.base_root, self.dataset_name))
            self.idx = self.dataset['berry_id'].max() + 1 if not self.dataset.empty else 0
                    
        else: # If the dataset does not exist: create it
            columns = ['arm_T_matrix',    # T matrix of the last joint of the arm
                        'berry_T_matrix', # T matrix of the tip of the blackberry
                        'berry_id',       # Berry progressive id --> also in the name of the saved files
                        'camera_intrin'] # Camera intrinsic parameters       
            self.dataset = pd.DataFrame(columns=columns)
            self.idx = 0

    def save_data(self, arm_pose, berry_pose, camera_intrin=None):
        
        new_data = {   
                'arm_T_matrix': [arm_pose],   
                'berry_T_matrix' : [berry_pose],
                'berry_id' : [self.idx],
                'camera_intrin' : [camera_intrin] 
            }
        
        new_data = pd.DataFrame(new_data, index=[0])
        self.dataset = pd.concat([self.dataset, new_data], ignore_index=True)

        self.save_dataset()

    def save_dataset(self):
        # Save the dataset to a CSV file
        save_path = os.path.join(self.base_root, self.dataset_name)
        self.dataset.to_csv(save_path, index=False)

    def save_image(self, image):
        img_filename = f'berry_image_{self.idx}.png'
        # Save the image
        cv2.imwrite(os.path.join(self.img_dir, img_filename), image)
    
    def save_depth_image(self, depth_image):
        depth_filename = f'depth_image_{self.idx}.png'
        # Save the depth image
        cv2.imwrite(os.path.join(self.depth_IMG_dir, depth_filename), depth_image)

    def save_pointcloud(self, point_cloud):    
        ply_filename = f'berry_cloud_{self.idx}.ply'     
        # Save point cloud using Open3D
        o3d.io.write_point_cloud(os.path.join(self.ply_dir, ply_filename), point_cloud)
    
    def save_depth_frame(self, depth_frame):
        depth_filename = f'depth_frame_{self.idx}.npy'
        np.save(depth_filename, depth_frame)
