# berry_dataset/berry_dataset/utils.py

import open3d as o3d
import cv2
import numpy as np

class CreatePointCloud:
    @staticmethod
    def from_images(color_image, depth_image, camera_info):
        if len(color_image.shape) == 3 and color_image.shape[2] == 3:
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        elif len(color_image.shape) == 2:
            color_image = cv2.cvtColor(color_image, cv2.COLOR_GRAY2RGB)
        else:
            raise ValueError("Invalid color image shape")

        o3d_color = o3d.geometry.Image(color_image.astype(np.uint8))
        o3d_depth = o3d.geometry.Image(depth_image.astype(np.uint16))

        if camera_info is None:
            raise ValueError("depth_intrin must be provided")

        print(f'Camera info in utils {camera_info}')

        o3d_intrin = o3d.camera.PinholeCameraIntrinsic(
            width=camera_info.width,
            height=camera_info.height,
            fx=camera_info.k[0],
            fy=camera_info.k[4],
            cx=camera_info.k[2],
            cy=camera_info.k[5]
        )

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d_color, o3d_depth,
            depth_scale=1000.0,
            depth_trunc=3.0,
            convert_rgb_to_intensity=False
        )

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d_intrin)
        return pcd