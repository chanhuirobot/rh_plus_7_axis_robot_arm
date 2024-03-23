import numpy as np
from pypcd import pypcd

pcd_data = pypcd.PointCloud.from_path('point_cloud_1.pcd')

width = pcd_data.width
if width % 2 != 0:
	width -= 1

points = np.zeros([width, 3],dtype=np.float32)
points[:, 0] = pcd_data.pc_data['x'][:width]
points[:, 1] = pcd_data.pc_data['y'][:width]
points[:, 2] = pcd_data.pc_data['z'][:width]

with open('point_cloud_1.bin', 'wb') as f:
    f.write(points.tobytes())

