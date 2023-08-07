import numpy as np
import open3d as o3d
import quaternion
import copy
import cv2

THRESHOLD = 50

def get_cloud(lc_images):
    intensity_img = lc_images[:, :, :, -1] # (N, 640, 512)
    intensity_img[np.isnan(intensity_img)] = 0.0 # (N, 640, 512)
    max_intensity_idxs = np.argmax(intensity_img, axis=0) # (640, 512)

    intensity_img = np.max(intensity_img, axis=0) # (640, 512)
    #cv2.imwrite('/home/mfi/scan2.png', intensity_img)
    filter_idxs = intensity_img >= THRESHOLD # gives me indices where intensity is greater than threshold
    
    H, W = intensity_img.shape
    pts = np.zeros((H, W, 3)) # (640, 512, 3)
    for y in range(H):
        for x in range(W):
            pts[y,x] = lc_images[max_intensity_idxs[y,x], y, x, :-1]

    # make the depth images, depth is zero for points which have intensity less than threshold
    depth_im = pts[:, :, 2] # (640, 512)
    #depth_im[intensity_img < THRESHOLD] = 0

    filtered_pts = pts[filter_idxs] # (N, 3)

    # finally remove the points which have z < 0.3
    #filter_idxs = filtered_pts[:, 2] >= 0.3
    #filtered_pts = filtered_pts[filter_idxs] 
    filter_idxs = filtered_pts[:, 2] <= 7.0
    filtered_pts = filtered_pts[filter_idxs]
    filter_idxs = filtered_pts[:, 0] <= 4.5
    filtered_pts = filtered_pts[filter_idxs]
    filter_idxs = filtered_pts[:, 0] >= -1.5
    filtered_pts = filtered_pts[filter_idxs]
    #filter_idxs = filtered_pts[:, 1] <= 2.0
    #filtered_pts = filtered_pts[filter_idxs]
    #filtered_pts = filtered_pts[:, 1] <=6
    return filtered_pts
 

def draw_cloud(pc1):
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    o3d.visualization.draw_geometries([pc1, coordinate_frame],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])

    o3d.io.write_point_cloud("cloud.ply", pc1)


def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

# Load the point clouds from the .npy files
point_cloud1 = get_cloud(np.load('./data/lc_images_red.npy'))

# Create Open3D point cloud objects
pc1 = o3d.geometry.PointCloud()
pc1.points = o3d.utility.Vector3dVector(point_cloud1)

draw_cloud(pc1)

#pc1 = pc1.voxel_down_sample(voxel_size=0.02)
#pc1, _ = pc1.remove_radius_outlier(nb_points=16, radius=0.05)

#pc1_ds, pc1_fpfh = preprocess_point_cloud(pc1, 0.05)

