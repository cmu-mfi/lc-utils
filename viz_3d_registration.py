import numpy as np
import open3d as o3d
import quaternion
import copy

THRESHOLD = 50

def get_cloud(lc_images):
    intensity_img = lc_images[:, :, :, -1] # (N, 640, 512)
    intensity_img[np.isnan(intensity_img)] = 0.0 # (N, 640, 512)
    max_intensity_idxs = np.argmax(intensity_img, axis=0) # (640, 512)

    intensity_img = np.max(intensity_img, axis=0) # (640, 512)
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

    filter_idxs = filtered_pts[:, 2] <= 7.0
    filtered_pts = filtered_pts[filter_idxs]
    filter_idxs = filtered_pts[:, 0] <= 3.5
    filtered_pts = filtered_pts[filter_idxs]
    filter_idxs = filtered_pts[:, 0] >= -1.5
    filtered_pts = filtered_pts[filter_idxs]

    return filtered_pts


def draw_registration_result(pc1, pc2, transformation):
    pc1_temp = copy.deepcopy(pc1)
    pc2_temp = copy.deepcopy(pc2)
    pc2_temp.paint_uniform_color([0.929, 0.151, 0])
    pc1_temp.paint_uniform_color([0, 0.651, 0.929])
    pc1_temp.transform(transformation)
    
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='Open3D', width=800, height=600)

    opt = vis.get_render_option()
    opt.background_color = np.asarray([0, 0, 0])

    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

    vis.add_geometry(pc1_temp)
    vis.add_geometry(pc2_temp)
    vis.add_geometry(coordinate_frame)

    vis.run()

    pc = o3d.geometry.PointCloud()
    pc = pc1_temp + pc2_temp
    o3d.io.write_point_cloud("copy.ply", pc)

def draw_cloud(pc1):
    o3d.visualization.draw_geometries([pc1],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])

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

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 5
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result

# Load the point clouds from the .npy files
#point_cloud1 = get_cloud(np.load('./data/sweep4/lc_images_blue.npy'))
#point_cloud2 = get_cloud(np.load('./data/sweep4/lc_images_red.npy'))

point_cloud1 = o3d.io.read_point_cloud('cloud_blue.ply')
point_cloud2 = o3d.io.read_point_cloud('cloud_red.ply')

point_cloud1 = np.asarray(point_cloud1.points)
point_cloud2 = np.asarray(point_cloud2.points)

filter_idxs = point_cloud1[:, 2] <= 3.5
point_cloud1 = point_cloud1[filter_idxs]

# Define a rigid transformation matrix (translation + rotation)
R1 = np.array([[-1, 0, 0],
             [0, 1, 0],
             [0, 0, -1]])

R2 = np.array([[1, 0, 0],
               [0, np.cos(-np.pi/2), -np.sin(-np.pi/2)],
               [0, np.sin(-np.pi/2), np.cos(-np.pi/2)]])

R = R1 @ R2
t = np.array([0, -6.5, 7])
T = np.hstack((R, t.reshape(-1,1)))
T = np.vstack((T, np.array([0, 0, 0, 1])))

T2 = np.array([[ 0.96604815, 0.25382649, -0.0481985, 0.70905648],
 [-0.25100912, 0.9662679, 0.05762603, -2.08822262],
 [ 0.06119967, -0.04357125, 0.99717408, 2.87394201],
 [ 0., 0., 0., 1.]])

T = np.linalg.inv(T2) @ T

print("T_blue_red:", T)

point_cloud2 = np.dot(T, np.vstack((point_cloud2.T, np.ones(point_cloud2.shape[0])))).T[:,:-1]

#condition = np.logical_not(np.logical_and(point_cloud2[:, 2] > 4, point_cloud2[:, 1] > 0.3))
#point_cloud2 = point_cloud2[condition]

# Create Open3D point cloud objects
pc1 = o3d.geometry.PointCloud()
pc2 = o3d.geometry.PointCloud()
pc1.points = o3d.utility.Vector3dVector(point_cloud1)
pc2.points = o3d.utility.Vector3dVector(point_cloud2)

pc1 = pc1.voxel_down_sample(voxel_size=0.02)
pc2 = pc2.voxel_down_sample(voxel_size=0.02)

pc1, _ = pc1.remove_radius_outlier(nb_points=16, radius=0.05)
pc2, _ = pc2.remove_radius_outlier(nb_points=16, radius=0.05)

#draw_cloud(pc1)
#draw_cloud(pc2)

pc1_ds, pc1_fpfh = preprocess_point_cloud(pc1, 0.05)
pc2_ds, pc2_fpfh = preprocess_point_cloud(pc2, 0.05)
#draw_registration_result(pc1_ds, pc2_ds, np.eye(4))

###### ICP Registration #####
reg_p2p = o3d.pipelines.registration.registration_icp(
    pc1_ds, pc2_ds, 0.6, np.eye(4),
    o3d.pipelines.registration.TransformationEstimationPointToPlane())
#print('Registration done!\n', reg_p2p.transformation)
#draw_registration_result(pc1_ds, pc2_ds, reg_p2p.transformation)

reg_p2p = o3d.pipelines.registration.registration_icp(
    pc1, pc2, 0.8, reg_p2p.transformation,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
print('Registration done!\n', reg_p2p)

point_cloud2 = np.asarray(pc2.points)
filter_idxs = point_cloud2[:, 2] >= 3.3
point_cloud2 = point_cloud2[filter_idxs]

pc2 = o3d.geometry.PointCloud()
pc2.points = o3d.utility.Vector3dVector(point_cloud2)

pc1_ds, pc1_fpfh = preprocess_point_cloud(pc1, 0.05)
pc2_ds, pc2_fpfh = preprocess_point_cloud(pc2, 0.05)

draw_registration_result(pc1_ds, pc2_ds, reg_p2p.transformation)

##### Fast Global Registration #####
#result_fast = execute_fast_global_registration(pc1_ds, pc2_ds, pc1_fpfh, pc2_fpfh, 0.05)
#print('Registration done!\n', result_fast.transformation)
#draw_registration_result(pc1_ds, pc2_ds, result_fast.transformation)

##### Global Registration #####
#result_ransac = execute_global_registration(pc1_ds, pc2_ds, pc1_fpfh, pc2_fpfh, 0.05)
#print('Registration done!\n', result_ransac.transformation)
#draw_registration_result(pc1_ds, pc2_ds, result_ransac.transformation)
