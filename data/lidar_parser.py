import open3d
import math
import itertools
import numpy as np
import matplotlib.pyplot as plt

def parse_lidar(path):
    #curcloud = open3d.io.read_point_cloud("./1636570668.255584.pcd")
    curcloud = open3d.io.read_point_cloud(path)

    #turn off debug
    open3d.utility.set_verbosity_level(open3d.utility.VerbosityLevel.Error)

    # bounding box:
    bounds = [[0, 3], [-3, 3], [-0.3, 5]]
    box_section = list(itertools.product(*bounds))
    bounding_box = open3d.geometry.AxisAlignedBoundingBox.create_from_points(
    open3d.utility.Vector3dVector(box_section))
    curcloud = curcloud.crop(bounding_box)

    #stat_outlier
    cropped_cloud, in_pts = curcloud.remove_statistical_outlier(nb_neighbors=200, std_ratio=1.6)
    in_cld = curcloud.select_by_index(in_pts)
    out_cld = curcloud.select_by_index(in_pts, invert=True)
    in_cld.paint_uniform_color([0.8, 0, 0])
    out_cld.paint_uniform_color([0.3, 1, 1])

    pcd = in_cld

    with open3d.utility.VerbosityContextManager(
            open3d.utility.VerbosityLevel.Error) as cm:
        labels = np.array(
            pcd.cluster_dbscan(eps=0.4, min_points=50, print_progress=False))

    max_label = labels.max()
    clustered_point = [[] for i in range(max_label+1)]
    center = [[] for i in range(max_label+1)]

    for i, label_num in enumerate(labels):
        clustered_point[label_num].append(i)

    boxes = []
    obstacle_pts = []

    print(f"point cloud has {max_label + 1} clusters")
    # colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    # colors[labels < 0] = 0
    # pcd.colors = open3d.utility.Vector3dVector(colors[:, :3])
    # open3d.visualization.draw_geometries([pcd])

    for i, cur_cluster in enumerate(clustered_point):
        cur_cluster_pts = [pcd.points[pts] for pts in cur_cluster]
        cluster_vector = open3d.utility.Vector3dVector(cur_cluster_pts)
        bounding_box = open3d.geometry.AxisAlignedBoundingBox.create_from_points(cluster_vector)
        bounding_box.color = [1,0,0]
        boxes.append(bounding_box)

        center[i] = bounding_box.get_center().tolist()
        # verticies = bounding_box.get_box_points()[3:5]

        # numstep = math.sqrt((verticies[0][0] - verticies[1][0])**2 + (verticies[0][1] - verticies[1][1])**2) // 1

        # obsposlist = [center[i]]

        # if numstep == 0:
        #     obstacle_pts += obsposlist
        #     continue

        # xstep = (verticies[0][0] - verticies[1][0])/numstep
        # ystep = (verticies[0][1] - verticies[1][1])/numstep

        # xtemp = verticies[1][0]
        # ytemp = verticies[1][1]

        # obsposlist.append([xtemp, ytemp, center[i][2]])

        # for k in range(int(numstep)):
        #     xtemp += xstep
        #     ytemp += ystep
        #     obspos = [xtemp, ytemp, center[i][2]]
        #     obsposlist.append(obspos)

        # obstacle_pts += obsposlist
        # obsposlist = np.array(obsposlist)

        # tempcloud = open3d.geometry.PointCloud()
        # tempcloud.points = cluster_vector
        # obscloud = open3d.geometry.PointCloud()
        # obscloud.points=open3d.utility.Vector3dVector(obsposlist)
        # obscloud.paint_uniform_color([0.8, 0, 0])
        # open3d.visualization.draw_geometries([tempcloud, obscloud])

    # obsposlist = np.array(obstacle_pts)
    # obsposlist = np.array(center)
    # obscloud = open3d.geometry.PointCloud()
    # obscloud.points=open3d.utility.Vector3dVector(obsposlist)
    # obscloud.paint_uniform_color([0.2, 0, 0])
    # open3d.visualization.draw_geometries([obscloud] + boxes)

    return center

#parse_lidar("./1636570668.255584.pcd")