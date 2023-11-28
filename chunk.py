import numpy as np
import open3d as o3d
def split_bbox_to_eight_blocks(bbox):
    min_bound = np.asarray(bbox.min_bound)
    max_bound = np.asarray(bbox.max_bound)
    center = (min_bound + max_bound) / 2.0

    points = np.asarray(point_cloud.points)

    vertices = [
        (min_bound, center),
        ([center[0], min_bound[1], min_bound[2]], [max_bound[0], center[1], center[2]]),
        ([center[0], min_bound[1], center[2]], [max_bound[0], center[1], max_bound[2]]),
        ([min_bound[0], min_bound[1], center[2]], [center[0], center[1], max_bound[2]]),
        ([min_bound[0], center[1], min_bound[2]], [center[0], max_bound[1], center[2]]),
        ([center[0], center[1], min_bound[2]], [max_bound[0], max_bound[1], center[2]]),
        ([center[0], center[1], center[2]], max_bound),
        ([min_bound[0], center[1], center[2]], [center[0], max_bound[1], max_bound[2]])
    ]

    # 创建八个块
    blocks = []
    for v1, v2 in vertices:
        block = o3d.geometry.PointCloud()
        indices = np.all(np.logical_and(v1 <= points, points <= v2), axis=1)
        block.points = o3d.utility.Vector3dVector(points[indices])
        blocks.append(block)

    return blocks

# 示例用法
if __name__ == "__main__":
    # 读取点云数据
    point_cloud = o3d.io.read_point_cloud("C:\\Users\\34066\\Desktop\\GZPI.ply")

    # 计算点云的 bounding box
    bbox = point_cloud.get_axis_aligned_bounding_box()

    # 分割 bounding box 为八块
    blocks = split_bbox_to_eight_blocks(bbox)

    # 可以在此处对每个块执行需要的操作，例如保存到文件
    for i, block in enumerate(blocks):
        o3d.io.write_point_cloud(f"C:\\Users\\34066\\Desktop\\block_{i}.ply", block)
        print("Successfully write block{}".format(i))
