import numpy as np
from scipy.spatial.transform import Rotation as R



# 初始位姿
initial_pose = {
    "position": {"x": 264.47128897902326, "y": 397.3294812893412, "z": -0.29895997047424316},
    "orientation": {"x": 0.0, "y": 0.0, "z":  -0.6222372744994262, "w": 0.7828287004342174}
}

# 路径点
path_points = {
    'p1': [455.0, 155.12, -128.61],
    'p2': [460.98, 160.62, -120.24],
    'p3': [468.97, 167.89, -120.26],
    'p4': [474.86, 173.24, -127.95],
}
# 初始四元数
q_initial = R.from_quat([
    initial_pose["orientation"]["x"],
    initial_pose["orientation"]["y"],
    initial_pose["orientation"]["z"],
    initial_pose["orientation"]["w"]
])

# 初始位置
initial_position = np.array([
    initial_pose["position"]["x"],
    initial_pose["position"]["y"],
    initial_pose["position"]["z"]
])

# 转化为局部坐标系
local_coordinates = {}
for key, point in path_points.items():
    point_global = np.array(point)
    # offset = point_global - initial_position
    # corrected_offset = q_initial.inv().apply(offset)  # 四元数逆旋转

    
    offset = q_initial.apply(point_global)  # 四元数逆旋转
    corrected_offset = offset + initial_position
    point = [round(corrected_offset[0], 2), round(corrected_offset[1], 2), round(corrected_offset[2], 2)]
    local_coordinates[key] = point

# 输出局部坐标系下的路径点
for key, local_point in local_coordinates.items():
    print(f"{key}: {local_point}")
