# 状态机双无人机模式 - 快速参考

## 🚀 快速启动

### 启动双无人机状态机（仅状态机）
```bash
roslaunch state_machine start_dual_uav_state_machine.launch
```

### 启动完整的双无人机系统（包括状态机）
```bash
roslaunch uav_control start_dual_uav.launch
```

### 单独启动某个无人机的状态机
```bash
# UAV1
roslaunch state_machine jy_state_machine_sim.launch uav_name:=uav1

# UAV2
roslaunch state_machine jy_state_machine_sim.launch uav_name:=uav2
```

---

## 📊 主题速查表

| 功能 | UAV1 | UAV2 |
|------|------|------|
| 状态机状态 | `/uav1/state_machine` | `/uav2/state_machine` |
| 目标点 | `/uav1/goal` | `/uav2/goal` |
| MPC轨迹 | `/uav1/mpc_trajectory` | `/uav2/mpc_trajectory` |
| 停止超级避障 | `/uav1/stop_super` | `/uav2/stop_super` |
| 静态点云 | `/uav1/filtered_static_pcd` | `/uav2/filtered_static_pcd` |
| 动态点云 | `/uav1/filtered_dynamic_pcd` | `/uav2/filtered_dynamic_pcd` |

---

## ✅ 验证系统

```bash
# 查看所有状态机节点
rosnode list | grep state_machine

# 查看所有状态机相关主题
rostopic list | grep state_machine

# 监听 UAV1 的状态机状态
rostopic echo /uav1/state_machine

# 查看计算图
rqt_graph
```

---

## 📁 文件位置

| 文件 | 位置 |
|------|------|
| 状态机launch | `src/state_machine/launch/jy_state_machine_sim.launch` |
| 双无人机launch | `src/state_machine/launch/start_dual_uav_state_machine.launch` |
| 状态机脚本 | `src/state_machine/scripts/jy_state_machine.py` |
| 修改说明 | `src/state_machine/launch/STATE_MACHINE_DUAL_UAV.md` |

---

## 🔧 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `uav_name` | `uav1` | 无人机标识和命名空间 |
| `real_fly` | `false` | 是否使用实机参数 |

---

## 🎯 一键启动完整系统

```bash
# 这个命令会启动所有模块（包括状态机）
roslaunch uav_control start_dual_uav.launch
```

启动的内容：
- ✅ FAST_LIO LIDAR处理 (UAV1 + UAV2)
- ✅ 状态机 (UAV1 + UAV2)
- ✅ MPC控制 (UAV1 + UAV2)
- ✅ 运动规划 (UAV1 + UAV2)

---

## 💡 常见操作

### 发送目标点给状态机
```bash
# 发送目标点给 UAV1
rostopic pub /uav1/goal geometry_msgs/PoseStamped \
'{header: {stamp: now, frame_id: "world"}, \
pose: {position: {x: 5.0, y: 5.0, z: 1.5}, orientation: {w: 1.0}}}'
```

### 监听状态机的轨迹输出
```bash
rostopic echo /uav1/mpc_trajectory
```

### 查看状态机的点云处理结果
```bash
# 静态点云
rostopic echo /uav1/filtered_static_pcd

# 动态点云
rostopic echo /uav1/filtered_dynamic_pcd
```

---

**详细说明见**: [STATE_MACHINE_DUAL_UAV.md](STATE_MACHINE_DUAL_UAV.md)
