Pika Remote Piper Teleop 使用说明（流程与IK容错详解）

本说明面向 `pika_remote_piper` 包的遥操作（teleop）整套流程，涵盖启动入口、节点与话题、数据流、参数配置，以及重点说明 IK 无解与位姿不可达时的处理机制与阈值。启动切入点为 `launch/teleop_rand_single_piper.launch.py` 与 `launch/teleop_rand_multi_piper.launch.py`。

一、启动入口
- 单臂：`ros2 launch pika_remote_piper teleop_rand_single_piper.launch.py`
  - 包含：`piper/launch/start_single_piper.launch.py`
  - 启动：`piper_FK.py`、`piper_IK.py`、`teleop_piper_publish.py`
  - 参数：`paramsFile`（默认 `config/piper_rand_params.yaml`）
- 双臂：`ros2 launch pika_remote_piper teleop_rand_multi_piper.launch.py`
  - 包含：`piper/launch/start_double_piper.launch.py`
  - 启动：左臂与右臂各一套 `piper_FK.py`、`piper_IK.py`、`teleop_piper_publish.py`，`index_name` 分别为 `_l` 与 `_r`

二、核心节点与话题
1) teleop_piper_publish.py（遥操入口）
- 订阅：
  - `/pika_pose{index}`：定位（`PoseStamped`）
  - `/piper_FK{index}/urdf_end_pose_orient`：当前末端位姿（四元数）（`PoseStamped`）
- 发布：
  - `/piper_IK{index}/ctrl_end_pose`：控制目标末端位姿（`PoseStamped`）。当 `use_orient` 为假时，`orientation.x/y/z` 用欧拉角（roll/pitch/yaw），`orientation.w` 用作夹爪开合量；当为真时按标准四元数填入。
  - `/teleop_status{index}`：遥操作状态（`TeleopStatus`），`fail` 表示等待就绪，`quit` 表示退出
- 服务：
  - `/teleop_trigger{index}`（`std_srvs/Trigger`）：开关遥操。触发后等待定位与当前末端位姿各来一帧以对齐参考，再开始发布控制目标。

2) piper_FK.py（正解与末端位姿发布）
- 订阅：
  - `/joint_states_single{index}`：当前各关节角（`JointState`）
  - 可选 `/joint_states_single_lift`：升降轴（`JointState`），当 `--lift` 为真
- 发布：
  - `/piper_FK{index}/urdf_end_pose`：末端位姿（欧拉角）（`PoseStamped`）
  - `/piper_FK{index}/urdf_end_pose_orient`：末端位姿（四元数）（`PoseStamped`）

3) piper_IK.py（逆解、控制与容错）
- 订阅：
  - `/piper_IK{index}/ctrl_end_pose`：teleop 目标末端位姿（`PoseStamped`）
  - `/joint_states_single{index}`：当前各关节角（`JointState`），用于插值与约束
- 发布：
  - `/joint_states{index}`：控制输出关节角（`JointState`）
  - `/piper_IK{index}/urdf_end_pose_orient`：根据逆解计算得到的末端位姿（四元数）（`PoseStamped`）
  - 非四元数模式（`use_orient` 为假）还会发布：
    - `/piper_IK{index}/urdf_end_pose`（欧拉角形式）
    - `/piper_IK{index}/receive_end_pose_orient`（回显目标位姿的四元数形式）
  - `/arm_control_status{index}`：控制状态（`ArmControlStatus`），其中 `over_limit` 指示不可达/越限/失败
  - 可选 `/joint_states_lift`：升降轴控制输出，当 `--lift` 为真

三、参数配置
- `gripper_xyzrpy`：工具坐标／夹爪相对第6关节的固定变换，来自 `config/piper_rand_params.yaml`（默认 `[0.19, 0, 0, 0, 0, 0]`）
- `index_name`：区分左右臂或单臂（例如 `""`、`_l`、`_r`）
- `use_orient`：是否在控制消息中使用四元数作为目标姿态；若为假则使用欧拉角并复用 `orientation.w` 作为夹爪开合量
- `lift`：是否启用升降轴

四、端到端数据流
1) 遥操开启：调用 `/teleop_trigger{index}`，节点进入等待态，直到收到：
  - 一帧 `/pika_pose{index}`（定位）
  - 一帧 `/piper_FK{index}/urdf_end_pose_orient`（当前末端位姿）
2) 对齐与发布：每收到新的 `/pika_pose{index}`，teleop 计算目标末端位姿
  - 公式：`target = arm_end_pose_matrix * inv(localization_pose_matrix) * current_localization_matrix`
  - 发布到 `/piper_IK{index}/ctrl_end_pose`
3) 逆解计算：`piper_IK` 收到 `ctrl_end_pose` 后
  - 按 `use_orient` 决定用欧拉或四元数构造 `pin.SE3` 目标；欧拉模式下 `orientation.w` 作为夹爪量传入
  - 调用 `Arm_IK.ik_fun(target.homogeneous, gripper)` 求解
  - 若成功且通过容错校验（详见下节），缓存解与末端位姿供发布线程使用
4) 控制发布：发布线程以 50Hz
  - 对关节角作线性插值（避免一次跳变超过约 30°），随后发布 `/joint_states{index}`
  - 同步发布当前求解得到的末端位姿话题
  - 若 1 秒未收到新的有效 `ctrl_end_pose`，则暂停发布（安全超时）

五、IK 求解与容错
1) 求解器与模型（Arm_FIK.Arm_IK）
  - Pinocchio + Casadi + Ipopt
  - 约束：关节上下限（来自 URDF）
  - 成本：`20 * (位置误差权重1.0 + 姿态误差权重0.1) + 0.01 * 正则化`
  - 求解：`opti.solve_limited()`，`max_iter=50`，`tol=1e-4`
  - 初值：沿用上次解 `init_data`；若与历史解差异超 30° 则重置为零向量，以缓解局部极差或跳解
  - 末端帧：在第6关节添加 `ee` 帧，前乘固定姿态与 `gripper_xyzrpy`

2) 自碰与失败检测（一级）
  - 自碰检测：`pin.computeCollisions(...)`，若发生碰撞则返回失败（`get_result=False`）
  - 求解失败：Ipopt 抛异常时返回失败（`sol_q=None, get_result=False`）

3) 目标一致性校验（二级）
  - 在 `piper_IK.arm_end_pose_callback` 中，对成功的解进行前向校验：
    - 计算该解的 FK 位姿 `xyzrpy` 与目标的差异
    - 位置阈值：`|dx|, |dy|, |dz| > 0.3 m` 任一超限判失败
    - 姿态阈值：`|droll|, |dpitch|, |dyaw| > 1 rad` 任一超限判失败
  - 任何超阈值都认为目标不可达或越限（`get_result=False`）

4) 失败时的系统行为
  - 发布 `ArmControlStatus.over_limit=True` 到 `/arm_control_status{index}`
  - 不更新缓存解，不进入发布线程的控制输出（即不下发新的关节目标）
  - teleop 侧仍在运行，但 IK 不会推动机器人到不可达目标

六、插值与安全机制
- 线性插值：若本次目标与上次已下发角度差异超过约 30°，在 200Hz 下按 1° 步长插值逐步下发，避免关节瞬间大幅跳变
- 时效性：发布线程检测 `ctrl_end_pose` 消息时间戳，若超过 1 秒未更新则暂停输出，防止旧命令持续控制

七、使用建议与排错
- 选择姿态表示：常规 teleop 建议 `use_orient=False`（欧拉角 + 夹爪 `w`），若需精确姿态跟踪可设 `use_orient=True`
- 夹爪开合：在欧拉模式下通过 `orientation.w` 传递；在四元数模式下需改用其他通道（当前代码未复用 `w`）
- 检查 `joint_states_single{index}`：若 `piper_IK` 打印 `check joint_state topic`，说明未收到底层关节状态，需检查来源节点
- 关注 `ArmControlStatus`：当 `over_limit=True` 时，提示目标不可达/越限/发生碰撞

八、双臂差异
- 双臂启动在左右臂各自的 `index_name` 上镜像部署话题与节点，例如 `_l` 与 `_r`
- teleop/FK/IK 三者逻辑相同，仅区分话题名与底层来源

九、参数文件
- `config/piper_rand_params.yaml`：`gripper_xyzrpy: [0.19, 0.0, 0.0, 0.0, 0.0, 0.0]`
  - 用于定义工具坐标相对第6关节的偏置；如实际夹爪不同，可调整该参数以对齐末端坐标

十、快速上手
1) 单臂：
   - `ros2 launch pika_remote_piper teleop_rand_single_piper.launch.py`
   - 调用：`ros2 service call /teleop_trigger Trigger {}` 开启/关闭遥操
   - 观察：`/arm_control_status`、`/teleop_status`、`/joint_states`、`/piper_*` 话题
2) 双臂：
   - `ros2 launch pika_remote_piper teleop_rand_multi_piper.launch.py`
   - 分别调用左右臂 `/teleop_trigger_l`、`/teleop_trigger_r` 控制

十一、实现要点（代码定位）
- Launch：`launch/teleop_rand_single_piper.launch.py`、`launch/teleop_rand_multi_piper.launch.py`
- Teleop：`scripts/teleop_piper_publish.py`
- IK：`scripts/piper_IK.py`（核心逻辑）与 `scripts/arm_FIK.py`（求解器实现）
- FK：`scripts/piper_FK.py` 与 `scripts/arm_FIK.py`（正解）

十二、IK 无解/不可达的处理总结（要点归纳）
- 一次判定：求解异常或自碰 -> 视为失败
- 二次判定：解的 FK 与目标差异超过阈值（位置 0.3m、姿态 1rad）-> 视为失败
- 失败反馈：发布 `over_limit=True`，不下发新的关节目标（安全）
- 防跳变：大角度差异插值下发，避免瞬间跨越奇异附近造成超限
- 超时保护：超过 1 秒无新目标即暂停发布，避免旧命令持续控制

备注：`use_orient` 解析自命令行参数（类型为字符串），默认 `False`；当传入非空字符串（如 `True`）时即视为使用四元数模式。代码中 Meshcat 可视化用于求解调试，非必须。