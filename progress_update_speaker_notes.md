# Progress update 逐字稿

## 第 1 页：标题页

各位老师、同学好，我今天汇报的是最近一段时间 ROS2 Humble、MoveIt2 和 UR 机械臂仿真平台上的混合避障算法迁移进展。

我的核心工作不是重新设计 MATLAB 里的算法，而是把原来在 MATLAB 中验证过的混合规划思路，迁移到真实机械臂软件链路里。这里面包括名义轨迹生成、局部 RRT 或 RRT-Connect 避障、FMP 调制、碰撞检查、轨迹下发和实验结果评估。

所以今天我主要按三个部分讲：第一，原有架构和它的问题；第二，我最近做了哪些工程改造；第三，我阅读 pRRTC 论文后，对后续混合算法优化有哪些启发。

## 第 2 页：实验现状与总体判断

目前项目已经从最开始的算法脚本迁移，推进到 ROS2 离线规划主链路。现在系统可以完成名义轨迹输入、危险段检测、局部路径生成、FMP 调制、轨迹下发和结果导出。

我现在对实验现状的判断是：算法链路和评估框架已经基本跑通，当前主要问题集中在仿真后端一致性和真实 collision backend 的稳定打通上。也就是说，现在不是单纯某个 RRT 参数的问题，而是 MoveIt PlanningScene、Gazebo 控制器、RViz 可视化、轨迹执行这些模块需要完全对齐。

所以我最近的工作重点不是盲目调参，而是先把系统变成可复核、可定位、可量化的结构。这样后面每一次实验才知道到底是算法失败、碰撞后端没接上，还是控制器执行失败。

## 第 3 页：原有架构

这一页是原来的系统架构。

最开始的结构比较集中，主要由 Python 主节点完成所有事情。输入是一条关节空间名义轨迹，然后 Python 节点内部做危险检测、RRT 局部避障、FMP 调制，最后直接把轨迹发给 FollowJointTrajectory 控制器。

碰撞检查这部分原来主要依赖 moveit_py 或者 analytic 简化碰撞模型。最后 Gazebo 和 RViz 负责显示仿真动画和轨迹可视化。

这个架构的好处是简单，开发快。但是当我们从 MATLAB 算法验证走向真实机械臂仿真和后续实机迁移时，它的问题就比较明显了。

## 第 4 页：原有架构存在的问题

原有架构第一个问题是安全正确性不足。早期更关注轨迹有没有发出去，但是对于轨迹下发前是否整条路径无碰撞，检查还不够严格。尤其是 FMP 调制后，轨迹形状会发生变化，如果没有 motion-level 的 states 和 edges 后检，就可能出现调制后轨迹不安全但仍然进入执行链路的问题。

第二个问题是平台迁移复杂。MATLAB 里我们更容易直接看二维或三维轨迹，但 ROS2 控制器最终接收的是关节轨迹。如果想用末端平面轨迹，就必须经过 IK 映射到关节空间，而 IK 解的连续性又会影响 FMP 的平滑效果。

第三个问题是调参难度高。障碍物尺寸、RRT 拓展步长、edge resolution、via 插值密度是耦合的。一个参数看起来只是改变采样密度，实际上可能同时影响绕障效果、轨迹抖动和控制器可执行性。

所以这次工作的目标不是只让某一次动画看起来绕过去，而是把整个实验系统做成可以长期调试和复现实验的工程平台。

## 第 5 页：当前最新架构

这是现在的系统架构。

输入层保留两种模式：关节空间名义轨迹和末端平面轨迹。末端平面轨迹通过 IK 转换为关节空间名义轨迹，然后进入统一的离线规划流程。

Python 主节点现在主要负责离线编排，包括全局危险扫描、碰撞段分段、FMP 调制、benchmark 记录和 evaluator 输入导出。也就是说 Python 侧保留算法主流程和 MATLAB 对齐逻辑。

运行时关键能力下沉到了 C++ runtime bridge。C++ 侧通过 MoveIt2 PlanningScene 做 CheckMotionBatch，检查 states 和 edges；通过 PlanLocalSegment 做局部 RRT-Connect；也负责轨迹下发和 marker 发布。

在执行前增加了 Safety Gate，也就是 FMP 后严格 post-check、起点一致性检查、关节边界和速度加速度估计。

最后新增 evaluator，用于输出绕障成功性、平滑性、可执行性和局部路径复核指标。

## 第 6 页：工作重点 1：C++ Runtime Bridge

第一项重点工作是 C++ Runtime Bridge。

这么做的原因是 Humble 环境下 moveit_py 不一定稳定可用，而真实机械臂仿真不能长期依赖 analytic 球体碰撞模型。碰撞检查和轨迹下发都属于运行时关键路径，所以更适合用 C++ MoveIt2 接口完成。

具体改动包括新增 CheckStatesBatch 和 CheckMotionBatch 服务。其中 CheckMotionBatch 不只检查离散状态点，也检查相邻点之间的 edge，这是非常关键的。

另外我新增了 PlanLocalSegment 服务，在 C++ 侧提供局部 RRT-Connect 规划。Python 主流程还保留 fallback，这样在开发阶段可以做对照和回退，不会因为某一个后端问题导致整个系统无法调试。

## 第 7 页：工作重点 2：安全正确性补丁

第二项重点工作是安全正确性补丁。

首先，FMP 调制后必须走 CheckMotionBatch。也就是说 modulated trajectory 的每个离散点和每条相邻边都要通过碰撞检查，才能进入 dispatch。

其次，在 C++ RRT-Connect 里增加了 direct shortcut。如果起点到终点本身就是安全边，就直接返回两点路径，不再进入随机 RRT，这可以避免安全局部段被不必要地扰动。

第三，增加 validatePath 和失败空路径机制。现在局部规划成功返回前必须验证路径首点、末点、每个点和每条边；如果失败，path_points 就是 0，不允许返回 start-goal 假路径。

最后，dispatch 前加入安全门，包括起点一致性、关节边界、速度加速度估计，以及保守时间缩放。这样可以减少控制器因为起点断层或动态约束不合理产生 ABORT 的情况。

## 第 8 页：工作重点 3：最小评估体系

第三项重点工作是最小评估体系。

这个 evaluator 的作用是把每次实验从“看动画”变成“看指标”。它会输出绕障成功性，比如 nominal 和 modulated 的 invalid state 和 invalid edge 数量；也会输出平滑性指标，比如关节路径长度、最大关节跳变、jerk、速度和加速度。

同时，它会记录控制器执行状态，包括 action status、error code 和是否 abort。对于局部路径，它会记录 RRT stop reason、耗时和碰撞查询次数。

最近我还增强了 evaluator 和真实 collision backend 的连通性检查。现在评估开始前会做 preflight，如果 check_motion_batch 后端没接上，结果会被标记为 result_interpretable=false。这样就不会把后端不可用误解成算法失败。

此外，系统现在支持 planner output 到 evaluator 到图表的一条命令流程，为后续批量实验打基础。

## 第 9 页：迁移难点：为什么这件事复杂

这一页我想强调一下为什么混合算法迁移到 ROS2 上比较复杂。

第一是任务空间到关节空间的映射。MATLAB 里我们更容易画出末端轨迹和障碍物之间的关系，但是 UR 控制器最后执行的是关节轨迹。如果使用末端平面轨迹，就必须通过 IK 映射到关节空间，而 IK 的连续性会直接影响后续 FMP 调制的平滑性。

第二是尺度映射。MATLAB 里的参数在抽象空间中效果比较稳定，但 ROS2 中所有东西都有物理尺度，包括障碍物大小、机械臂 link 尺寸、RRT 步长、edge resolution 和 via 插值密度。这些参数必须成组调整。

第三是执行链路。规划成功不等于控制器能跟踪成功。Gazebo、RViz、MoveIt PlanningScene 和 controller state 必须一致。因此我这段时间做的很多工作，本质上是在建立可复核的工程闭环。

## 第 10 页：当前预期效果

短期来看，我希望系统形成一条稳定的离线实验流程：启动仿真、执行规划、碰撞复核、轨迹下发、导出评估结果。

每次实验都会产生 JSON、CSV 和图表。这样之后调参时，我们不是凭主观观察判断效果，而是可以比较绕障成功率、碰撞点数量、轨迹长度、jerk 和控制器执行状态。

中期来看，我会用五类固定场景做实验，包括无障碍、轻微障碍、中等障碍、窄通道和不可绕障场景。目标是逐步对齐 MATLAB 参数尺度，并为后续真实 UR 机械臂迁移保留安全约束和回退机制。

## 第 11 页：论文阅读：pRRTC 是什么

后面我汇报一下最近阅读的 pRRTC 论文。

这篇论文全名是《pRRTC: GPU-Parallel RRT-Connect for Fast, Consistent, and Low-Cost Motion Planning》。它提出了一种面向 GPU 的并行 RRT-Connect。

我理解它的重点不是简单地并行跑很多个 RRT，而是从树扩展、碰撞检测、最近邻搜索和内存访问方式上整体重构 RRT-Connect。

论文在 MotionBenchMaker 上报告了最高大约 10 倍加速，规划时间标准差最高降低约 5.4 倍，初始路径代价平均降低约 1.4 倍。同时论文还展示了双 Franka Panda 机械臂在动态障碍物场景下的实时 replanning。

## 第 12 页：pRRTC 的关键技术点

pRRTC 有几个对我比较有启发的技术点。

第一是双树并行扩展。多个 GPU block 同时执行 RRT-Connect 扩展，并用 atomic 操作维护共享树。这说明局部规划的瓶颈不只在采样策略，也在扩展方式和数据结构。

第二是边碰撞检测并行。论文不是只检查节点，而是把一条边上的多个离散配置同时检查。这个思想和我现在做的 CheckMotionBatch 很接近，也说明 edge validation 对机械臂避障非常重要。

第三是最近邻搜索并行化。它把树节点划分给多个线程，再用 reduction 得到最近邻。这对我后续优化 C++ local planner 有启发。

第四是粗到细碰撞模型。论文使用球体近似和分层碰撞检测，这对后续提升 collision backend 性能和建模安全边界也有参考价值。

## 第 13 页：pRRTC 对本项目的影响

我认为 pRRTC 对本项目最直接的启发有三点。

第一，RRT-Connect 比单向 RRT 更适合局部段快速连接。现在我已经把 C++ local planner 做成 RRT-Connect 形式，并加入 direct shortcut 和 validatePath。

第二，edge validation 是关键。以前只看离散节点容易漏掉中间边的碰撞，现在我已经把 states 和 edges 都纳入 CheckMotionBatch。

第三，规划时间稳定性和路径质量需要指标化。pRRTC 特别强调 consistency，也就是规划时间方差小。对应到我的项目里，后续 evaluator 不只看成功率，还要看耗时、collision queries、路径长度和 jerk。

不过 GPU 并行不是我当前阶段的重点。当前我先保证 ROS2、MoveIt2 和 UR 仿真链路正确，后续再考虑批量 edge check、更高效最近邻或更接近 pRRTC 的局部规划优化。

## 第 14 页：后续计划

下一阶段我会做两件事。

第一是固定 5 类测试场景，形成可重复的评估表。每组实验都输出轨迹图、collision timeline、JSON 和 CSV，并且把仿真动画、RViz marker 和 evaluator 指标对齐。

第二是继续对齐 MATLAB 的尺度关系。具体来说，要联动调整 RRT step、edge resolution 和 via 插值密度，而不是单独改某一个参数。同时，继续使用 C++ collision backend 作为真实碰撞判断入口。

总的来说，当前项目已经从“算法移植能跑”推进到“运行链路可复核、失败原因可区分、实验结果可量化”。这为后面做正式实验和写论文打下了基础。
