# UR BT - 机器人决策系统

ur_bt 是一个基于py_trees的机器人决策系统框架，支持机械臂、夹爪的联合控制

## 特性

- **机器人行为封装** ：将机械臂和手部操作封装为py_trees行为节点
- **行为树管理** : 提供完整的行为树创建、执行和监控功能
- **ZMQ集成** ：基于现有ZMQ通信架构
- **黑板系统**: 支持行为节点间数据共享和状态管理
- **配置驱动**: 通过YAML配置文件管理参数

## 安装

```bash
pip install -r requirements.txt
```

### 主要依赖

- `py-trees>=2.2.0` - 行为树核心库
- `pyzmq>=25.1.1` - ZMQ通信
- `numpy>=1.24.3` - 数值计算
- `pyyaml>=6.0` - 配置文件解析


## 项目结构

```
ur_bt/
├── src/ur_bt/                      # 核心代码
│   ├── behavior_tree.py            # 行为树管理器
│   ├── blackboard_manager.py       # 黑板管理器
│   ├── behaviors/                  # 行为节点
│   └── clients/                    # ZMQ客户端
|
├── test/                           # 单元测试代码
├── tasks/                          # 自定义任务程序
├── example/                        # 演示demo
├── config.yaml                     # 配置文件
└── requirements.txt                # 依赖管理
```

## 快速开始

### 1. 配置系统

编辑 `config.yaml` 文件：

```yaml
zmq:
  arm:
    # ur_move接口 (端口5605) - 用于轨迹规划和执行
    ur_move:
      host: "localhost"
      port: 5605
      timeout_ms: 120000  # 超时时间(毫秒)
```
TODO

## 故障排除

### 常见问题

1. **ZMQ连接失败**
   - 检查服务器地址和端口配置
   - 确认ZMQ服务器正在运行

2. **行为树执行超时**
   - 调整超时时间配置
   - 检查机器人硬件状态

3. **黑板权限错误**
   - 确保在行为节点中正确注册了变量权限
   - 检查变量名是否正确

### 日志分析

系统日志保存在 `ur_bt.log` 文件中，包含详细的执行信息和错误信息。

## 许可证

本项目采用MIT许可证。