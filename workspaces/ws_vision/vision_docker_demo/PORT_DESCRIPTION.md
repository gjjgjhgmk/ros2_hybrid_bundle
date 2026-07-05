# 端口说明文档

本文档描述了 `ws_vision/vision_docker_demo` 项目中各个服务使用的端口及其对应程序。

## 端口概览

| 端口 | 协议 | 服务名称 | 容器 | 状态 |
|------|------|----------|------|------|
| 7000 | HTTP | 容器管理API | 主机 | 启用 |
| 7001 | HTTP | 标定服务器 | vision_calibration_jazzy | 启用
| 7002 | HTTP | 配置服务器 | 主机 | 启用
| 7010 | WebSocket | 图像推送服务器 | vision_record_jazzy | 启用 |
| 7020 | ZMQ | 图像技能 | vision_vision_jazzy | 启用 |
| 7021 | ZMQ | 标定服务 | vision_calibration_jazzy | 启用 |
