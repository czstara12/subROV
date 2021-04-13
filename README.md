# subROV

- [x] raspi 串口错误处理
- [x] 目标值通道同步改为树莓派串口和线控分别同步
- [ ] 积分分离PID
- [ ] 微分滤波PID
- [x] yaw控制bug修复

## PID调试笔记

|      | Roll    | Pitch    | Yaw  | Throttle(z) | Forward(x) | Lateral(y) |
| ---- | ------- | -------- | ---- | ----------- | ---------- | ---------- |
| P    | 0.03    | -0.015   |      | 2           |            |            |
| I    | 0.00004 | -0.00006 |      | 0.004       |            |            |
| D    | 0.2     | -0.3     |      | 1           |            |            |

 

 