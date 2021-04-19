# subROV

- [x] raspi 串口错误处理
- [x] 目标值通道同步改为树莓派串口和线控分别同步
- [ ] 积分分离PID
- [ ] 微分滤波PID
- [x] yaw控制bug修复
- [x] 命令15修改
- [x] 帧差值积累
- [x] LED报警
- [x] 运行指示灯LED
- [x] oled显示优化
- [ ] flash PID参数保存
- [ ] 自检与调试程序
- [ ] OLED 屏幕显示float 并切换显示参数
- [ ] 蓝牙驱动与初始化
- [ ] 急停锁 上电默认解锁 只能由上位机锁定解锁 优先级高于电机锁
- [ ] 串口控制优化(丢包问题) 尤其对于帧差法控制
- [ ] 边缘计算回传数据包格式扩充(转为使用mavlink?)
- [ ] 上实时系统
- [ ] 全模型控制
- [ ] 报警电路(高亮度LED配合功率放大MOS/LED)
- [ ] 电调保护 避免长时间满油门控制
- [ ] 远程串口IAP

## PID调试笔记

| 树莓派 | Roll    | Pitch    | Yaw   | Throttle(z) | Forward(x) | Lateral(y) |
| ------ | ------- | -------- | ----- | ----------- | ---------- | ---------- |
| P      | 0.015   | -0.015   | -0.02 | 1.3         |            |            |
| I      | 0.00003 | -0.00005 | 0     | 0.001       |            |            |
| D      | 0.2     | -0.3     | -0.02 | 5           |            |            |

 

 