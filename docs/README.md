# 文档索引

| 查阅事项 | 文档 |
| --- | --- |
| 原型接线、编译烧录、串口命令及排障 | [用户指南](user-guide.md) |
| 分板接口、实时线程、EEPROM 分区、资源预算与阶段规划 | [系统架构](architecture.md) |
| 干涉模型、锁定点、探测器功率预算及台架 | [光学设计](design/optical.md) |
| FPC2 逐脚映射、测点、供电、模拟链路与采样时序 | [电气设计](design/electrical.md) |
| 双板叠装、3D 打印导槽外壳尺寸与装机规程 | [机械与外壳设计](design/mechanical.md) |
| 谐波解调、在线校准、观测器与闭环控制 | [算法设计](design/algorithm.md) |
| 返修位置、实装差异与下版检查项 | [PCB 勘误](design/pcb-errata.md) |
| 实验条件、历史验证结果与待测项目 | [实验台账](experiments/README.md) |

矢量图源码位于 `docs/figures/src/*.tex`，修改后可运行 `python3 scripts/render_docs_figures.py` 重新编译导出 `docs/figures/*.svg`。
