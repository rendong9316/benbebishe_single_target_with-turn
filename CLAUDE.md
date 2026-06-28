## 多入口一致性约束

本项目有两个主入口（`run_simulation_turn.m` 单次仿真、`run_mc_turn.m` 蒙特卡洛）和两个 runner（`single_track_runner_nanyang.m` 基础版、`single_track_runner_nanyang_adaptive.m` 自适应版）。任何架构级改动（新增/删除 runner、新增/删除 Phase、修改快照格式、修改评估指标等）必须同时更新所有相关入口和 runner，保持整体架构统一。**一改则全改**，禁止只改一个入口而遗留其他文件不同步。
