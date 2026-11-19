# 当前后端与录制回放

接口与操作以 [无动力执行端](17_No_Power_Executor.md) 为准。默认wearable保留虚拟执行端；serial入口接入新版robot_no_power适配，不启用旧PC_MANUAL或实体轨迹回放。

Web/新录制schema4区分真实输入、执行端、控制权、参考、requested/output_target/measured及夹爪保护；measured始终null。录制保存保护事件与终止原因，再归档；失去遥测不伪造打开完成。记录双机boot和配置标识，变更时中断。最长10分钟。

历史schema2/3原样保留；明确几何约定的已保存目标仅在虚拟执行端查看和回放，不重新解释旧IMU。无动力记录回放同样不向robot发送目标。来源和角度语义不明确的记录仅查看元数据与导出。

[旧后端设计](archive/Backend_Data_Replay_Pre_V2.md)保留历史证据，其旧实体控制与“悬空超时不打开”等条款不适用于当前软件。当前闭爪到期必须请求打开；后续有动力夹持仍需工位与承接条件单独验证。
