# R模型专项分析

r_four_parameter_reasonableness_20261101.md对应近期统一口径的参数合理性研究；r_joint_fit、r_reduced_models、r_nonlinear_models和r_rain_validation_review保留联合拟合、少参数、非线性与跨天探索。

不同分析的样本集合与处理时点可能不同。当前主结论使用v14/v22/v26共331个40 s冻结窗口，按整轮留出比较泛化，不随机拆分相邻窗口。常数模型未被四参数模型稳定超越，因此提交运行固定R80、关闭在线训练。

标签含GNSS参考、TDC、局部OCXO变化及相关噪声，不称为纯测量噪声。跨天差异不能单独归因于降雨。当前数据支持范围见[证据映射](../../docs/evidence_map.md)，复算见[数据结果说明](../../data/results/README.md)。

部分分析中引用的绘图文件为当时本地生成，未随本提交包分发；对应数值结果见 data/results/。
