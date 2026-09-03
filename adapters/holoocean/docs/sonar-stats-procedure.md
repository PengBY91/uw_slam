# 声呐统计对比流程（PREP-A-10）

> 工具：`adapters/holoocean/tools/sonar_stats.py` · 单测：`tests/test_sonar_stats.py` · 状态：2026-09-03 工具在合成帧与 MCAP 往返上验证，尚未对真实 HoloOcean 录制或真机声呐跑过。

## 0. 工具输出什么

对一段（或两段）MCAP 里的 `uw.domain.SonarFrame`（uint8，行=距离、列=升序方位）给出：

| 指标 | 含义 | 到货后用途 |
|---|---|---|
| `histogram` | 256 档强度直方图（全部帧累计） | 直观看真机是否有不同的量化/增益曲线 |
| `floor_rayleigh_sigma` | 无目标区域的 Rayleigh 尺度 σ（LSB） | 对齐 HoloOcean `AddSigma`（加性噪声主导时 σ≈AddSigma×255）|
| `floor_sample_std` | 同一批单元的普通标准差 | 与 σ 交叉核对分布是否真是 Rayleigh |
| `signal_to_clutter_ratio` / `_db` | 目标单元均值 / 底噪 Rayleigh 均值 | 对齐 `MultSigma`（乘性噪声压低目标/底噪比） |
| `per_range_mean_all` / `per_range_mean_floor` | 逐距离行均值（全部 / 只算底噪单元） | 对齐 `RangeSigma` 与距离衰减（真机 TVG 曲线） |
| `per_beam_mean` | 逐波束均值 | 看方位增益不均匀 / 旁瓣 |
| `nonzero_fraction` / `zero_fraction` / `saturation_fraction` | 非零 / 零 / 饱和单元比例 | HoloOcean 把负噪声裁到 0，真机通常不会：零比例差异本身就是差距 |

无目标区域的判定：先用所有非零单元的中位数反推一个 σ₀（Rayleigh 中位数 = 1.1774 σ），把 σ₀ × 3.717（Rayleigh 99.9% 分位）以下的非零单元算作底噪，再在这些单元上做 Rayleigh 极大似然（含 Sheppard 量化修正）。目标单元 = 阈值以上。已知局限：(1) 底噪的 1e-3 尾部会混入"目标"，目标均值略被拉低（单测里约 3%）；(2) 零值单元不参与拟合——HoloOcean 的加性高斯噪声裁零后并不是 Rayleigh，真机包络检波才是，所以对仿真数据 σ 的绝对值只做相对比较用。

## 1. 现在就能做：HoloOcean 两种声呐模式互比

1. 用 A-03 的合同基线分别以 `sonar_1200khz`（RangeMax 50）和 `sonar_750khz`（RangeMax 120，`AddSigma`/`RangeSigma` 调大近似更宽波束）各录一段同场景 60 s（`record_session.py`，声呐 `/raw/sonar_frame`）。
2. 运行：

```bash
cd adapters/holoocean
.venv/bin/python tools/sonar_stats.py --bag /tmp/sim_1200k.mcap --compare /tmp/sim_750k.mcap \
    --out docs/perf/sonar_stats_sim_1200k_vs_750k_<date>.json --plot /tmp/sonar_stats.png
```

3. 预期：`floor_rayleigh_sigma` 的 B/A 比值约等于两份配置 `AddSigma` 之比；`per_range_mean_floor` 在 750 kHz 配置上随距离衰减更慢（量程更远）。比值对不上说明工具的底噪判定被目标占比带偏，先减少场景里的目标再比。

这一步验证的是工具本身；结论表提交到 `adapters/holoocean/docs/perf/`。

## 2. 到货当天：真机 vs HoloOcean

1. **同一场景**：水池里放同一组几何体（PREP-A-08 水池关卡在 UE5 里按实物尺寸复现），ROV 静止悬停在同一位姿。
2. **同一量程与模式**：真机 SDK 设 1.2 MHz / 50 m（或 750 kHz / 120 m），HoloOcean 用对应 overlay；帧率 10 Hz 即可（统计量与帧率无关）。
3. **录制**：真机经 PREP-B-04 声呐 adapter 写到 `/raw/sonar_frame`（若 SDK 只出扇形笛卡尔图，先反投影回极坐标，这是 adapter 的职责），60 s；HoloOcean 用 `record_session.py` 录同样时长。
4. **对比**：`sonar_stats.py --bag real.mcap --compare sim.mcap`。
5. **调参顺序**：
   - `AddSigma`：让仿真 `floor_rayleigh_sigma` 追上真机（先看 `zero_fraction`——真机若几乎没有零单元，仿真要把 `AddSigma` 抬高到裁零可忽略）；
   - `MultSigma`：让 `signal_to_clutter_db` 对齐；
   - `RangeSigma` 与场景材质反射率：让 `per_range_mean_floor` 曲线形状对齐（真机 TVG 会把远处底噪抬平，HoloOcean 没有 TVG，需要在 adapter 侧或规格里说明）；
   - 逐波束：`per_beam_mean` 差异大说明真机有方位增益不均或旁瓣，HoloOcean 无对应参数，记入 `SIM-SON-003` 局限。
6. 结果与选定参数写回 `blue_rov_contract_base.json` 的声呐 overlay，并在 `docs/specifications/holoocean-realtime-closed-loop-simulation-spec.md` 的 `SIM-SON-*` 条目登记"已对齐 / 不可对齐"的项。

## 3. 已知不能被这套统计覆盖的差距

- 波束宽度（0.95° / 0.6°）决定的方位模糊：HoloOcean `ImagingSonar` 没有该参数，逐波束均值看不出来，需要用 PREP-B-06 控制点的角向定位误差间接量化。
- 多径 / 旁瓣结构：直方图只能看到能量，看不到几何位置；到货后用带角反射器的场景人工比对图像。
