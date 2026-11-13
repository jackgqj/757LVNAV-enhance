# 757 LNAV/VNAV Plus — Release 1.2.36

**Version:** `1.2.36+vnav-profile+lnav-turn+cifp-nav+rnav1-bank`  
**Release date:** 2026-11-13  
**Platform:** Windows x64 (`64/win.xpl`)  
**Package:** `757LVNAVPlus-win-1.2.36-rnav1-bank.zip`  
**Binary SHA256:** `D3E7A4B754CBE4B1EA90676E02D932DE717AEA3C8A97BF195C17FD61FFE003B5`

> **使用前必做 / BEFORE FIRST USE**
>
> 必须把至少一部 CDU 停在 **LEGS** 页面（优先右 CDU），否则插件无法扫描航路，
> LNAV 增强不会生效。
>
> At least one CDU must be on the **LEGS** page (right CDU preferred). Without it
> the route scanner cannot read the flight plan and LNAV stays idle.
>
> **使用 FMS `DIR`（直飞）前，必须把 LNAV 数据源切换为 `GPS CDI`，并保持到下一个航点；本插件不自动检测 DIR。**
>
> **Before using FMS `DIR` (direct-to), switch the LNAV source to `GPS CDI` and keep it there until the next waypoint; the plugin does not auto-detect DIR.**

---

## 中文

### 本次发布重点

这是一个 **CIFP + RNAV1 实验分支**，不是稳定版。它在原有 LNAV/VNAV 监督层上加入了：

1. **CIFP-only 离场程序补全**
   - 从 X-Plane `Custom Data/CIFP/<ICAO>.dat` 解析 ARINC 424 SID；
   - 航点坐标通过 nav database 解析；
   - 60 NM 内最近机场自动匹配；
   - 航路从程序中间开始时，允许匹配程序末尾 2 个航点；
   - 在第一个公共航点处接回 FMS 航路；
   - 状态窗显示 `CIFP: <SID> <transition>, n/n fixes, n RF`。

2. **RNAV1 直接滚转**
   - `lnav_bank_direct=1`（默认）写绝对 FD 滚转角；
   - 坡度 = 航迹误差 + 偏航距，限 25°；
   - 指令速率 6°/s；
   - 0.5° / 0.02 NM 死区；
   - 0.08 s 低通；
   - 0.5 航迹转率阻尼；
   - FD 滚转不可写时自动回退航向直写。

3. **圆弧坡度前馈与方向保护**
   - fly-by / RF 圆弧给出计划坡度前馈；
   - 前馈符号与当前目标航向误差相反时，本帧不使用前馈，避免“前馈压住飞机、
     航迹卡死”。

4. **RF 弧**
   - RF 圆心、半径、转向、扫掠来自 CIFP；
   - RF 腿完全绕过普通 LNAV 的路段锁存；
   - RF 几何未改。

5. **VNAV 修正**
   - 剖面路径在飞机上方时保持当前高度；
   - 不再因为垂直速度非零，把反向推算出的高度（例如 65540 ft）当成 PTH 目标。

6. **性能**
   - CIFP 成功合并后按航路签名永久缓存；
   - 只有检测失败才每 5 s 重试；
   - CDU 内容未变化时不再每帧复制整条航路。

7. **DIR 使用方式**
   - 本版**不自动检测 DIR**；
   - 打 DIR 前切到 **GPS CDI**；
   - 保持到下一个航点；
   - 到点后切回 ROUTE。

### 安装

1. 完全退出 X-Plane。
2. 解压发布包，把 `64/` 放入当前 757 机型：
   `plugins/757LVNAVPlus/64/`
3. 启动 X-Plane，加载 FF757。日志应显示：
   `757 LNAV/VNAV Plus v1.2.36+vnav-profile+lnav-turn+cifp-nav+rnav1-bank starting`
4. 把一部 CDU 放到 **LEGS** 页面，装载航路，接通 LNAV。

### 默认配置

```ini
enabled=1
enable_lnav=1
enable_vnav=1
lnav_heading_direct=1
lnav_bank_direct=1
lnav_bank_k_track=1.4
lnav_bank_k_xtk=0.8
lnav_bank_max_deg=25
lnav_bank_rate_deg_s=6
lnav_bank_deadband_deg=0.5
lnav_bank_deadband_nm=0.02
lnav_bank_filter_tau_s=0.08
lnav_bank_track_rate_damp=0.5
cifp_procedure_route=1
```

完整配置见 [`docs/CONFIG.md`](docs/CONFIG.md)。

### 验证

```text
guidance          226 checks
bank                8 checks
procedure          25 checks
procedure-route    16 checks
CDU                48 checks
CIFP               21 checks
```

全部 0 失败。

### 已知限制

- CIFP 不含跑道入口坐标，离场第一段可能从第一个可解析航点开始；
- 跑道过渡与公共航段仍是分开的 CIFP 块；
- SID 检测基于航点 ident，平行跑道共享长前缀时可能并列；
- CIFP 周期可能与 FF757 navdata 不同，使用前对照 FMS 航路；
- 直滚为实验性功能，FF757 AP 自身滚转速率仍是最终限制；
- 不自动检测 DIR；必须手动切 GPS CDI；
- 仅提供 Windows x64 包。

### 回退

把上一版 `win.xpl` 放回 `plugins/757LVNAVPlus/64/`，或删除插件文件夹。  
稳定版仍在 `E:\FF757 enhance`。

---

## English

### Release highlights

This is the experimental **CIFP + RNAV1 branch**, not the stable release. It adds
the following on top of the existing LNAV/VNAV supervisory layer:

1. **CIFP-only departure completion**
   - ARINC 424 SIDs from X-Plane `Custom Data/CIFP/<ICAO>.dat`;
   - coordinates resolved through the nav database;
   - nearest airport within 60 NM;
   - a route starting mid-procedure may match with the last two SID fixes;
   - joins back to the FMS route at the first common fix;
   - status line `CIFP: <SID> <transition>, n/n fixes, n RF`.

2. **RNAV1 direct bank**
   - `lnav_bank_direct=1` (default) writes an absolute FD roll target;
   - bank from track error + cross-track, capped at 25°;
   - 6°/s command slew;
   - 0.5° / 0.02 NM deadband;
   - 0.08 s low-pass;
   - 0.5 track-rate damping;
   - falls back to heading-direct when FD roll is not writable.

3. **Arc bank feed-forward with direction guard**
   - fly-by and RF arcs publish the bank their radius needs;
   - when the feed-forward sign opposes the current target-course error, it is
     ignored for that frame (prevents the IDBON-style heading lock).

4. **RF arcs**
   - CIFP supplies centre, radius, turn and sweep;
   - RF legs bypass the ordinary LNAV leg latch;
   - RF geometry is unchanged.

5. **VNAV correction**
   - when the profile path is above the aircraft, it holds the current altitude;
   - a non-zero vertical speed no longer lets a backwards-extrapolated altitude
     (e.g. 65540 ft) become the PTH target.

6. **Performance**
   - a successful CIFP merge is cached for the lifetime of its route signature;
   - only failed detections retry every 5 s;
   - unchanged CDU content is no longer copied every frame.

7. **DIR procedure**
   - this build does **not** auto-detect DIR;
   - before DIR, switch LNAV source to **GPS CDI**;
   - keep it until the next waypoint;
   - then return to ROUTE.

### Installation

1. Close X-Plane completely.
2. Unzip the release and copy `64/` into the current 757 aircraft:
   `plugins/757LVNAVPlus/64/`
3. Start X-Plane and load the FF757. The log should show
   `757 LNAV/VNAV Plus v1.2.36+vnav-profile+lnav-turn+cifp-nav+rnav1-bank starting`.
4. Put one CDU on the **LEGS** page, load the route and engage LNAV.

### Defaults

```ini
enabled=1
enable_lnav=1
enable_vnav=1
lnav_heading_direct=1
lnav_bank_direct=1
lnav_bank_k_track=1.4
lnav_bank_k_xtk=0.8
lnav_bank_max_deg=25
lnav_bank_rate_deg_s=6
lnav_bank_deadband_deg=0.5
lnav_bank_deadband_nm=0.02
lnav_bank_filter_tau_s=0.08
lnav_bank_track_rate_damp=0.5
cifp_procedure_route=1
```

Full reference: [`docs/CONFIG.md`](docs/CONFIG.md).

### Validation

```text
guidance          226 checks
bank                8 checks
procedure          25 checks
procedure-route    16 checks
CDU                48 checks
CIFP               21 checks
```

All zero failures.

### Known limitations

- CIFP has no runway-threshold coordinates; the first departure leg may start
  at the first resolved fix;
- runway transitions and common legs remain separate CIFP blocks;
- SID detection is ident-based and parallel-runway procedures can tie;
- the CIFP cycle may differ from the FF757 navdata; cross-check the FMS route;
- direct bank is experimental; the FF757 AP's roll rate is the final limit;
- DIR is not auto-detected; switch to GPS CDI manually;
- only a Windows x64 package is provided.

### Rollback

Restore the previous `win.xpl` to `plugins/757LVNAVPlus/64/`, or delete the
plugin folder. The stable plugin remains in `E:\FF757 enhance`.

---

**License:** MIT, author credit **JACKGONG** required. See [`LICENSE`](LICENSE)
and [`NOTICE.md`](NOTICE.md).
