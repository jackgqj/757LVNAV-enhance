# Release 1.2.36 — CIFP Nav + RNAV1 Direct Bank

**Version:** `1.2.36+vnav-profile+lnav-turn+cifp-nav+rnav1-bank`  
**Platform:** Windows x64 (`64/win.xpl`)  
**Date:** 2026-11-12  
**Package:** `757LVNAVPlus-win-1.2.36-rnav1-bank.zip`

> **Before first use / 使用前必做**
>
> Leave at least one CDU on the **LEGS** page (right CDU preferred). Without
> it the route scanner cannot read the flight plan and LNAV stays idle.
>
> 必须把至少一部 CDU 停在 **LEGS** 页面（优先右 CDU），否则插件无法扫描航路，
> LNAV 增强不会生效。

---

## 中文

### 这是什么

这是 `757 LNAV/VNAV Plus` 的 CIFP + RNAV1 实验分支，不是稳定版。它保留原有的
LNAV/VNAV 监督层，并加入：

- **CIFP-only 离场程序补全**：从 X-Plane `Custom Data/CIFP/<ICAO>.dat` 解析
  ARINC 424 SID，坐标由 nav database 解析；不再用 FF757 `Proc` 作为程序源。
- **RNAV1 直接滚转**：插件直接写 FD 滚转角，而不是只写航向目标。
- **RF 弧 CIFP-only**：圆心、半径、转向、扫掠来自 CIFP；RF 段不使用普通
  LNAV 的路段锁存；RF 坡度前馈增加同帧方向检查。

### 主要变化

1. **CIFP 离场覆盖**
   - 60 NM 内最近机场的 CIFP SID 自动匹配；
   - 航路从程序中间开始时，允许匹配程序末尾 2 个航点；
   - 程序前缀在第一个公共航点处接回 FMS 航路；
   - 状态窗显示 `CIFP: <SID> <transition>, n/n fixes, n RF`；
   - `cifp_procedure_route=0` 或菜单 **CIFP departure route** 可关闭。

2. **RNAV1 直接滚转**
   - `lnav_bank_direct=1`（默认）：写绝对 `flight_director_roll`，并保持
     `override_flightdir_roll=1`；
   - 坡度 = 航迹误差 + 偏航距，限 25°，指令速率 6°/s；
   - 0.5°/0.02 NM 死区、0.10 s 低通、0.5 航迹转率阻尼；
   - fly-by / RF 圆弧提供坡度前馈；
   - 前馈符号与当前目标航向误差相反时，本帧不使用前馈（`ff=0`）；
   - FD 滚转不可写时自动回退到航向直写。

3. **RF 弧**
   - 从 CIFP RF 行读取圆心、半径、左/右转和扫掠；
   - RF 腿完全绕过普通 LNAV 锁存；
   - RF 几何未改；RF 坡度前馈只影响直滚目标；
   - 状态窗/日志 `src=RF ARC`，`RFarc=..%`。

4. **可观察性**
   - 状态窗新增 `CIFP:` 行、`BankD=1/0`；
   - `LNAV servo` 日志新增 `bankCmd`、`tb`（前馈坡度）、`ff`（是否使用前馈）、
     `tz`、`rem`、`tan`、`arc`。

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
lnav_bank_filter_tau_s=0.10
lnav_bank_track_rate_damp=0.5
cifp_procedure_route=1
```

完整配置见 [`docs/CONFIG.md`](docs/CONFIG.md)。

### 安装

1. 完全退出 X-Plane。
2. 解压发布包，把 `64/` 放到当前 757 机型的：
   `plugins/757LVNAVPlus/64/`
3. 启动 X-Plane，加载 FF757；`Log.txt` 应显示
   `757 LNAV/VNAV Plus v1.2.36+cifp-nav+rnav1-bank starting`。
4. 把一部 CDU 放到 **LEGS** 页面，装载航路，接通 LNAV。

### 验证

离线测试：

```text
guidance          223 checks
bank                8 checks
procedure          25 checks
procedure-route    16 checks
CDU                48 checks
CIFP               21 checks
```

全部 0 失败。

已做真实 CIFP 数据检查（ZULS EO28R）：程序可解析约 22/23 个航点、16 个 RF
圆心；LS610 附近的 RF 前馈方向不一致已通过同帧检查拦截。

### 已知限制

- CIFP 不含跑道入口坐标，离场第一段可能从第一个可解析航点开始。
- 跑道过渡与公共航段仍是分开的 CIFP 块，当前只做程序/公共段合并。
- SID 检测基于航点 ident；平行跑道共享长前缀时可能并列，状态窗显示实际选中的
  程序名。
- CIFP 周期可能与 FF757 navdata 不同，使用前对照 FMS 航路。
- 直滚是实验性功能；FF757 AP 自身滚转速率仍是最终限制。
- 仅提供 Windows x64 包；macOS/Linux 需从源码构建。

### 回退

把备份的上一版 `win.xpl` 放回 `plugins/757LVNAVPlus/64/`，或删除本插件文件夹。
本分支的稳定版仍在 `E:\FF757 enhance`。

---

## English

### What this is

This is the experimental CIFP + RNAV1 branch of **757 LNAV/VNAV Plus**, not the
stable release. It keeps the existing LNAV/VNAV supervisory layer and adds:

- **CIFP-only departure completion**: ARINC 424 SIDs from X-Plane
  `Custom Data/CIFP/<ICAO>.dat`, with coordinates resolved through the nav
  database. The FF757 `Proc` file is no longer a procedure source.
- **RNAV1 direct bank**: the plugin writes an absolute FD roll target instead
  of only a heading target.
- **CIFP-only RF arcs**: centre, radius, turn and sweep come from CIFP. RF legs
  bypass the ordinary LNAV leg latch, and RF bank feed-forward has a same-frame
  direction check.

### Highlights

1. **CIFP departure overlay**
   - matches the nearest airport CIFP SID within 60 NM;
   - a route that starts mid-procedure may match with the last two SID fixes;
   - the procedure prefix joins the FMS route at the first common fix;
   - the status window shows `CIFP: <SID> <transition>, n/n fixes, n RF`;
   - disable with `cifp_procedure_route=0` or **CIFP departure route**.

2. **RNAV1 direct bank**
   - `lnav_bank_direct=1` (default): writes absolute `flight_director_roll`
     with `override_flightdir_roll=1`;
   - bank = track error + cross-track, capped at 25°, slewed at 6°/s;
   - 0.5° / 0.02 NM deadband, 0.10 s low-pass, 0.5 track-rate damping;
   - fly-by/RF arcs provide bank feed-forward;
   - feed-forward is ignored for the frame when its sign opposes the current
     target-course error (`ff=0`);
   - falls back to heading-direct if FD roll is not writable.

3. **RF arcs**
   - CIFP RF rows supply centre, radius, turn and sweep;
   - RF legs bypass the ordinary LNAV latch;
   - RF geometry is unchanged; RF feed-forward only affects the bank target;
   - status/log: `src=RF ARC`, `RFarc=..%`.

4. **Observability**
   - status window adds `CIFP:` and `BankD=1/0`;
   - `LNAV servo` logs `bankCmd`, `tb` (feed-forward bank), `ff` (used or not),
     `tz`, `rem`, `tan`, `arc`.

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
lnav_bank_filter_tau_s=0.10
lnav_bank_track_rate_damp=0.5
cifp_procedure_route=1
```

Full reference: [`docs/CONFIG.md`](docs/CONFIG.md).

### Installation

1. Close X-Plane completely.
2. Unzip the release and copy `64/` into the current 757 aircraft:
   `plugins/757LVNAVPlus/64/`
3. Start X-Plane, load the FF757; `Log.txt` should show
   `757 LNAV/VNAV Plus v1.2.36+cifp-nav+rnav1-bank starting`.
4. Put one CDU on the **LEGS** page, load the route and engage LNAV.

### Validation

```text
guidance          223 checks
bank                8 checks
procedure          25 checks
procedure-route    16 checks
CDU                48 checks
CIFP               21 checks
```

All zero failures.

Real CIFP data check (ZULS EO28R): about 22/23 procedure fixes resolved and
16 RF centres. The RF feed-forward direction mismatch near LS610 is now
rejected by the same-frame check.

### Known limitations

- CIFP has no runway-threshold coordinates; the first departure leg may start
  at the first resolved fix.
- Runway transitions and common legs remain separate CIFP blocks; only the
  procedure/common sequence is merged today.
- SID detection is ident-based; parallel-runway procedures with a long shared
  prefix can tie, and the status window shows the procedure actually selected.
- The CIFP cycle may differ from the FF757 navdata; cross-check the FMS route.
- Direct bank is experimental; the FF757 AP's own roll rate remains the final
  limit.
- Only a Windows x64 package is provided; macOS/Linux require a source build.

### Rollback

Restore the previous `win.xpl` backup to `plugins/757LVNAVPlus/64/`, or delete
the plugin folder. The stable plugin remains in `E:\FF757 enhance`.

---

**Binary SHA256:** `4EFF7E63FF05B19E7B77F4C19D750642CF2FB410CABC6ADCD44D5EC1C1C4BCD6`  
**License:** MIT, author credit **JACKGONG** required. See [`LICENSE`](LICENSE)
and [`NOTICE.md`](NOTICE.md).
