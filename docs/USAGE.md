# 使用说明（中文 / English）

757 LNAV/VNAV Plus — FlightFactor 757/767 的独立 LNAV/VNAV 增强插件。
本插件不修改 SASL、不改飞机文件、不改 FF 的 avionics 插件。

---

# 一、中文使用说明

## 1. 安装

1. 把 release 压缩包解压到：

   ```
   X-Plane 12/Aircraft/Laminar Research/Boeing757-Full/plugins/757LVNAVPlus/
   ```

2. 目录结构应为：

   ```
   757LVNAVPlus/
     64/
       win.xpl          （或 mac.xpl / lin.xpl）
     README.md
     LICENSE
     NOTICE.md
     docs/
   ```

3. 启动 X-Plane，载入 FF757。首次加载时若出现“未签名插件”提示，在插件的安全设置里允许它即可。

4. 打开 X-Plane 的 `Log.txt`，确认出现：

   ```
   [757LVNAV+] 757 LNAV/VNAV Plus v1.1.1 starting
   [757LVNAV+] FF avionics datarefs: found
   ```

## 2. 首次使用流程

1. **在 FF757 的 FMS 里输入并激活航路**（`ACT RTE`）。插件读取的是“已激活”的航路。
2. **让一台 CDU 停留在 LEGS 页面**。优先用右座 CDU2，这样机长位 CDU 不受影响。
   - 插件扫描时会临时翻页，扫描结束后会恢复到你原来所在的页面。
   - 只有 CDU 显示 LEGS 页面时才能扫描出航路。
3. 正常接通 **A/P + LNAV / VNAV**。插件只在检测到 FF757 的 LNAV/VNAV 模式时才写入修正。
4. 打开 **Plugins → 757 LNAV/VNAV Plus → Show status window** 观察状态。

## 3. 状态窗口怎么看

| 行 | 含义 |
| --- | --- |
| `Plugin : ENABLED` | 插件总开关状态 |
| `Aircraft: FF757/767` | 是否正确识别为 FF 机模 |
| `AP : mode / roll / pitch / vnavPhase` | FF757 原始自动驾驶模式值 |
| `engaged / lnavMode / vnavMode` | 插件识别到的 AP、LNAV、VNAV 状态 |
| `LNAV : ACTIVE src=ROUTE` | LNAV 正在写入；`src` 表示来源（`ROUTE` / `ROUTE/SYNTH` / GPS CDI） |
| `xtk / dist / base / target / corr` | 偏航距、距下一航点距离、基准航向、目标航向、修正量 |
| `active #n -> next #n` | 当前生效航点与下一航点序号 |
| `VNAV : PTH ACTIVE / SPD STBY` | 首词为 FF757 当前 VNAV 子模式（`SPD` / `PTH`），对应 PFD 上的 FMA；`ACTIVE` = 正在修正垂直路径，`STBY` = 已识别但不干预 |
| `reqVS / vs / corr` | 所需升降率、实际升降率、俯仰修正量 |
| `Route : CDU bound=1 valid=1 legs=... wpts=...` | 航路来源与解析结果；`valid=1` 表示航路可用 |
| `Status : ...` | 当前提示/错误信息 |

要点：

- **`SPD STBY` / `PTH STBY` 是正常的待命状态**：前者是 VNAV SPD（爬升/速度保持，靠俯仰控速），后者通常是 VNAV PTH 但正保持 MCP 高度、尚未到 TOD 或误差在死区内。插件只是不写俯仰，模式已经被识别。
- 只有 `VNAV : ACTIVE` 时才会看到俯仰修正。

## 4. 航路扫描规则

- 默认 **页面内容变化时扫描一次**：LEGS 页面上的航点、高度、航段距离变化后触发。
- 同一份内容不会反复扫描；默认**没有周期性定时扫描**。
- 修改航路后（`MOD RTE` → `EXEC` → `ACT RTE`），内容变化会自动触发扫描。
- 也可以手动点菜单 **Rescan CDU route** 强制扫描一次。
- 若不想在 MOD 页面扫描，可在配置里设 `cdu_scan_on_exec_only=1`，只在执行后扫描。

## 5. 菜单项

菜单路径：**Plugins → 757 LNAV/VNAV Plus**

| 菜单 | 作用 |
| --- | --- |
| Enable plugin | 总开关 |
| LNAV enhancement | 开关 LNAV 增强 |
| VNAV enhancement | 开关 VNAV 增强 |
| VNAV smoothing | 开关 VNAV 俯仰平滑 |
| LNAV FD roll (experimental) | 实验性 FD 滚转桥接，默认关闭 |
| Rescan CDU route | 手动重新扫描 CDU 航路 |
| Show status window | 显示/隐藏状态窗 |
| Reload config | 重新读取配置文件 |
| Reset defaults | 恢复默认配置 |
| Dump diagnostics | 把诊断信息写入日志（不会触发扫描） |
| About | 版本与说明 |

也可以把以下命令绑定到按键：

```
757lvnv/toggle          总开关
757lvnv/toggle_lnav     LNAV 增强
757lvnv/toggle_vnav     VNAV 增强
757lvnv/toggle_fd_roll  实验性 FD 滚转
757lvnv/status_window   状态窗
757lvnv/dump            输出诊断日志
```

## 6. 配置文件

- 位置：`X-Plane 12/Output/preferences/757LVNAVPlus.cfg`
- 日志：`X-Plane 12/Output/757LVNAVPlus.log`
- 删除配置文件即恢复默认；改完文件后点菜单 **Reload config** 生效。
- 可调项包括：LNAV 增益/切入角/转弯提前量、VNAV 俯仰修正上限/速率/平滑时间常数、CDU 扫描方式、调试输出等。完整列表见 `docs/CONFIG.md`。

## 7. 安全边界

| 限制 | 默认 |
| --- | --- |
| LNAV 切入角 | 25° |
| LNAV 航向误差上限 | 60° |
| LNAV 转弯提前用的坡度 | 25° |
| VNAV 俯仰修正上限 | ±2.5° |
| VNAV 俯仰修正速率 | 3°/s |
| VNAV 所需升降率上限 | ±3000 fpm |
| VNAV 平滑时间常数 | 0.12 s |

插件在以下情况自动停止写入：飞机在地面、A/P 未接通、未识别到 FF757 的 LNAV/VNAV 模式、航路或 CDI 数据无效、所需数据引用缺失。任何一项不满足，飞机就按 FF757 原本的逻辑飞。

## 8. 常见问题

| 现象 | 处理 |
| --- | --- |
| `route_valid=0`，`Route : -` | 确认有一台 CDU 停在 LEGS 页面，且航路已 `ACT RTE`；再点 **Rescan CDU route** |
| 状态窗显示 `CDU is not showing a LEGS page` | 把右座 CDU 切到 LEGS 页 |
| `CDU scan timeout` | 页面键没被接受，再点一次 Rescan，或检查 CDU2 的翻页键数据引用 |
| 显示 `PTH STBY`，`src=MCP HOLD` | 正常现象：飞机正保持在 MCP 高度，VNAV PTH 无需修正。进入下降剖面后 `src` 变为 `ROUTE PROFILE` 并显示 `PTH ACTIVE` |
| 显示 `SPD STBY` | 正常现象：处于 VNAV SPD（爬升/速度保持）阶段，插件不干预俯仰 |
| `Aircraft: not FF757/767` | 机模不是 FF757/767，或 FF avionics 未加载，插件保持空闲 |
| 完全无反应 | 看 `Log.txt` 中是否有 `[757LVNAV+]` 行；确认插件未被安全设置拦截 |

## 9. 卸载

关闭 X-Plane，删除 `plugins/757LVNAVPlus/` 整个目录即可。如需清理残留，再删除
`Output/preferences/757LVNAVPlus.cfg` 和 `Output/757LVNAVPlus.log`。

---

# 二、English User Guide

757 LNAV/VNAV Plus is a standalone enhancement layer for the FlightFactor 757/767.
It does not modify SASL, aircraft files, or the FF avionics plugin.

## 1. Installation

1. Extract the release zip into:

   ```
   X-Plane 12/Aircraft/Laminar Research/Boeing757-Full/plugins/757LVNAVPlus/
   ```

2. Expected layout:

   ```
   757LVNAVPlus/
     64/
       win.xpl          (or mac.xpl / lin.xpl)
     README.md
     LICENSE
     NOTICE.md
     docs/
   ```

3. Start X-Plane and load the FF757. If X-Plane warns about an unsigned
   plugin, allow it in the plugin security settings.

4. Check `Log.txt` for:

   ```
   [757LVNAV+] 757 LNAV/VNAV Plus v1.1.1 starting
   [757LVNAV+] FF avionics datarefs: found
   ```

## 2. First-flight workflow

1. **Enter and activate a route in the FF757 FMS** (`ACT RTE`). The plugin
   reads the active route.
2. **Leave one CDU on a LEGS page.** The right-hand CDU2 is preferred so the
   captain's CDU stays free.
   - The plugin pages through the LEGS pages while scanning and then restores
     the page it started from.
   - A route can only be captured while a CDU is showing a LEGS page.
3. Engage **A/P + LNAV / VNAV** normally. The plugin only writes guidance while
   the FF757 LNAV/VNAV mode is detected.
4. Open **Plugins → 757 LNAV/VNAV Plus → Show status window**.

## 3. Reading the status window

| Line | Meaning |
| --- | --- |
| `Plugin : ENABLED` | Master switch |
| `Aircraft: FF757/767` | Aircraft detected correctly |
| `AP : mode / roll / pitch / vnavPhase` | Raw FF757 autopilot mode values |
| `engaged / lnavMode / vnavMode` | States detected by the plugin |
| `LNAV : ACTIVE src=ROUTE` | LNAV is writing; `src` is the source (`ROUTE` / `ROUTE/SYNTH` / GPS CDI) |
| `xtk / dist / base / target / corr` | Cross-track, distance to fix, base course, target course, correction |
| `active #n -> next #n` | Active and next waypoint indices |
| `VNAV : PTH ACTIVE / SPD STBY` | First word is the FF757 VNAV submode (`SPD` / `PTH`) matching the PFD FMA; `ACTIVE` = correcting the vertical path, `STBY` = recognised but hands off |
| `reqVS / vs / corr` | Required VS, actual VS, pitch correction |
| `Route : CDU bound=1 valid=1 legs=... wpts=...` | Route source and parse result; `valid=1` means usable |
| `Status : ...` | Current message or error |

Key points:

- **`SPD STBY` / `PTH STBY` are normal standby states.** `SPD` is VNAV SPD
  (climb/speed hold, pitch controls speed); `PTH` usually means VNAV PTH while
  holding the MCP altitude, before TOD, or inside the deadband. The mode is
  recognised; the plugin simply writes no pitch.
- Pitch correction is only applied when the window shows `VNAV : ACTIVE`.

## 4. Route scanning rules

- By default the scanner runs **once per visible page-content change**: waypoint
  idents, altitudes, or leg distances on the LEGS page.
- The same content is never rescanned, and there is no periodic timer by default.
- Editing the route (`MOD RTE` → `EXEC` → `ACT RTE`) changes the content and
  triggers a scan automatically.
- You can force a scan with the menu item **Rescan CDU route**.
- To skip MOD pages entirely, set `cdu_scan_on_exec_only=1` in the config.

## 5. Menu items

Menu path: **Plugins → 757 LNAV/VNAV Plus**

| Item | Action |
| --- | --- |
| Enable plugin | Master switch |
| LNAV enhancement | Toggle LNAV layer |
| VNAV enhancement | Toggle VNAV layer |
| VNAV smoothing | Toggle FD pitch smoothing |
| LNAV FD roll (experimental) | Optional FD-roll bridge, off by default |
| Rescan CDU route | Manual route rescan |
| Show status window | Show/hide the status window |
| Reload config | Re-read the config file |
| Reset defaults | Restore default config |
| Dump diagnostics | Write diagnostics to the log (does not scan) |
| About | Version and notices |

Bindable commands:

```
757lvnv/toggle          master switch
757lvnv/toggle_lnav     LNAV layer
757lvnv/toggle_vnav     VNAV layer
757lvnv/toggle_fd_roll  experimental FD roll
757lvnv/status_window   status window
757lvnv/dump            diagnostics dump
```

## 6. Configuration

- Config: `X-Plane 12/Output/preferences/757LVNAVPlus.cfg`
- Log: `X-Plane 12/Output/757LVNAVPlus.log`
- Delete the config file to restore defaults. After editing, use **Reload config**.
- Tunables cover LNAV gain/intercept/turn lead, VNAV pitch limit/rate/smoothing,
  CDU scan mode, and debug output. Full list in `docs/CONFIG.md`.

## 7. Safety limits

| Limit | Default |
| --- | --- |
| LNAV intercept angle | 25° |
| LNAV course error | 60° |
| LNAV bank used for turn anticipation | 25° |
| VNAV pitch correction | ±2.5° |
| VNAV pitch rate | 3°/s |
| VNAV required VS | ±3000 fpm |
| VNAV smoothing time constant | 0.12 s |

The plugin stops writing when the aircraft is on the ground, the AP is off, the
FF757 LNAV/VNAV mode is not detected, the route or CDI data is invalid, or a
required dataref is missing. In any of those cases the aircraft flies exactly as
the FF757 does on its own.

## 8. Troubleshooting

| Symptom | Fix |
| --- | --- |
| `route_valid=0`, `Route : -` | Make sure a CDU is on a LEGS page and the route is `ACT RTE`; then use **Rescan CDU route** |
| `CDU is not showing a LEGS page` | Switch the right CDU to LEGS |
| `CDU scan timeout` | Page keys were not accepted; rescan again or check the CDU2 page-key datarefs |
| `PTH STBY` with `src=MCP HOLD` | Expected: the aircraft is holding the MCP altitude, so VNAV PTH needs no correction. In the descent the source becomes `ROUTE PROFILE` and the line shows `PTH ACTIVE` |
| `SPD STBY` | Expected: the aircraft is in VNAV SPD (climb/speed hold) and the plugin does not touch pitch |
| `Aircraft: not FF757/767` | Not an FF757/767, or the FF avionics are not loaded; the plugin stays idle |
| Nothing happens at all | Look for `[757LVNAV+]` lines in `Log.txt`; check the plugin is not blocked by security settings |

## 9. Uninstall

Quit X-Plane and delete the `plugins/757LVNAVPlus/` folder. To remove leftovers,
also delete `Output/preferences/757LVNAVPlus.cfg` and `Output/757LVNAVPlus.log`.
