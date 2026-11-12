# 757 LNAV/VNAV Plus

> **Experimental CIFP + RNAV1 repository.** This tree is a copy of the stable
> 1.2.36+vnav-profile plugin used to prototype CIFP-only departure-procedure
> parsing and the direct-bank RNAV1 lateral mode.  ARINC 424 sequences come
> from `Custom Data/CIFP/<ICAO>.dat`; coordinates are resolved from the nav
> database.  See [`docs/CIFP_PROCEDURE_SOURCES.md`](docs/CIFP_PROCEDURE_SOURCES.md)
> and the bilingual [`RELEASE_NOTES.md`](RELEASE_NOTES.md).  The stable plugin
> remains in `E:\FF757 enhance`.

**An independent X-Plane plugin that improves LNAV/VNAV guidance for the FlightFactor 757/767 — it does not modify SASL, the aircraft files, or the FF avionics plugin.**

> 独立的 X-Plane 插件，用于增强 FlightFactor 757/767 的 LNAV / VNAV 表现。它**不修改 SASL**、不改机模文件、也不改 FF 的 avionics 插件；只读取 FF757 公开的数据引用，并写入 X-Plane 标准的导航 / 飞行指引目标数据引用。删掉一个文件夹即可完全移除。

当前版本：**v1.2.36+vnav-profile+lnav-turn+cifp-nav+rnav1-bank**（完整更新见 [`CHANGELOG.md`](CHANGELOG.md)；发布说明见 [`RELEASE_NOTES.md`](RELEASE_NOTES.md)）

> ## ⚠️ 使用前必做 / BEFORE FIRST USE
>
> **必须把至少一部 CDU 停在 `LEGS` 页面（优先右 CDU）；否则插件无法扫描航路，LNAV 增强不会生效。**
>
> **At least one CDU must be on the `LEGS` page before flight (right CDU preferred). Without it the route scanner cannot read the flight plan and the LNAV enhancement will stay idle.**

**目录**： [中文](#中文) | [English](#english)

---

## 中文

### 功能概览

- **RF 定半径弧段（CIFP-only）**：从 X-Plane `Custom Data/CIFP/<ICAO>.dat` 读取 ARINC 424 RF 航段（圆心、半径、左/右转、扫掠角），坐标由 nav database 解析。插件沿入弧航道带飞，弧上按“切线 + 径向修正”跟踪；RF 段完全绕过普通 LNAV 的路段锁存，避免弧后跳回旧腿。RF 坡度前馈带同帧方向一致性检查：当前馈符号与目标航向误差相反时（例如 IDBON），该帧不使用前馈，退回普通直滚。
- **横向控制：直滚默认，航向直写兜底**：默认 `lnav_bank_direct=1`，插件写绝对 FD 滚转角（航迹误差 + 偏航距，限 25°），同时保留航向直写路径。FD 滚转不可写或直滚关闭时，回退到 `override_nav_heading=1` + `nav_steer_deg_mag` 航向直写（风补角 `hdg−trk` + 航迹阻尼，限 ±10°），同一时间只允许一个横向控制器。机模每帧把 override 清 0 属正常，插件会重新置位。`lnav_heading_direct=0` 时直飞段交还机模、override 只保留在 RF 段；`lnav_rf_nav_override=0` 让 RF 段退回 ±4° FD 滚转修正。直飞段的小偏差再由 L1 前视项（`lnav_l1_factor`，默认 1.6）温和收敛。
- **LNAV 转弯与截获**：Boeing 式捕获——|XTK| ≥ 2.5 NM 时固定 30° 截获，以内按 `atan(xtk/dist)` 直飞修正；按地速与坡度限制计算提前转弯；航向锚定仅在 FMS 与自算航向差 ≤8° 时使用；CDI 校验需连续 ~2 s 不一致才降级，并有 `ROUTE HOLD` 兜底。
- **航点识别**：以 CDU LEGS 第 1 页第 1 行为"下一个航点"，双向校正索引；辅以 FF757 `AirnavDis/AirnavBrg` 同步与沿航迹投影排序。RF 标注只在（入口→终点）在程序库中确为 RF 对时才成立，不会把别的 SID 的弧套到直飞段。
- **CIFP 离场程序补全**：按航路前几个航点自动匹配最近机场（60 NM 内）的 CIFP SID；如果 CDU 路由从程序中间开始，允许 2 个航点且位于程序末尾的后缀匹配。程序航段、RF 圆弧和坐标在第一个公共航点处接回 FMS 航路；`cifp_procedure_route=0` 可关闭。CIFP 无跑道入口坐标，且跑道过渡与公共航段仍是分开的 CIFP 块，离场起始段可能从第一个可解析航点开始。
- **RNAV1 直接滚转**：`lnav_bank_direct=1`（默认）由插件写绝对 FD 滚转角，而不是只给航向目标；坡度按航迹误差 + 偏航距计算，限 25°，指令速率 6°/s，带 0.5°/0.02 NM 死区、0.10 s 低通和 0.5 的航迹转率阻尼。fly-by / RF 圆弧有坡度前馈；前馈符号与当前目标航向误差相反时本帧不使用，避免 IDBON 那类“前馈压住飞机、航迹卡死”。菜单 **LNAV bank direct (RNAV1)** 可开关。RF 段不使用普通 LNAV 的路段锁存。
- **VNAV 垂直路径**：按约束高度/垂直路径计算所需升降率，叠加垂直加速度阻尼，经低通与限速率写入 FD 俯仰；VNAV SPD（爬升/控速）阶段完全透传。下降（DES）时插件锁定俯仰优先权（`vnav_priority`），消除与机模 VNAV 的逐帧"夺权"。CDU 高度限定符按 `A`（at or above）/ `B`（at or below）/ 无后缀（at）分别处理：已满足的 at-or-above 下限不再被当成下降目标，但仍限制下降坡度，避免提前跌破限制。
- **VNAV 速度限制**：自动处理 250 KT / 10000 ft 硬限制，并按 1 NM / 10 KT 计算减速提前量（`vnav_speed_limits_enabled` 可关）；自动 250 KT 只在进入减速窗口后激活，不会在巡航高度提前几十海里点亮。
- **RNP / ANP 监控**：按飞行阶段给出 RNP（巡航 2.0 / 终端 1.0 / 最后进近 0.3 NM），`ANP(trk)` 为航迹保持误差代理量；持续超限约 2 s 显示 `NAV UNABLE RNP` 并写日志（仅提示，不干预操纵）。
- **航路读取**：优先读 FF757 私有 `rteData`；没有时扫描 CDU LEGS 页面（优先右 CDU），解析航点、高度、速度与航段距离，扫完恢复原页面。
- **性能**：FF757 navdata 每会话只建一次内存索引（16 字节记录，硬上限约 10 MB），之后按 ident 二分查找；飞行循环里没有磁盘扫描。
- **可观察、可关闭**：状态窗实时显示模式、航段来源、XTK、目标航向、RF 弧度进度、RNP/ANP 与提示；每个增强层都能用菜单、配置或 dataref 单独关闭。

### 工作原理

| 层 | 读取 | 写入 | 作用 |
| --- | --- | --- | --- |
| **航路** | 优先 FF757 私有 `rteData`；否则通过公开 CDU 显示 dataref 读 LEGS 页面 | 仅在扫描期间按 CDU 翻页键，扫完恢复原页面 | 航点坐标、高度、速度、距离、当前/下一航段 |
| **LNAV** | 航路几何、当前航段、GPS CDI、FD 滚转、航迹/航向 | 默认 `flight_director_roll` + `override_flightdir_roll=1`（绝对滚转角，直滚模式）；回退为 `override_nav_heading=1` + `nav_steer_deg_mag` 航向直写 | 更平滑、带提前转弯和坡度保持的目标；避免转弯过早回正 |
| **RF 弧** | X-Plane CIFP `Custom Data/CIFP/<ICAO>.dat` 的 RF 行；坐标由 nav database 解析 | 同 LNAV 直滚/航向路径；RF 坡度前馈带同帧方向检查 | 插件带飞入弧与真实定半径弧；RF 不走普通 LNAV 路段锁存 |
| **VNAV** | 航路约束、垂直路径、FD 俯仰、MCP 高度 | `flight_director_pitch` + `override_flightdir_ptch`；cockpit2 FD 俯仰仅在可写时 | 带阻尼、限速率的垂直路径修正；SPD 模式透传 |
| **RNP/ANP** | 航路几何 | 不写 | 仅状态窗与日志提示 |

插件按**透传**原则设计：只在 FF757 自动驾驶接通且识别到对应 LNAV/VNAV 模式时写入；VNAV 只在 ±2.5° 俯仰限幅内修正，绝不逆着 MCP 高度主动爬升/下降；缺少必要 dataref 时对应层自动待命，飞机行为与未装插件时一致。

不读取、不修改任何机模文件、SASL 脚本、`.secured` 模块或 FF 二进制。唯一写入的 `757Avionics/*` dataref 是扫描期间用到的 CDU 翻页键。

### 兼容性

- **X-Plane 12**（主要目标）。XP11.30+ 可能可用（只用稳定 XPLM API），但未测试。
- **FlightFactor 757 v2**（所有使用 `757767Avionics` 的 -200/-300/F 型号）。767 共用 avionics，也会被识别，但不是主要目标。
- 当前发布包 `757LVNAVPlus-win-1.2.36-rnav1-bank.zip` 为 **Windows x64**；其他平台按源码构建。

FF757 私有航路 dataref 是**未公开**的：插件做尽力绑定，并始终用标准 GPS CDI 校验。如果某版本完全不暴露私有航路树（本机测试的 FF757 v2 就是如此），插件改读 CDU **LEGS 页面**：抓取显示的航点、高度、速度与航段距离，用 FF757 navdata 解析坐标，扫完恢复原页面。如果连 CDU 显示也不可用，LNAV 回退到 GPS CDI，仍可工作。

扫描需要两台 CDU 中至少一台停在 LEGS 页，优先使用右 CDU 以保留机长 CDU。扫描期间插件会短暂翻页，结束后回到原页面。默认按**可见页面内容变化**触发；内容不变不重复扫描，默认没有定时扫描。`MOD` 页内容变化会扫一次但保留原 ACT 缓存；`EXEC`（ACT RTE）后再扫描替换。菜单 **Rescan CDU route** 可手动扫描；**Dump diagnostics** 只写日志、不触发扫描。

当前航点每 0.5 s 用 FF757 `AirnavDis` + `AirnavBrg` 同步；CDU LEGS 第 1 页第 1 行（下一个航点）双向权威；几何回退只在飞机真正沿出航方向越过该点时才推进。每次扫描都会在 `Output/757LVNAVPlus.log` 写 `CDU scan begin` / `CDU scan done`。

### 安装

1. **完全退出 X-Plane。**
2. 解压发布包，把 `64/` 复制到 757 机型目录（必要时新建 `plugins/757LVNAVPlus`）：

   ```
   X-Plane 12/Aircraft/<你的 757 机型目录>/plugins/757LVNAVPlus/
   ```

   最终结构：

   ```
   Boeing757-Full/
     plugins/
       757LVNAVPlus/
         64/
           win.xpl   (mac 为 mac.xpl，Linux 为 lin.xpl)
         docs/
         licenses/
         README.md
+        RELEASE_NOTES.md
         CHANGELOG.md
         LICENSE
         NOTICE.md
         THIRD_PARTY_NOTICES.md
   ```

   不要多套一层同名文件夹——`.xpl` 必须直接位于 `757LVNAVPlus/64/` 下。

3. 启动 X-Plane 并加载 FF757，`Plugins → 757 LNAV/VNAV Plus` 应出现。`Log.txt` 中应能看到：

   ```
   [757LVNAV+] 757 LNAV/VNAV Plus v1.2.36+cifp-nav+rnav1-bank starting
   [757LVNAV+] FF avionics datarefs: found
   ```

   可能出现"未签名插件"提示；若模拟器设置为阻止未签名插件，请在插件安全设置里放行。

4. 打开 **Plugins → 757 LNAV/VNAV Plus**，按需启用各层。

插件会生成：

```
X-Plane 12/Output/preferences/757LVNAVPlus.cfg
X-Plane 12/Output/757LVNAVPlus.log
```

删除配置文件即可恢复默认。

### 首次使用

1. 在 FF757 FMS 里装载航路，并让一台 CDU 停在 LEGS 页（优先右 CDU）。
2. 正常接通 LNAV/VNAV，从菜单打开状态窗。
3. 普通航段显示 `src=ROUTE`，notes 通常为 `direct bank ±xx.x (RNAV1)`；RF 段显示 `src=RF ARC` 和 `RFarc=..%`。如果直滚不可用或关闭，notes 会回到 `nav target + FD roll` / heading-direct 一类提示。航路行下面还会显示 `CIFP: <SID> <transition>, n/n fixes, n RF`。
4. 飞行中改航路后用菜单 **Rescan CDU route**。
5. 若你的构建报告的模式值与默认不符，调整配置里的 `lnav_roll_mode`、`vnav_pitch_mode_a/b/c`、`ap_cmd_mode` 后重新加载配置。

### 状态窗阅读

| 行 | 含义 |
| --- | --- |
| `AP : mode / roll / pitch / vnavPhase` | FF757 原始自动驾驶模式值 |
| `LNAV : ACTIVE src=ROUTE / RF ARC / ROUTE HOLD / GPS CDI` | LNAV 层状态与航路来源 |
| `CIFP: <SID> <transition>, n/n fixes, n RF` | 当前 CIFP 离场覆盖：程序名、过渡、解析航点数、RF 段数；无匹配时显示 `no CIFP departure match` |
| `base / target / corr` | 基准航向、目标航向、滚转修正 |
| `turn=..% / RFarc=..%` | 转弯进度 / RF 弧度进度 |
| `BankD=1`（Config 行） | 直滚模式开关状态；`1` 表示写绝对 FD 滚转角，`0` 表示航向直写/回退 |
| `active #n -> next #n` | 当前与下一航点索引 |
| `RNP=.. ANP(trk)=.. OK / NAV UNABLE RNP` | RNP 要求与航迹保持估计（`ANP(trk)` 是代理量，不是 FMS 的 GPS/IRS ANP） |
| `VNAV : PTH ACTIVE / SPD STBY` | FF757 VNAV 子模式；`ACTIVE` = 正在修正，`STBY` = 已识别但不干预 |
| `reqVS / vs / corr` | 所需升降率、实际升降率、俯仰修正 |
| `Notes` | 直滚/航向直写、RF 入弧/弧段、俯仰优先权、只读 dataref、保持状态 |

### 默认行为

- **LNAV** 开启：改善航向跟踪、提前转弯与 RF 弧。
- **VNAV** 开启：小幅垂直路径修正 + FD 俯仰平滑。
- **VNAV smoothing** 开启：无垂直路径时只平滑 FD 俯仰基线（不做修正）。
- **直滚默认开启**（`lnav_bank_direct=1`）：写绝对 FD 滚转角，限 25°、速率 6°/s、低通 0.10 s、转率阻尼 0.5；FD 滚转不可写时自动回退到航向直写。`lnav_heading_direct=0` 则直飞段交还机模、override 只用于 RF 段；`lnav_rf_nav_override=0` 让 RF 段退回 ±4° FD 滚转；`lnav_rf_enabled=0` 则 RF 段完全透传。
- **CIFP 离场覆盖**默认开启（`cifp_procedure_route=1`）；不匹配时自动使用原始 FMS 航路。
- 地面、AP 断开、未识别到导航/垂直模式、航路或 CDI 无效时自动待命。

### 安全限值（默认）

| 限值 | 默认 |
| --- | --- |
| LNAV 截获角（偏航距 ≥ `lnav_xtk_blend_nm` 时） | 30° |
| LNAV 直飞截获上限 | 45° |
| LNAV 航向误差钳位 | 60° |
| LNAV 提前转弯使用坡度 | 17°（≤180 KT）→18°（≥260 KT） |
| RF 出弧提前量 | 3° 弧度 |
| RF 航迹阻尼 | 0.35 × 航迹误差，上限 10° |
| RF 坡度前馈 | 按 `atan(V²/(gR))`，方向与目标航向误差相反时本帧禁用 |
| 直滚坡度上限 | 25° |
| 直滚指令速率 | 6°/s |
| 直滚低通 / 死区 | 0.10 s；0.5° / 0.02 NM |
| 直滚转率阻尼 | 0.5°/（°/s），上限 6° |
| VNAV 俯仰修正 | ±2.5° |
| VNAV 俯仰变化率 | 3°/s |
| VNAV 所需升降率 | ±3000 fpm |
| VNAV 平滑时间常数 | 0.12 s |
| VNAV 硬限速 | 10000 ft 以下 250 KT |
| VNAV 减速提前量 | 每 10 KT 提前 1 NM |

均可在 [`docs/CONFIG.md`](docs/CONFIG.md) 中调整；双语操作说明见 [`docs/USAGE.md`](docs/USAGE.md)。

### 菜单、命令与 dataref

菜单项：

- `Enable plugin`
- `LNAV enhancement`
- `VNAV enhancement`
- `VNAV smoothing`
- `LNAV FD roll (experimental)`
- `CIFP departure route`
- `LNAV bank direct (RNAV1)`
- `Rescan CDU route`
- `Show status window`
- `Reload config`
- `Reset defaults`
- `Dump diagnostics`
- `About`

可绑定命令：

| 命令 | 作用 |
| --- | --- |
| `757lvnv/toggle` | 总开关 |
| `757lvnv/toggle_lnav` | LNAV 增强开关 |
| `757lvnv/toggle_vnav` | VNAV 增强开关 |
| `757lvnv/toggle_fd_roll` | 实验性 LNAV FD 滚转开关 |
| `757lvnv/toggle_cdi_source` | LNAV 数据源在 route / GPS CDI 之间切换 |
| `757lvnv/status_window` | 显示/隐藏状态窗 |
| `757lvnv/dump` | 把 dataref 诊断写入 `Output/757LVNAVPlus.log` |

发布的 dataref（前缀 `757lvnv/`）：

| dataref | 类型 | 说明 |
| --- | --- | --- |
| `enabled` | int RW | 总开关 |
| `lnav_enabled` | int RW | LNAV 层 |
| `vnav_enabled` | int RW | VNAV 层 |
| `status` | int R | 0 关闭，1 待命，2 工作，3 非 FF757 |
| `lnav_active` | int R | LNAV 正在写入 |
| `vnav_active` | int R | VNAV 正在写入 |
| `route_valid` | int R | 航路模型有效 |
| `route_source` | int R | 0 无，1 私有 `rteData`，2 CDU LEGS 页 |
| `route_legs`, `route_wpts` | int R | 航路规模 |
| `active_wpt`, `active_leg`, `next_leg` | int R | 当前索引 |
| `ap_mode`, `roll_mode`, `pitch_mode`, `vnav_phase` | int R | FF757 原始模式值 |
| `ap_engaged`, `lnav_override` | int R | 检测状态（含插件在 RF 段持有的 override） |
| `xtk_nm`, `distance_nm` | float R | 偏航距 / 距离 |
| `base_course_deg`, `desired_course_deg` | float R | 航向目标 |
| `lnav_correction_deg` | float R | LNAV 修正量 |
| `target_alt_ft`, `required_vs_fpm`, `vs_error_fpm` | float R | VNAV 路径 |
| `vnav_correction_deg` | float R | VNAV 修正量 |
| `cdi_xtk_nm` | float R | CDI 原始偏航距 |

可用于 DataRefTool、硬件绑定或自定义 UI。

### 故障排查

- **菜单里没有插件**：当前发布包仅 Windows x64；确认放在**当前加载机型**的 `plugins/757LVNAVPlus/64/win.xpl`、复制时 X-Plane 已关闭、插件管理器中未禁用。检查 `Log.txt` 里的 `757LVNAVPlus` / `dlerror` / `Failed to load`。
- **`written=1` 但无效果**：X-Plane 按加载顺序执行插件回调。把文件夹改名为 `zzz_757LVNAVPlus` 后重启，让插件回调晚于 FF757 自身的写入。
- **CDU 未扫描**：必须有一台 CDU 停在 LEGS 页；插件不会从任意页面自动导航到 LEGS，装载航路后用 **Rescan CDU route**。
- **RF override 提示**：机模每帧把 `override_nav_heading` 清 0 是正常现象，插件会重新置位。若更希望只用滚转，设 `lnav_rf_nav_override=0`。
- **直滚没有跟随**：确认 Config 行 `BankD=1`，且日志中 `LNAV servo` 的 `bankCmd` 有值。若 FD roll 不可写，插件会回退到航向直写并在 notes 提示；此时可关闭直滚（菜单或 `lnav_bank_direct=0`）。
- **RF 段航向卡住**：先看 `tb=` 与 `ff=`。`ff=0` 表示前馈方向与目标航向误差相反、已忽略，属正常保护；若仍卡住，附上 `src=RF ARC` 那几行日志。
- **CIFP 没匹配**：检查最近机场是否在 60 NM 内、CIFP 文件是否存在，以及状态窗 `CIFP:` 行；程序从中间开始时至少要匹配到程序末尾 2 个航点。
- **`NAV UNABLE RNP`**：`ANP(trk)` 是相对本插件路径的航迹保持代理量，不是 FMS 的 GPS/IRS ANP；仅提示，不参与操纵。

### 从源码构建

需要 CMake 3.20+、C++17 编译器与 [X-Plane SDK](https://github.com/X-Plane/xpsdk)（未设置 `XPLANE_SDK_DIR` 时 CMake 会自动拉取）。

```bash
cmake -S . -B build -DXPLANE_SDK_DIR=/path/to/xpsdk -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

产物位于 `build/dist/64/`：`win.xpl`、`mac.xpl` 或 `lin.xpl`。复制到机型的 `plugins/757LVNAVPlus/64/`。

也可用 Zig 作为便携 C++ 编译器本地构建：

```powershell
zig c++ -std=c++17 -O2 -shared -DIBM=1 -DXPLM200=1 -DXPLM210=1 -DXPLM300=1 `
  -DXPLM301=1 -DXPLM303=1 -I xpsdk/CHeaders/XPLM -I xpsdk/CHeaders/Widgets `
  -I src src/plugin.cpp src/config.cpp src/route_reader.cpp src/guidance.cpp `
  src/cdu_parser.cpp src/cdu_reader.cpp src/nav_database.cpp src/cifp_parser.cpp `
  src/procedure_parser.cpp src/procedure_route.cpp src/bank_controller.cpp `
  xpsdk/Libraries/Win/XPLM_64.lib -o win.xpl
```

### 为什么不改 SASL

SASL 是机模使用的加密脚本环境。修改它脆弱、涉及许可，并且每次 FF757 更新都会失效。本插件刻意与机模**进程外隔离**：

- 它是 `plugins/757LVNAVPlus/64/` 下的普通 XPLM 插件；
- 从不加载、解密、替换或修补 `.secured` 文件；
- 只使用公开的 X-Plane dataref 和 avionics 插件已发布的 FF757 dataref；
- 删除一个文件夹即可移除。

因此它不能新增 FMS 页面或修改 FF757 CDU，只增强现有自动驾驶已经消费的引导。

### 限制与免责

- **实验性**：已编译并通过单元测试，但未在所有 FF757 变体上实飞验证；请先在可弃档或带安全员的情况下试飞。
- FF757 航路 dataref 未公开；以 CDI 校验为护栏。私有树缺失时改读 CDU LEGS 页（扫描会短暂翻页再恢复）。
- RF 弧依赖 X-Plane CIFP：CIFP 没有对应 RF 行时按普通直飞段处理。CIFP 不含跑道入口坐标，且跑道过渡与公共航段是分开的块，离场第一段可能从第一个可解析航点开始。
- CIFP SID 检测基于航点 ident：平行跑道的程序如果共享很长前缀可能并列选中，状态窗会显示实际选中的程序名；CIFP 周期也可能与 FF757 navdata 不同，使用前对照 FMS 航路。
- VNAV 效果取决于机模是否走旧版 FD 俯仰路径；若 `written=0`，VNAV 未到达伺服层，可只用 LNAV 或反馈 dataref 值。
- 若私有航路高度/`vtk` 不可用，VNAV 以 FD 俯仰平滑层运行（`src=FD SMOOTH`，修正量 0）。
- 与 FlightFactor、StepToSky、Laminar Research、Boeing 无关联；757/767 名称为各自商标。
- 风险自负：这是飞行模拟增强，不是认证导航或自动驾驶系统。

### 许可证

MIT，版权 **JACKGONG**。详见 [`LICENSE`](LICENSE)。

允许自由再分发（原始或修改版，含商业分发），但必须在文档、关于框或致谢中保留作者署名 **JACKGONG**，不得删除或替换。每次启动都会在 `Output/757LVNAVPlus.log` 记录该声明。完整双语声明见 [`NOTICE.md`](NOTICE.md)。第三方构建依赖与声明见 [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md)。

### 致谢

- **JACKGONG** — 757 LNAV/VNAV Plus 作者与维护者。
- FlightFactor / StepToSky — 757/767 Professional 系列。
- Laminar Research — X-Plane SDK。
- FF757 社区 — dataref 观测与试飞反馈。

测试后请附上状态窗内容与 `Log.txt` 中的 `[757LVNAV+]` 行，这是改进模式识别与增益默认值最快的方式。

---

## English

### What it does

The FlightFactor 757 has a deep custom FMS and autopilot. Its LNAV can cut
corners, overshoot turns or fly RF legs as polygons, its VNAV pitch can be
noisy, and it does not monitor RNP. This plugin adds a **supervisory guidance
layer** on top of the aircraft's own autopilot.

- **RF constant-radius arcs (CIFP-only)**: geometry comes from X-Plane
  `Custom Data/CIFP/<ICAO>.dat` ARINC 424 RF rows (centre, radius, left/right,
  sweep); coordinates are resolved through the nav database. The plugin flies
  the lead-in, captures the circle and tracks the tangent with a bounded radial
  correction. RF legs bypass the ordinary LNAV leg latch completely. RF bank
  feed-forward has a same-frame direction check: if its sign opposes the target
  course error, it is ignored for that frame (the IDBON lock case).
- **Lateral control: direct bank by default, heading-direct fallback**:
  `lnav_bank_direct=1` writes an absolute FD roll target (track error +
  cross-track, 25° cap). If FD roll is not writable or direct bank is disabled,
  the plugin falls back to `override_nav_heading=1` +
  `nav_steer_deg_mag` heading guidance (wind-correction angle plus track
  damping, ±10°). Only one lateral controller is active at a time. The model
  clearing the override every frame is normal and is re-asserted.
  `lnav_heading_direct=0` hands straight legs back to the FF757 and keeps the
  override RF-only; `lnav_rf_nav_override=0` uses the ±4° FD-roll nudge on RF.
  Small straight-leg offsets are closed by an L1 look-ahead term
  (`lnav_l1_factor`, 1.6 default).
- **LNAV capture and turns**: Boeing-style capture - a constant 30° intercept
  beyond 2.5 NM cross-track, direct homing `atan(xtk/dist)` below it; turn
  anticipation from groundspeed and bank limit; FMS course anchoring only
  within 8°; CDI validation needs ~2 s of sustained disagreement before
  downgrading, with `ROUTE HOLD` as the fallback.
- **Waypoint recognition**: CDU LEGS page 1 line 1 (the next waypoint) is
  authoritative in both directions, backed by `AirnavDis/AirnavBrg` sync and
  along-track sequencing. A leg is annotated as RF only when its
  entry→exit pair really is RF in the procedure database, so another SID's
  arc is never borrowed for a straight leg.
- **CIFP departure completion**: the active SID is matched from the route's
  first fixes against the nearest airport CIFP (60 NM). A route that starts
  mid-procedure may match with the last two SID fixes only. The procedure
  sequence, RF arcs and resolved coordinates are merged at the first common fix;
  `cifp_procedure_route=0` disables it. Runway thresholds are not in CIFP, and
  runway transitions are still separate CIFP blocks, so the very first
  departure leg may start at the first resolved fix.
- **RNAV1 direct bank**: `lnav_bank_direct=1` (default) writes an absolute FD
  roll target from track error + cross-track, capped at 25°, slewed at 6°/s,
  with a 0.5° / 0.02 NM deadband, 0.10 s low-pass and 0.5 track-rate damping.
  Fly-by/RF arcs publish a bank feed-forward; it is rejected when its sign
  opposes the current target-course error. Menu **LNAV bank direct (RNAV1)**
  toggles it.
- **VNAV path**: required vertical speed from constraints or the vertical path,
  vertical-acceleration damping, low-pass filtering and rate limiting into FD
  pitch; VNAV SPD (speed-on-pitch) passes through. During descent the plugin
  latches pitch priority (`vnav_priority`) so it cannot alternate with the
  FF757's own VNAV. CDU altitude qualifiers are honoured (`A` = at or above,
  `B` = at or below, plain = at); a satisfied at-or-above floor is not a
  descent target, but it still limits the descent gradient so the path cannot
  cross it below its minimum.
- **VNAV speed limits**: 250 KT below 10000 ft and a 1 NM per 10 KT decel lead
  (`vnav_speed_limits_enabled` turns it off); the automatic 250 KT target arms
  only inside the deceleration window, not tens of miles early at cruise.
- **RNP/ANP monitoring**: RNP by phase (2.0 enroute / 1.0 terminal / 0.3 final),
  `ANP(trk)` as a track-keeping proxy; sustained exceedance shows
  `NAV UNABLE RNP` and logs it (monitoring only).
- **Route source**: private FF757 `rteData` first; otherwise the CDU LEGS page
  (right CDU preferred) is scanned for idents, altitudes, speeds and distances,
  then the page is restored.
- **Performance**: navdata is indexed once per session (16-byte records, hard
  cap ~10 MB) with binary-search lookups; no disk scans in the flight loop.
- **Observable and optional**: the status window shows modes, source, XTK,
  target course, RF progress, RNP/ANP and notes; every layer can be disabled
  from the menu, config or datarefs.

### How it works

```
FF757 avionics (read only)                 Standard X-Plane (write only)
┌────────────────────────────────┐         ┌─────────────────────────────────┐
│ route arrays (rteData/...)     │         │ nav_steer_deg_mag               │
│ or CDU LEGS page (fallback)    │  ────►  │ override_nav_heading (RF)       │
│ AirnavDtk / AirnavDis / ...     │         │ flight_director_pitch / roll    │
│ 757Avionics/ap/* modes         │         │ override_flightdir_ptch/roll    │
│ FD pitch / roll (cockpit2)     │         │                                 │
└────────────────────────────────┘         └─────────────────────────────────┘
                 ▲                                        │
                 │                                        ▼
          ┌──────┴──────────────────────────────────────────────────┐
          │  757 LNAV/VNAV Plus (this plugin, separate XPLM DLL)   │
          │  - route parser + CDU/navdata resolution               │
          │  - RF arc geometry from X-Plane CIFP                  │
          │  - course capture, turn anticipation, RF tracking      │
          │  - VNAV path VS error + acceleration damping           │
          │  - RNP/ANP monitoring, rate limits, mode gating        │
          └─────────────────────────────────────────────────────────┘
```

| Layer | Reads | Writes | Effect |
| --- | --- | --- | --- |
| **Route** | private FF757 route arrays when exposed; otherwise the CDU LEGS page via public CDU display datarefs | CDU page keys only while scanning, then restores the page | Waypoint coordinates, altitudes, speeds, distances, active/next leg |
| **LNAV** | route geometry, active-leg data, GPS CDI, FD roll, track/heading | default `flight_director_roll` + `override_flightdir_roll=1` (absolute bank, direct-bank mode); fallback `override_nav_heading=1` + `nav_steer_deg_mag` heading | Smoother, turn-anticipating target with bank held through the turn |
| **RF arcs** | X-Plane CIFP `Custom Data/CIFP/<ICAO>.dat` RF rows; coordinates resolved via the nav database | same direct-bank/heading path; RF feed-forward with same-frame direction check | Plugin-flown lead-in and true constant-radius arc; RF legs bypass the ordinary LNAV latch |
| **VNAV** | route constraints, vertical path, FD pitch, MCP altitude | `flight_director_pitch` + `override_flightdir_ptch`; cockpit2 FD pitch only if writable | Damped, rate-limited vertical-path correction; SPD passes through |
| **RNP/ANP** | route geometry | nothing | Status/log monitoring only |

The plugin is a **pass-through** by design: it writes only when the FF757
autopilot is engaged and the corresponding mode is detected. VNAV never
initiates a climb/descent against the MCP altitude and stays inside a ±2.5°
pitch limit. Missing datarefs make that layer idle, leaving the aircraft exactly
as it was.

No aircraft files, SASL scripts, `.secured` modules or FF binaries are read or
modified. The only `757Avionics/*` datarefs written are the CDU page keys used
while a route scan is running.

### Compatibility

- **X-Plane 12** (primary target). XP11.30+ may work (only stable XPLM APIs),
  but XP11 is not tested.
- **FlightFactor 757 v2** (all -200/-300/F variants using `757767Avionics`).
  The 767 shares the avionics and is detected too; it is not the primary target.
- The current release (`757LVNAVPlus-win-1.2.36-rnav1-bank.zip`) is **Windows
  x64**. macOS/Linux can be built from source.

The FF757 private route datarefs are **undocumented**. They are read
best-effort and validated against the standard GPS CDI. If a build does not
expose the private route tree (the FF757 v2 build tested here does not), the
plugin reads the CDU **LEGS page** instead: it captures displayed waypoint
idents, altitudes, speeds and leg distances, resolves coordinates from the
FF757 navdata, and restores the CDU page it started from. If even the CDU
display is unavailable, LNAV falls back to the GPS CDI and remains usable.

A route scan needs one of the two CDUs to show a LEGS page when scanning starts.
The right CDU is preferred so the captain's CDU stays free. While scanning, the
plugin briefly pages through the LEGS pages and returns the CDU to the page it
found. By default the scanner triggers on **visible page content changes**; the
same content is never rescanned until it changes, and there is no periodic
timer by default. A MOD page is scanned once when its content changes, but the
previous ACT cache is kept; the EXEC (`ACT RTE`) content then replaces it.

The active waypoint is synchronised from `AirnavDis` + `AirnavBrg` every 0.5 s;
CDU LEGS page 1 line 1 is authoritative in both directions; the geometric
fallback advances only when the aircraft is genuinely past a fix along the
outbound leg. Every scan writes `CDU scan begin` / `CDU scan done` to
`Output/757LVNAVPlus.log`.

### Installation

1. **Close X-Plane completely.**
2. Unzip the release and copy the `64/` folder into your 757 aircraft folder
   (create `plugins/757LVNAVPlus` if needed):

   ```
   X-Plane 12/Aircraft/<your Boeing 757 folder>/plugins/757LVNAVPlus/
   ```

   Final layout:

   ```
   Boeing757-Full/
     plugins/
       757LVNAVPlus/
         64/
           win.xpl   (or mac.xpl / lin.xpl)
         docs/
         licenses/
         README.md
+        RELEASE_NOTES.md
         CHANGELOG.md
         LICENSE
         NOTICE.md
         THIRD_PARTY_NOTICES.md
   ```

   Do not add an extra nested folder with the same name - the `.xpl` must be
   directly inside `757LVNAVPlus/64/`.

3. Start X-Plane, load the FF757, and check `Log.txt` for:

   ```
   [757LVNAV+] 757 LNAV/VNAV Plus v1.2.36+cifp-nav+rnav1-bank starting
   [757LVNAV+] FF avionics datarefs: found
   ```

   X-Plane may show an unsigned-plugin warning; allow this plugin in the
   security settings if you block unsigned plugins.

4. Open **Plugins → 757 LNAV/VNAV Plus** and enable the layers you want.

The plugin creates:

```
X-Plane 12/Output/preferences/757LVNAVPlus.cfg
X-Plane 12/Output/757LVNAVPlus.log
```

Delete the config file to restore defaults.

### First flight

1. Load a route in the FF757 FMS and leave one CDU on a LEGS page (right CDU
   preferred) so the plugin can scan it.
2. Engage LNAV/VNAV normally and open the status window.
3. On a normal leg `src=ROUTE`, usually with the note
   `direct bank ±xx.x (RNAV1)`; on an RF leg `src=RF ARC` with
   `RFarc=..%`. If direct bank is unavailable/disabled the note falls back to
   heading-direct wording. The route block also shows
   `CIFP: <SID> <transition>, n/n fixes, n RF`.
4. Use **Rescan CDU route** after changing the route while running.
5. If your build reports different raw mode values, adjust
   `lnav_roll_mode`, `vnav_pitch_mode_a/b/c` or `ap_cmd_mode` and reload.

### Reading the status window

| Line | Meaning |
| --- | --- |
| `AP : mode / roll / pitch / vnavPhase` | Raw FF757 autopilot mode values |
| `LNAV : ACTIVE src=ROUTE / RF ARC / ROUTE HOLD / GPS CDI` | LNAV state and path source |
| `CIFP: <SID> <transition>, n/n fixes, n RF` | Active CIFP departure overlay, resolved fixes and RF count; `no CIFP departure match` when none |
| `base / target / corr` | Base course, commanded course, roll correction |
| `turn=..% / RFarc=..%` | Turn progress / RF arc progress |
| `BankD=1` (Config line) | Direct-bank switch: `1` = absolute FD roll target, `0` = heading-direct/fallback |
| `active #n -> next #n` | Active and next waypoint indices |
| `RNP=.. ANP(trk)=.. OK / NAV UNABLE RNP` | RNP requirement vs track-keeping estimate (`ANP(trk)` is a proxy, not the FMS GPS/IRS ANP) |
| `VNAV : PTH ACTIVE / SPD STBY` | FF757 VNAV submode; `ACTIVE` = correcting, `STBY` = recognised but hands off |
| `reqVS / vs / corr` | Required VS, actual VS, pitch correction |
| `Notes` | Direct-bank / heading-direct, RF lead-in/arc, pitch priority, read-only datarefs, holds |

### Default behaviour

- **LNAV** enabled: course tracking, turn anticipation and RF arcs.
- **VNAV** enabled: small vertical-path correction plus FD pitch smoothing.
- **VNAV smoothing** enabled: filters the FD pitch baseline when no path exists.
- **Direct bank** enabled (`lnav_bank_direct=1`): absolute FD roll target,
  25° cap, 6°/s slew, 0.10 s low-pass, 0.5 track-rate damping. Falls back to
  heading-direct when FD roll is not writable. `lnav_heading_direct=0` hands
  straight legs back to the FF757 and keeps the override RF-only;
  `lnav_rf_nav_override=0` uses the ±4° FD-roll path on RF;
  `lnav_rf_enabled=0` passes RF legs through.
- **CIFP departure overlay** enabled (`cifp_procedure_route=1`); an unmatched
  route stays on the original FMS route.
- Idle on the ground, with AP off, when the mode is not detected, or when
  route/CDI data is invalid.

### Safety limits (defaults)

| Limit | Default |
| --- | --- |
| LNAV intercept (beyond `lnav_xtk_blend_nm`) | 30° |
| LNAV direct intercept cap | 45° |
| LNAV course error clamp | 60° |
| LNAV bank for turn anticipation | 17° (≤180 KT) → 18° (≥260 KT) |
| RF roll-out lead | 3° of arc |
| RF track damping | 0.35 × track error, max 10° |
| RF bank feed-forward | `atan(V²/(gR))`; suppressed when its sign opposes the current course error |
| Direct-bank limit | 25° |
| Direct-bank slew | 6°/s |
| Direct-bank filter / deadband | 0.10 s; 0.5° / 0.02 NM |
| Direct-bank track-rate damping | 0.5° per °/s, max 6° |
| VNAV pitch correction | ±2.5° |
| VNAV pitch rate | 3°/s |
| VNAV required VS | ±3000 fpm |
| VNAV smoothing time constant | 0.12 s |
| VNAV hard speed limit | 250 KT below 10000 ft |
| VNAV decel lead | 1 NM per 10 KT |

All configurable - see [`docs/CONFIG.md`](docs/CONFIG.md); operator guide in
[`docs/USAGE.md`](docs/USAGE.md).

### Menu, commands and datarefs

Menu items:

- Enable plugin
- LNAV enhancement
- VNAV enhancement
- VNAV smoothing
- LNAV FD roll (experimental)
- CIFP departure route
- LNAV bank direct (RNAV1)
- Rescan CDU route
- Show status window
- Reload config
- Reset defaults
- Dump diagnostics
- About

Commands (bindable in X-Plane):

| Command | Action |
| --- | --- |
| `757lvnv/toggle` | Toggle the whole plugin |
| `757lvnv/toggle_lnav` | Toggle LNAV enhancement |
| `757lvnv/toggle_vnav` | Toggle VNAV enhancement |
| `757lvnv/toggle_fd_roll` | Toggle the experimental LNAV FD-roll bridge |
| `757lvnv/toggle_cdi_source` | Toggle the LNAV source between route and GPS CDI |
| `757lvnv/status_window` | Show/hide the status window |
| `757lvnv/dump` | Write a dataref diagnostic dump to `Output/757LVNAVPlus.log` |

Published datarefs (prefix `757lvnv/`):

| Dataref | Type | Notes |
| --- | --- | --- |
| `enabled` | int RW | Master switch |
| `lnav_enabled` | int RW | LNAV layer |
| `vnav_enabled` | int RW | VNAV layer |
| `status` | int R | 0 disabled, 1 idle, 2 active, 3 not FF757 |
| `lnav_active` | int R | LNAV writing |
| `vnav_active` | int R | VNAV writing |
| `route_valid` | int R | Route model validated |
| `route_source` | int R | 0 none, 1 private `rteData`, 2 CDU LEGS page |
| `route_legs`, `route_wpts` | int R | Route size |
| `active_wpt`, `active_leg`, `next_leg` | int R | Active indices |
| `ap_mode`, `roll_mode`, `pitch_mode`, `vnav_phase` | int R | Raw FF757 values |
| `ap_engaged`, `lnav_override` | int R | Detected state (`lnav_override` includes an RF override held by the plugin) |
| `xtk_nm`, `distance_nm` | float R | Cross-track / distance |
| `base_course_deg`, `desired_course_deg` | float R | Course target |
| `lnav_correction_deg` | float R | LNAV correction |
| `target_alt_ft`, `required_vs_fpm`, `vs_error_fpm` | float R | VNAV path |
| `vnav_correction_deg` | float R | VNAV correction |
| `cdi_xtk_nm` | float R | Raw CDI cross-track |

Useful for DataRefTool, hardware bindings or a custom UI.

### Troubleshooting

- **Menu entry missing**: current release is Windows x64 only; make sure the
  plugin is in the *currently loaded* aircraft's
  `plugins/757LVNAVPlus/64/win.xpl`, that X-Plane was restarted after copying,
  and that it is not disabled in Plugin Admin. Search `Log.txt` for
  `757LVNAVPlus` / `dlerror` / `Failed to load`.
- **`written=1` but no effect**: flight loops run in load order. Rename the folder
  to `zzz_757LVNAVPlus` and restart so the callback runs after the FF757's own
  write.
- **CDU not scanned**: one CDU must be on a LEGS page; use **Rescan CDU route**
  after loading the route.
- **RF override note**: the model clearing `override_nav_heading` every frame is
  normal; the plugin re-asserts it. Set `lnav_rf_nav_override=0` for the ±4°
  FD-roll path.
- **Direct bank not following**: check `BankD=1` in the Config line and a
  non-zero `bankCmd` in `LNAV servo`. If FD roll is not writable the plugin
  falls back to heading-direct and says so in the notes; disable direct bank
  from the menu or `lnav_bank_direct=0` if the model fights it.
- **RF heading appears stuck**: check `tb=` and `ff=`. `ff=0` means the
  feed-forward sign opposed the target-course error and was correctly ignored.
  Include the `src=RF ARC` lines in a report if it still does not turn.
- **CIFP not matched**: check the airport is within 60 NM, the CIFP file
  exists, and the status `CIFP:` line. A route starting mid-procedure needs at
  least the last two SID fixes to match.
- **`NAV UNABLE RNP`**: `ANP(trk)` is a track-keeping proxy against our own path,
  not the FMS GPS/IRS ANP; it monitors only and never commands.

### Build from source

Requirements: CMake 3.20+, a C++17 compiler, and the
[X-Plane SDK](https://github.com/X-Plane/xpsdk) (fetched automatically when
`XPLANE_SDK_DIR` is not set).

```bash
cmake -S . -B build -DXPLANE_SDK_DIR=/path/to/xpsdk -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

The binary is written to `build/dist/64/`: `win.xpl`, `mac.xpl` or `lin.xpl`.
Copy it into `plugins/757LVNAVPlus/64/`.

Local Zig build:

```powershell
zig c++ -std=c++17 -O2 -shared -DIBM=1 -DXPLM200=1 -DXPLM210=1 -DXPLM300=1 `
  -DXPLM301=1 -DXPLM303=1 -I xpsdk/CHeaders/XPLM -I xpsdk/CHeaders/Widgets `
  -I src src/plugin.cpp src/config.cpp src/route_reader.cpp src/guidance.cpp `
  src/cdu_parser.cpp src/cdu_reader.cpp src/nav_database.cpp src/cifp_parser.cpp `
  src/procedure_parser.cpp src/procedure_route.cpp src/bank_controller.cpp `
  xpsdk/Libraries/Win/XPLM_64.lib -o win.xpl
```

### Why no SASL changes

SASL is the encrypted scripting environment used by the aircraft. Modifying it
would be fragile, license-sensitive, and would break on every FF757 update.
This plugin is deliberately **out-of-process with respect to the aircraft**:

- it is a normal XPLM plugin in `plugins/757LVNAVPlus/64/`;
- it never loads, decrypts, replaces or patches `.secured` files;
- it uses only public X-Plane datarefs and the FF757 datarefs the avionics
  plugin already publishes;
- it can be removed by deleting one folder.

It cannot add new FMS pages or change the FF757 CDU. It improves the guidance
the existing FF757 autopilot already consumes.

### Limitations and disclaimer

- **Experimental.** Compiled and unit-tested, but not flight-tested in every
  FF757 variant. Fly with a safety pilot or in a disposable session first.
- FF757 route datarefs are undocumented; CDI validation is the guardrail. If
  the private tree is missing, the CDU LEGS page is read instead (the scanner
  briefly pages the CDU and restores it).
- RF arcs depend on the X-Plane CIFP database. If the CIFP file has no RF row,
  the leg is flown as a normal straight leg. CIFP has no runway-threshold
  coordinates, and runway transitions/common legs are separate CIFP blocks, so
  the first departure leg may begin at the first resolved fix.
- CIFP SID detection is ident-based: parallel-runway procedures with a long
  shared prefix can tie, and the status `CIFP:` line shows the procedure that
  was actually selected. The CIFP cycle can also differ from the FF757 navdata;
  cross-check the FMS route before use.
- VNAV effectiveness depends on the FF757 using the legacy FD pitch path. If
  `written=0`, VNAV is not reaching the servo layer; use LNAV or report dataref
  values.
- If the private route altitude/`vtk` data is unavailable, VNAV remains active
  as an FD-pitch smoothing layer (`src=FD SMOOTH`, correction 0).
- Not affiliated with FlightFactor, StepToSky, Laminar Research or Boeing.
  Boeing 757/767 names are trademarks of their respective owners.
- Use at your own risk. This is a flight-simulation enhancement, not a
  certified navigation or autopilot system.

### License

MIT, copyright **JACKGONG**. See [`LICENSE`](LICENSE).

This plugin may be redistributed freely - original or modified, including
commercial redistribution - but the author name **JACKGONG** must stay credited
in the documentation, about box, or credits, and may not be removed or replaced.
The startup line in `Output/757LVNAVPlus.log` records this notice on every run.
The full bilingual statement is in [`NOTICE.md`](NOTICE.md). Third-party build
dependencies and notices are listed in
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).

### Credits

- **JACKGONG** — author and maintainer of 757 LNAV/VNAV Plus.
- FlightFactor / StepToSky for the 757/767 Professional series.
- Laminar Research for the X-Plane SDK.
- The FF757 community for dataref observations and flight-testing feedback.

If you test this plugin, please include the status-window contents and the
`[757LVNAV+]` lines from `Log.txt` in your report. That is the fastest way to
make mode detection and gain defaults better for everyone.
