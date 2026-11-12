# 757LVNAVPlus 交接文档（DeepSeek Harness）

生成日期：2026-11-10（LNAV 转弯修订）
当前构建：`1.2.36+vnav-profile+lnav-turn`
文档目的：把当前工作区、构建/测试/安装方法、VNAV 剖面和 RNAV 离场 LNAV 转弯改动，以及已知限制交接给下一个在 DeepSeek Harness 中接手的人或 AI。

---

## 1. 当前状态

- 这是一个 FlightFactor 757/767 的外部制导插件，不修改机模 SASL/核心文件，只通过 XPLM 插件和公开/已发布的 dataref 工作。
- 本次在上一个基线 **1.2.36** 上新增了 Zibo 风格的 VNAV 垂直剖面（`vnav_path_profile`），旧的“追下一个高度约束”逻辑完整保留作为回退。
- 单元测试：guidance **222**、CDU **48**、CIFP **21**，全部 0 失败。
- 发布包已生成：`E:\FF757 enhance\757LVNAVPlus-win-1.2.36-vnav-profile-lnav-turn.zip`（当前构建）和旧的 `757LVNAVPlus-win-1.2.36-vnav-profile.zip`（仅 VNAV 剖面）。
- `E:\FF757 enhance\64\win.xpl`（源码树副本）已是本次构建。
- 机模安装位置的 `win.xpl` 已是上一构建（17/18° 坡度，SHA256 见 5.3）；2026-11-11 试飞时 X-Plane 正在运行，源码树副本已恢复为同一二进制。25° 实验已回退。
- 运行配置已启用剖面（`vnav_path_profile=1`）。
- 尚未进行真实飞行验证。

---

## 2. 工作区与关键路径

### 2.1 源码与构建

| 用途 | 绝对路径 |
| --- | --- |
| **源码工作区根目录** | `E:\FF757 enhance` |
| 源码 | `E:\FF757 enhance\src\` |
| 单元测试 | `E:\FF757 enhance\tests\` |
| 文档 | `E:\FF757 enhance\docs\` |
| 归档的旧版本源码/文档 | `E:\FF757 enhance\docs\archive\` |
| Zig 编译器 | `E:\FF757 enhance\_tools\zig-x86_64-windows-0.14.1\zig.exe` |
| X-Plane SDK | `E:\FF757 enhance\_tools\xpsdk\SDK` |
| 构建输出目录 | `E:\FF757 enhance\_tools\lvnv_build\` |
| 本次构建的二进制 | `E:\FF757 enhance\_tools\lvnv_build\win.xpl` |
| 源码树内的已安装副本 | `E:\FF757 enhance\64\win.xpl` |
| 本次发布包 | `E:\FF757 enhance\757LVNAVPlus-win-1.2.36-vnav-profile.zip` |
| 1.2.36 源/二进制备份 | `E:\757LVNAVPlus_backup_v1.2.36_20261108` |

### 2.2 机模、安装与运行

| 用途 | 绝对路径 |
| --- | --- |
| **目标机模（FF757）** | `E:\SteamLibrary\steamapps\common\X-Plane 12\Aircraft\Laminar Research\Boeing757-Full` |
| **插件安装位置** | `E:\SteamLibrary\steamapps\common\X-Plane 12\Aircraft\Laminar Research\Boeing757-Full\plugins\757LVNAVPlus\64\win.xpl` |
| 插件随附文档 | `...\Boeing757-Full\plugins\757LVNAVPlus\docs\` |
| 运行时配置 | `E:\SteamLibrary\steamapps\common\X-Plane 12\Output\preferences\757LVNAVPlus.cfg` |
| 插件日志 | `E:\SteamLibrary\steamapps\common\X-Plane 12\Output\757LVNAVPlus.log` |
| X-Plane 主日志 | `E:\SteamLibrary\steamapps\common\X-Plane 12\Output\Log.txt` |
| 参考机模（Zibo 737-800X 4.05.33） | `E:\SteamLibrary\steamapps\common\X-Plane 12\Aircraft\Laminar Research\B737-800X` |

> 注意：配置文件在 X-Plane 全局 `Output/preferences` 下，不是机模目录内。删除该文件会恢复代码默认值，剖面默认是关闭的。

### 2.3 Zibo VNAV 参考代码位置（只读参考，不要复制其 dataref 结构）

- `E:\SteamLibrary\steamapps\common\X-Plane 12\Aircraft\Laminar Research\B737-800X\plugins\xlua\scripts\B738.a_fms\B738.a_fms.lua`
  - `create_idle_path`（约 L153564）、`recalc_vnav`（约 L141454）
  - `calc_vnav_pth_alt/dist`（约 L152387/L152448）、`B738_vnav_pth3`（约 L157710）
  - `B738_calc_vnav_spd`（约 L163099）、`B738_vnav_desc_spd`（约 L164243）
- 真正的 LNAV 转弯/航段排序和 AFDS 伺服在闭源 `zibomod.xpl` 内，无源码，不要试图从 Lua 找。

---

## 3. 代码地图

| 文件 | 作用 |
| --- | --- |
| `src/plugin.cpp` | XPLM 入口、飞行循环、dataref 读写、FF757 模式识别、FD 俯仰/滚转写入、状态窗口、日志 |
| `src/guidance.cpp/.h` | `computeLnav` / `computeVnav`，航路几何、RF 弧、RNP/ANP、VNAV 剖面与目标逻辑 |
| `src/route_reader.cpp/.h` | 读取 FF757 航路 dataref；CDU LEGS 缓存回退 |
| `src/cdu_reader.cpp` + `cdu_parser.cpp` | CDU 屏幕扫描、LEGS 页解析、航点/高度/速度约束提取 |
| `src/cifp_parser.cpp` + `nav_database.cpp` | CIFP 解析（RF 弧几何）与导航数据库内存索引 |
| `src/config.cpp/.h` | `757LVNAVPlus.cfg` 读写与取值范围裁剪 |
| `tests/guidance_tests.cpp` | 制导单元测试（无 XPLM 依赖） |
| `tests/cdu_tests.cpp` / `tests/cifp_tests.cpp` | CDU 解析 / CIFP 解析测试 |

---

## 4. 本次 VNAV 改动（重点）

### 4.1 思路

用一条连续的慢车下降剖面替代“对下一个高度约束求弦线”：

1. 从航路中**最后一个有高度约束的航点**（E/D）开始；
2. 按 `vnav_path_angle_deg`（默认 3°，约 318 ft/NM）向飞机方向反推；
3. 每个约束成为折点：
   - `AltAt`：精确压在该高度；
   - `AltAtOrAbove`：不低于该高度（地板）；
   - `AltAtOrBelow`：不高于该高度（天花板）；
   - 强制剖面单调下降（前面的点不能低于后面的点）；
4. 航点排序后，把刚飞过的最近一个 `AltAt` 约束保留为**负距离锚点**，避免换段时路径跳回 3°；
5. 飞机当前位置查表得到 `pathAlt` 和局部梯度，`reqVS = 梯度 × GS/60 − devGain × 路径偏差`。

### 4.2 代码位置

| 内容 | 位置 |
| --- | --- |
| `BuildVnavProfile()` 剖面构建 | `E:\FF757 enhance\src\guidance.cpp` 约 L240 |
| `VnavProfileAt()` 高度/梯度查表 | 同文件约 L352 |
| `computeVnav()` 中的剖面分支（opt-in） | 同文件约 L1273 |
| 新增配置字段 | `E:\FF757 enhance\src\config.h` 约 L147–151 |
| 配置解析/保存/裁剪 | `E:\FF757 enhance\src\config.cpp` L164、L255、L89–90 |
| 新增测试 | `E:\FF757 enhance\tests\guidance_tests.cpp` 约 L390–519 |

### 4.3 行为细节

- **TOD 前**：剖面在当前点高于飞机超过 50 ft 且飞机接近平飞时，输出 `ROUTE HOLD` 并 `passThrough`，俯仰交还 FF757。
- **E/D 后**：保持最后一个约束高度；不会继续追一个已经飞过的航点。
- **MCP 高度**：作为下降下限。仅当飞机在 MCP 之上、且剖面低于 MCP 时才夹紧，不会从 MCP 下方开始俯冲。
- **不爬升**：下降剖面中 `reqVS > 0` 会被钳到 0（平飞等待剖面追上来）。
- **俯仰权限滞回**：沿用旧逻辑——进入需要 `|VS 误差| > 200 fpm` 或 `|路径偏差| > 250 ft`；退出需要两者都小于 80 fpm / 120 ft。
- **减速**：只在距离速度约束 `max(5 NM, decelLead + 2 NM)` 以内软化梯度；未实现 Zibo 的完整 decel 平飞段。
- **回退条件**：剖面关闭、爬升、没有高度约束、末端梯度 >1000 ft/NM（约 9.4°，多为误解析高度）、活动航点已经在飞机身后时，自动走旧目标逻辑。

### 4.4 配置项

```ini
vnav_path_profile=1        # 1=启用剖面；代码默认 0
vnav_path_angle_deg=3      # 名义慢车下降角，范围 1–6
vnav_path_dev_gain=0.6     # 每 1 ft 路径偏差增加的 fpm，范围 0–3
```

当前运行配置 `E:\SteamLibrary\steamapps\common\X-Plane 12\Output\preferences\757LVNAVPlus.cfg` 已写入以上三项。

### 4.5 RNAV 离场 LNAV 转弯修订（`lnav-turn`）

现场日志（ZSHC RWY25 SUPAR8D）显示 FF757 会在高速时提前约 1 分钟切换 FMS 活动航点：HC306→HC606 在 HC306 前 9.4 NM（285 KT），SUPAR→OKTUG 在 SUPAR 前 12.2 NM（461 KT）。旧代码立即按新航段用 30° 截获，导致转弯提前开始、切弯，随后因截获量衰减而浅坡度、过冲。

本次改动（`src/guidance.cpp`）：

- **折衷航段锁**：FMS 提前切换后，继续飞前一航段直到飞机越过共享航点；该航段自己的 fly-by 圆弧负责转弯。锁是“粘性”的，同一次高速转弯里 FMS 连续切换两段也不会把弯丢在半路。`lnav_turn_latch=0` 可恢复旧行为。
- **闭环 fly-by 弧**：在原切向指令上叠加径向误差项 `1.5 × asin(离圆距离 / 半径)`（限 ±25°）。飞机掉到圆弧外侧时自动增加转弯量/坡度，不再只给切向角而放任坡度变浅、过冲。
- **随速度调度的转弯坡度**：`lnav_turn_bank_deg`（默认 17°）→ `lnav_turn_bank_high_deg`（默认 18°），在 `lnav_turn_bank_low_kt`（180）与 `lnav_turn_bank_high_kt`（260）之间线性插值，匹配 AFDS 高速减小坡度限制的特性。
- **转弯窗口暂停 CDI 校验**：FMS 在换段瞬间 CDI 航向/距离跳变，旧逻辑会持续 2 s 后降级到 GPS CDI，正好把 `turn=0` 的直线截获留在弯里。下段航向差 >5° 且距航点 <20 NM 时跳过 CDI 校验。
- **换段截获量保持**：换段瞬间 `atan(xtk/距离)` 因距离跳到下一段长度而骤降；现在把换段时的截获量保持最多 3 s 线性衰减，只在新航段仍有 >0.3 NM 偏差时生效。
- **L1 截获量上限提高**：新增 `lnav_l1_max_deg`（默认 8°，原硬编码 ±4°）。高速转弯后偏差 5–6 NM 时，4° 的 L1 附加量会饱和，30° 基础截获在到下个航点前压不回来；8° 让过冲回收更有力，小偏差工况仍远低于上限、不会引起摆头。
- **CDU 重扫保留前一航段**：`cdu_reader.cpp` 的 `finishScan` 现在会在新页面集前面补回最多 2 条旧航段（按新首航点在整个旧缓存中的位置查找），避免转弯中扫描把入航段丢掉、让折衷航段锁无几何可用（现场：SUPAR 扫描后 active=0、wpt[0]=SUPAR）。
- **首段占位航点防护**：拒绝 `(500)` / `(520)` 这类私有航路起点占位符作为航段起点，改用 FMS 航向的合成入航段（`ROUTE/SYNTH`）。
- 日志 `LNAV servo` 增加 `bank=`（实际坡度，来自 `roll_AHARS_deg_pilot`），用于下次试飞核对坡度。
- 测试：guidance **222**（原 203），CDU 48，CIFP 21，全部 0 失败。

---

## 5. 构建、测试、打包、安装

### 5.1 编译插件

```powershell
$root='E:\FF757 enhance'
$zig="$root\_tools\zig-x86_64-windows-0.14.1\zig.exe"
$sdk="$root\_tools\xpsdk\SDK"
& $zig c++ -std=c++17 -O2 -shared -DIBM=1 -DXPLM200=1 -DXPLM210=1 `
  -DXPLM300=1 -DXPLM301=1 -DXPLM303=1 `
  -I"$sdk\CHeaders\XPLM" -I"$sdk\CHeaders\Widgets" -I"$root\src" `
  "$root\src\plugin.cpp" "$root\src\config.cpp" "$root\src\route_reader.cpp" `
  "$root\src\guidance.cpp" "$root\src\cdu_parser.cpp" "$root\src\cdu_reader.cpp" `
  "$root\src\nav_database.cpp" "$root\src\cifp_parser.cpp" `
  "$sdk\Libraries\Win\XPLM_64.lib" -o "$root\_tools\lvnv_build\win.xpl"
```

### 5.2 编译并运行测试

```powershell
& $zig c++ -std=c++17 -O1 -I"$root\src" "$root\tests\guidance_tests.cpp" `
  "$root\src\guidance.cpp" "$root\src\config.cpp" -o "$root\_tools\lvnv_build\guidance_tests.exe"
& $zig c++ -std=c++17 -O1 -I"$root\src" "$root\tests\cdu_tests.cpp" `
  "$root\src\cdu_parser.cpp" -o "$root\_tools\lvnv_build\cdu_tests.exe"
& $zig c++ -std=c++17 -O1 -I"$root\src" "$root\tests\cifp_tests.cpp" `
  "$root\src\cifp_parser.cpp" "$root\src\nav_database.cpp" -o "$root\_tools\lvnv_build\cifp_tests.exe"

& "$root\_tools\lvnv_build\guidance_tests.exe"
& "$root\_tools\lvnv_build\cdu_tests.exe"
& "$root\_tools\lvnv_build\cifp_tests.exe"
```

期望输出：`222 checks, 0 failures`、`48 checks, 0 failures`、`21 checks, 0 failures`。

### 5.3 安装

X-Plane 必须先关闭（否则 `win.xpl` 被占用）：

```powershell
Copy-Item "E:\FF757 enhance\_tools\lvnv_build\win.xpl" `
  "E:\SteamLibrary\steamapps\common\X-Plane 12\Aircraft\Laminar Research\Boeing757-Full\plugins\757LVNAVPlus\64\win.xpl" -Force
Copy-Item "E:\FF757 enhance\_tools\lvnv_build\win.xpl" `
  "E:\FF757 enhance\64\win.xpl" -Force
```

本次构建哈希：

- 大小：`1,137,152` bytes
- SHA256：`4C3046CA06E852E49668263F79102008B8B31EED864FEF35CE52EEB714E62456`
- 源码树副本 `E:\FF757 enhance\64\win.xpl` 与机模安装位置均已是该哈希（2026-11-11 从上一构建原样恢复；25° 实验已回退）。注意：用当前源码重新编译可能得到不同的二进制哈希，但 17/18° 坡度、锁与 L1 行为一致。

### 5.4 打包结构

发布包沿用之前的布局（顶层 `757LVNAVPlus\`）：

```text
757LVNAVPlus\
  CHANGELOG.md
  LICENSE
  NOTICE.md
  README.md
  THIRD_PARTY_NOTICES.md
  64\win.xpl
  docs\AIRCRAFT_STRUCTURE.md
  docs\CONFIG.md
  docs\DATAREFS.md
  docs\HANDOFF_DEEPSEEK_HARNESS.md
  docs\USAGE.md
  licenses\libcxx-LICENSE.txt
```

---

## 6. 运行与验证

### 6.1 状态窗口/日志里应该看到

| 场景 | `source` | 行为 |
| --- | --- | --- |
| 正常沿剖面下降 | `VNAV PATH` | 写 FD 俯仰 |
| 被 MCP 高度夹住 | `VNAV PATH/MCP` | 下降到 MCP 后保持 |
| TOD 之前 | `ROUTE HOLD` | 不写俯仰，交还 FF757 |
| 偏差在死区内 | `FD PASS` | 不写俯仰 |
| VNAV SPD | `VNAV SPD PASS` | 完全不写俯仰 |
| 剖面不可用但 VNAV 已接通 | `VNAV PASS` | 交还 FF757 |

日志关键字：

- `VNAV pitch priority latched: reqVS=... vs=... targetAlt=...`
- `VNAV pitch priority released`
- `VNAV servo: corr=... req=... pathErr=... held=...`
- `VNAV write: corr=... out=... reqVS=... vs=...`（需 `debug=1`）

### 6.2 飞行验证要点

1. DES 阶段 `reqVS` 应约等于 `高度差 / (距离 / GS)`，且随剖面连续变化，不出现 4000 fpm 级别的尖峰。
2. 过每个高度约束时 V/DEV 应收敛，不应出现换段瞬间的坡度跳变。
3. 修改 MCP 高度应进入 `VNAV PATH/MCP` 或 `MCP HOLD`，不会穿过 MCP。
4. 排序前后（例如过 IAF）路径梯度应连续，不应从弦线突然跳到 3°。
5. 若出现俯仰拉扯，先看日志里 priority 是否反复 latch/release。

---

## 7. 已知限制与风险

1. **源码基线是 1.2.36**。`1.2.37`–`1.2.50` 的源码不在此工作区；`docs/archive` 里只有 37/38/39 的部分快照。不要假设那些版本里的修复（航段锁、CDI 重对齐、转弯窗口等）存在。
2. `vnav_path_profile` 在代码里默认 **0**；本次运行配置手工设为 1。删除 `757LVNAVPlus.cfg` 会关闭剖面。
3. 活动航点已经在飞机身后时，剖面返回不可用并回退旧目标逻辑；最后一个航点飞过后可能显示 `ROUTE HOLD`，这是安全回退，不是指令错误。
4. 尚未实现 Zibo 的完整 decel 平飞段/速度表；目前只有减速窗口内的梯度软化。
5. `zibomod.xpl` 是闭源二进制，只能通过 dataref 和日志观察；Zibo 的 LNAV 转弯不在 Lua 里。
6. `plugin.cpp` 没有单元测试覆盖，改动飞行循环/写入逻辑后必须实机验证。
7. 版本号是构建标识 `1.2.36+vnav-profile`，不是一个正式发布号；正式发版前需要负责人确定版本号。

---

## 8. 硬性约束（不要违反）

- 不修改 FF757 机模核心文件、SASL 脚本或 `.secured` 文件；只做外部 XPLM 插件。
- 不依赖未公开的内部 dataref（如 `rteData`、`dumpRte1Now`）作为核心功能。
- `cifp_parser.cpp` 中 `ParseFfProcLines` 的 `p.size() < 2` 单字段行防护是**受保护机制**：未经明确许可，禁止删除、弱化或重构（包括绕过它的 `p[1]` 访问）。它用于防止 Proc 文件解析启动崩溃。
- 导航数据库预索引内存不超过 10 MB。
- RNP 航段不得参考 FF757 原生 FMS 计算的航路。
- 修改后必须跑三套测试，并在安装前确认 X-Plane 已退出。

---

## 9. 下一步建议

1. 真实航班验证 DES：重点看 `VNAV PATH` 的 `reqVS`、V/DEV 收敛、换段连续性、MCP 夹紧。
2. 验证通过后把 `vnavPathProfile` 代码默认改为 `true`，并更新 `docs/CONFIG.md` 的默认值说明。
3. 如需更接近 Zibo：补 decel 平飞段（把减速点并入剖面）、速度计划表、E/D 飞过后的明确检测。
4. 与负责人确认正式版本号后，再改 `src/version.h`、`CMakeLists.txt` 和 `CHANGELOG.md` 并重新打包。

---

## 10. 交接快速自检

```powershell
# 1) 测试
& "E:\FF757 enhance\_tools\lvnv_build\guidance_tests.exe"
& "E:\FF757 enhance\_tools\lvnv_build\cdu_tests.exe"
& "E:\FF757 enhance\_tools\lvnv_build\cifp_tests.exe"

# 2) 二进制哈希（源码树与安装位置应一致）
Get-FileHash "E:\FF757 enhance\64\win.xpl" -Algorithm SHA256
Get-FileHash "E:\SteamLibrary\steamapps\common\X-Plane 12\Aircraft\Laminar Research\Boeing757-Full\plugins\757LVNAVPlus\64\win.xpl" -Algorithm SHA256

# 3) 运行配置是否启用剖面
Select-String -Path "E:\SteamLibrary\steamapps\common\X-Plane 12\Output\preferences\757LVNAVPlus.cfg" -Pattern "vnav_path"

# 4) X-Plane 是否在运行（安装前必须为空）
Get-Process X-Plane -ErrorAction SilentlyContinue
```

当前期望哈希：`4C3046CA06E852E49668263F79102008B8B31EED864FEF35CE52EEB714E62456`
（源码树副本与机模安装位置均已对齐，见 5.3）
