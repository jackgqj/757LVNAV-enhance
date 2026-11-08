# 1.2.22 VNAV 高度约束限定符修复

状态：**1.2.22 已重新开发、构建并安装**（2026-11-07）。实现副本同时保存在
[`docs/archive/vnav_fix_1.2.22/`](archive/vnav_fix_1.2.22/)，作为 1.2.22
参考实现与回滚备份。本文记录问题、根因、改动与验证步骤。

## 现象

VNAV PTH 下降时飞机不跟随 FMC 剖面（状态窗口长期显示较大的
`reqVS` 与实际 `vs` 偏差，例如 `reqVS=-4037`、`vs=-2950`、
`corr=-0.87°`，插件一度把修正压到 -2.37°）。

## 日志证据

`Output/757LVNAVPlus.log` / `Log.txt` 中同一次下降：

```
CDU|AA146         299/FL157A      (43 NM)
CDU|AA145         299/FL138A      (22 NM)
CDU|AA144         299/11765       (14 NM)
CDU|AA143         240/ 6890B      (26 NM)
```

```
[757LVNAV+] guidance: targetAlt=15700 reqVs=-4037
  notes=<VNAV profile to AA146, 26.2 NM, 15700 ft | speed 250 kt @ 89 NM (auto 250/10000)>
[757LVNAV+] VNAV authority: source=ROUTE PROFILE corr=-2.37 written=1
  note=cockpit2 FD is read-only; legacy FD only | plugin pitch priority
```

FL340、距 AA146 43 NM 时，若把 `FL157A` 当成“必须精确到 15700 ft”，
几何直线要求约 4000 fpm；实际 FMC/飞机按慢车剖面约 2950 fpm。
插件持续低头修正，反而把飞机压得比 FMC 剖面更陡。

## 根因

CDU LEGS 页的高度带限定符：

| CDU 显示 | 含义 |
| --- | --- |
| `FL157A` / `6890A` | **at or above**（下限） |
| `6890B` / `FL200B` | **at or below**（上限） |
| `11765`（无后缀） | 精确通过（at） |

旧版解析（`parseAltitudeString` / `ParseCduAltitude`）在第一个非数字字符
处截断，`A`/`B` 被丢掉，所有高度都被当成“精确通过”。于是“已经满足的
at-or-above 下限”被当成必须下降到的目标，产生物理上不可飞、也不必要的
下降率，并与 FF757 自身的 VNAV 抢俯仰。

## 改动内容（1.2.22 实现）

### 1. 数据模型

`src/route_model.h` 新增枚举与字段：

```cpp
enum AltConstraint { AltAt = 0, AltAtOrAbove = 1, AltAtOrBelow = 2 };

struct RouteWaypoint {
    ...
    AltConstraint altConstraint = AltAt;
};
```

`src/cdu_parser.h` 的 `CduLeg` 同样增加 `AltConstraint altConstraint = AltAt;`。

### 2. CDU 文本解析

`src/cdu_parser.cpp` 新增（在匿名 namespace 内）：

```cpp
// "FL157A" = at or above, "6890B" = at or below, "11765" = at.
AltConstraint ParseAltQualifier(const std::string& text) {
    std::string up = Upper(TrimCdu(text));
    if (up.find("ABOVE") != std::string::npos) return AltAtOrAbove;
    if (up.find("BELOW") != std::string::npos) return AltAtOrBelow;
    size_t lastDigit = std::string::npos;
    for (size_t i = 0; i < up.size(); ++i)
        if (std::isdigit((unsigned char)up[i])) lastDigit = i;
    if (lastDigit == std::string::npos) return AltAt;
    size_t j = lastDigit + 1;
    while (j < up.size() && IsSpace(up[j])) ++j;
    if (j < up.size()) {
        if (up[j] == 'A') return AltAtOrAbove;
        if (up[j] == 'B') return AltAtOrBelow;
    }
    return AltAt;
}
```

`ParseSpeedAltField(..., AltConstraint& altType)` 多输出一个限定符：
右侧字段解析成功时取右侧文本的限定符，左右互换时取左侧文本的限定符。
`ParseCduLegsPage` 在写 `leg.altFt` 的同时写 `leg.altConstraint`；
顶部 “NM 之后的硬限制”分支同样写入 `leg.altConstraint`。限定符变化也会
进入 `PageSignature`，触发重新扫描。

### 3. 私有 rteData 航路

`src/route_reader.cpp` 新增 `parseAltConstraint()`，逻辑同上；
`parseAltitudeString(input, altType)` 多输出限定符；
浮点数组（无文本）一律按 `AltAt`；字符串缓冲区按后缀解析。

### 4. VNAV 目标选择

`src/guidance.cpp` 的 `FindVnavTarget` 循环中加入：

```cpp
if (!w->altValid) continue;
if (std::abs((double)w->altFt - state.altFt) < 100.0) continue;
if (cum < minDistance) continue;
// 已满足的 at-or-above 下限不是下降目标
if (w->altConstraint == AltAtOrAbove && w->altFt < state.altFt) continue;
// 已满足的 at-or-below 上限不需要爬升
if (w->altConstraint == AltAtOrBelow && w->altFt > state.altFt) continue;
```

即：下降时跳过“飞机已经在它上方”的 at-or-above 下限，改为取下一个
精确/at-or-below 点，所需下降率回到可飞范围。

### 4a. at-or-above 下限的坡度保护

跳过的下限不是目标，但直线飞向下一个硬限制时仍可能提前穿过该点且低于
其最低高度。`VnavTarget` 记录“最严格的一个下限”，`computeVnav` 在算出
`finalRequired` 后把下降率限制到“恰好在该点达到下限高度”的坡度：

```cpp
bool floorClamped = false;
if (target.floorValid && finalRequired < 0.0) {
    const double floorTimeMin =
        target.floorDistanceNm / std::max(state.gsKts, 60.0) * 60.0;
    const double floorAllowed =
        -(state.altFt - (double)target.floorAltFt) /
        std::max(floorTimeMin, 1.0e-3);
    if (finalRequired < floorAllowed) {
        finalRequired = floorAllowed;
        floorClamped = true;
    }
}
```

触发时状态窗口备注追加 `| at-or-above floor kept`。

### 5. 自动 250 KT / 10000 ft

`FindSpeedTarget` 中，自动 250 kt 不再按“航点距离”判定，而是按飞机
自身的减速窗口：

```cpp
const double to10k = std::max(0.0, state.altFt - cfg.vnavHard250BelowFt) / 318.0;
const double autoLead = leadFactor > 0.0
    ? leadFactor * std::max(0.0, curKt - 250.0) / 10.0 : 0.0;
if (to10k <= autoLead + 1.0) { limit = 250; autoFlag = true; }
```

避免巡航时出现 “250 kt @ 106 NM”。

## 测试

1.2.22 实现下三套测试全部通过：

| 测试 | 检查数 | 结果 |
| --- | --- | --- |
| `guidance_tests` | 168 | 0 failures |
| `cdu_tests` | 39 | 0 failures |
| `cifp_tests` | 18 | 0 failures |

新增覆盖：

- `cdu_tests`：`FL197A` → `AltAtOrAbove`；`6890B` → `AltAtOrBelow`；
  无后缀 → `AltAt`。
- `guidance_tests`：已满足的 `FL157A` 被跳过，目标变为下一个
  `11765`，`requiredVs` 落在可飞范围；同样几何下改成精确通过时需求更陡。
- `guidance_tests`：被跳过的 at-or-above 下限仍限制下降坡度
  （`-852 fpm` 对无下限时的 `-2222 fpm`），备注包含
  `at-or-above floor kept`。
- `guidance_tests`：FL330、低高度航点仍在 45 NM 外时不产生 250 kt 目标。

## 附：1.2.22 崩溃修复（Proc 文件解析越界）

首版 1.2.22 装在 ZSPD→ZSNJ 航路后崩溃。X-Plane 把崩溃归到本插件，
Windows 事件日志给出的 fault offset 为 `win.xpl +0x2a015`；用匹配的
`win.pdb` + DbgHelp 解析为：

```
symbol: lvnv::`anonymous namespace'::Trim +0xa5
line:   src\cifp_parser.cpp:21
```

根因在 `CifpDatabase::ParseFfProcLines`：

```cpp
const std::vector<std::string> p = SplitComma(line);
if (p.empty()) { prevIdent.clear(); continue; }   // 永远为 false
...
prevIdent = Upper(Trim(p[1]));                    // 单字段行越界
```

`SplitComma("")` 返回一个含空串的 vector，`p.empty()` 从不成立；FF757 的
`Custom Data/STSFF/navdata/Proc/<ICAO>.txt` 用空行分隔程序块，于是第一
个空行就让 `p[1]` 读到 vector 之外，得到一个内容损坏的 `std::string`，
`Trim` 读取其长度/指针时访问违规。ZSPD.txt（3510 行）与 ZSNJ.txt（827
行）都含大量空行，必然触发。

修复：

```cpp
if (p.size() < 2) { prevIdent.clear(); continue; }
```

空行/短行现在按“块结束”处理（同时避免跨程序拼接 RF 入口点），并加入
回归测试：在 FF Proc 测试数据中插入空行、单字段 `RF` 与任意文本，解析
结果仍为 4 条 RF 弧。

验证：用修复后的解析器直接读取真实
`Custom Data/STSFF/navdata/Proc/ZSPD.txt` 与 `ZSNJ.txt`，正常完成
（3510/827 行，0 条 RF 弧，无崩溃）。三套测试为 168 / 39 / 18。

## 附：VNAV 俯仰权限与“高于剖面 1630 ft”

实测：`PTH ACTIVE`、`reqVS=-3078`、`vs=-1920`，PFD/ND 垂直偏差
1630 ft 高于 FMC 剖面。日志显示插件在 `ROUTE PROFILE` 与 `FD PASS`
之间反复切换：

```
VNAV authority: source=ROUTE PROFILE ... corr=-0.35 ... plugin pitch priority
VNAV authority: source=FD PASS ... vertical path within deadband; FF757 owns pitch
VNAV authority: source=ROUTE PROFILE ... corr=-0.33 ...
```

原因：旧死区在 `|vsError| < 150 fpm` 时立即交出俯仰；FF757 用自己的
（更浅的）VNAV 速率接管，偏差重新变大后插件再次接管。净下降率仍停留在
FF757 的速率，飞机长期高于剖面。

改动：

- `Guidance` 新增 `vnavPathHeld_` / `vnavPathErrorFt_`。后者把
  `vsError * dt / 60` 积分成高度偏差（负值 = 高于剖面，限 ±6000 ft），
  目标航点变化或模式切换时清零。
- 俯仰权限带迟滞：接管需要 `|vsError| > 200 fpm` 或偏差 `> 250 ft`；
  释放需要 `|vsError| < 80 fpm` **且** 偏差 `< 120 ft`。软化的 DECEL
  段始终保留权限。
- `computeVnav` 增加 `dtSec` 参数（测试默认 0，不积分）。
- 插件每 5 s 在 `Output/757LVNAVPlus.log` 写一条 `VNAV servo` 诊断：
  `corr`、我们写入的 FD 俯仰 `out`、legacy 回读、cockpit2 FD、override
  `/flightdir`、X-Plane AP 与伺服状态、`override_artstab` /
  `artstab_pitch_ratio`、`vs` / `req` / `pathErr` / `held`。这条日志用于
  确认俯仰写入是否真正生效，以及是 X-Plane AP 还是机模 artstab 在控制俯仰。

验证（新测试）：

- 500 fpm 误差持续 30 s → 接管并累计约 250 ft。
- 之后 VS 与剖面一致，但 250 ft 偏差仍存在 → 继续保持权限。
- 反向修正把偏差抹平后 → 释放为 `FD PASS`。
- 单帧 150 fpm 误差（新实例）→ 不接管。

### 4b. “before TOD” 只在平飞时成立

同一现象的另一个来源：状态窗显示 `PTH STBY` / `src=ROUTE HOLD`、
`reqVS=+0`、`vs=-1619`、`written=0`，而 PFD FMA 明确是 `VNAV PTH`，
下一个限制 ESBAG 11900 ft 在 31.2 NM 外。旧逻辑用 318 ft/NM 估算
“3° 需要 23.6 NM”，认为还在 TOD 之前就保持高度透传；但飞机已经在下降，
FMC 的垂直路径是活的。

修正：`beforeTod` 追加 `|vsFpm| < 200 ft/min` 条件——只有真正平飞时才
保持；一旦已经开始下降，就按路径计算所需下降率（该例中
`req ≈ -1541 fpm`，与实际 -1619 接近，进入释放死区，仍安全透传；
若高于剖面则立即接管修正）。

新增测试：同样的 before-TOD 几何、`vs=-1500` 时不再保持，而是
`pathBased`，目标为下一个限制，`reqVS ≈ -741 fpm`。

### 4c. 进近阶段交还俯仰

`VNAV servo` 日志末尾出现 `servos=0`、`ovrArtstab=1`，说明 FF757 已经
接管俯仰（进近/自动落地），而插件仍在 `held=1` 下写入 `out=8.88°/7.43°`
的 FD 俯仰。现在插件在该状态持续约 1 秒（20 帧）后释放权限并清空
override，不再与机模争抢。

### 4d. 日志实测结论（220 条 `VNAV servo`）

- **写入生效**：`legacyRead = out = fdC2`，`ovrFd=1`、`apOn=1`、`servos=1`、
  `ovrArtstab=0`；俯仰命令确实进入 X-Plane AP，`cockpit2` 与 legacy 同值。
- **下降段跟随良好**：典型 `vs=-674/req=-669`、`vs=-686/req=-677`，
  修正量仅 ±0.05°；`pathErr` 长期在 -150 ft 左右（略高于剖面），
  相比之前的 1630 ft 已大幅收敛。
- **仍有大偏差的时段**：`pathErr` 范围 -342 ~ +1412 ft。多数来自高度截获
  （`req≈-1` 而 `vs=-1460`，插件抬头把下降率拉平）和航点切换瞬间；
  积分在目标切换处已清零，但截获段的暂态仍会积累。
- **出现爬升指令**：末尾 `req=+658/+300`。该航路含明显错误的航点
  （`FF24`/`ZSNJ24` 被解析到 -23°、`(660)`/`(4930)` 被当成航点 ident，
  其中一条高度 66000 ft），目标高度跳变会让插件要求爬升。属于 CDU
  航路解析/导航数据匹配问题，需单独修复。

### 4e. VNAV 响应偏慢

实测 `reqVS=-802`、`vs=-1747`（误差 945 fpm）时修正只有 `+0.75°`。
在 343 KT 下 1° 俯仰约对应 600 fpm，所以 0.75° 只能抵消约 450 fpm，
不到误差的一半，表现为“响应慢、压不住”。

物理反推：`deg ≈ vsError / (1.767 × GS)`。400 KT 时约 1.41°/1000 fpm，
343 KT 时约 1.65°/1000 fpm。旧增益 0.8°/1000 fpm 明显偏软。

调整默认值：

- `vnav_vs_gain`：0.8 → **1.2**（约为物理反推的 85% @ 400 KT）
- `vnav_smoothing_tau_s`：0.12 → **0.10**

同时更新用户配置文件 `Output/preferences/757LVNAVPlus.cfg` 中的这两项，
否则已保存的旧值会覆盖默认值。`vnav_vs_gain` 的可调范围仍是 0–2，
嫌快可下调到 1.0，嫌慢可到 1.4。

### 4f. 离谱的 reqVS 来自“假航点”

实测状态窗：`PTH ACTIVE src=ROUTE/MCP targetAlt=3000 ft`、
`reqVS=-43 fpm`、`vs=-1783 fpm`，备注为
`VNAV profile to FI07, 2241.8 NM`。2241.8 NM 外的目标显然是航点被错误
解析（同名 fix 在世界另一处），于是用“高度差 / 2241.8 NM”算出的 -43 fpm
毫无意义。

两处防护：

1. `NavDatabase::resolve`：同名候选中离当前航路超过 **500 NM** 的 fix
   视为导航数据同名冲突，不再采用（`out[i]` 保持无效，CDU 航路读取器会
   用页面上的航向/距离合成该航段）。
2. `FindVnavTarget` / `FindSpeedTarget`：沿航路累计距离超过 **500 NM**
   即停止搜索，不再把几千里外的“限制”当成 VNAV 目标。

新增测试：目标位于 720 NM 外时不再 `pathBased`，备注
`no usable altitude target`，回退为透传。

## 构建产物（1.2.22，含崩溃修复）

- `win.xpl`：1,116,672 字节
- SHA256：`194FD04D6E09C6F128A9D2DFFC0D25D953C6EF81BC68E0754BAF1066A225AE22`
- 安装位置：`plugins\757LVNAVPlus\64\win.xpl`
- 发布 zip：513,198 字节
- zip SHA256：`AD89474D2DA62228978F7705663C44EBCD90DC46AE4A9B2B8D5BAE9EE9D553D8`
  （包内 DLL 与上面一致）

> 中间版本（限定符 + 坡度保护，但 Proc 解析仍会崩溃）为
> `2164FD8A5B94C380F04D829D23B7402EAB8A383A01E774DF81839BA964C2D1A0`；
> 含崩溃修复、但无权限迟滞的版本为
> `782B19AE53AD27B45D40062CDD04522895280A063445070A9E4E001A9E29CCBD`；
> 含权限迟滞、但 before-TOD 仍会在下降中透传的版本为
> `A62E3CA15A6321F667B5F6B6D1A1A0B1E8EA038539BA40CCF96BA32FC7661DB2`；
> 含 before-TOD 修复、但进近阶段未交还俯仰的版本为
> `43985412F15046382992C2F436EB3F77B0F79D2786A7C8109FB425544546648C`；
> 含进近交还俯仰、但 VNAV 增益仍为 0.8 的版本为
> `0A23DC42F9E4666C0D8DE5B6F7B9EF9DAF4E5DD30D8F221B99C299E355F244F3`；
> 增益已调整、但未过滤远处假航点的版本为
> `111DE964C1FE41BA6B7A4DE531FD53888B01276051BCF451364839A935F6EB31`。
> 以上均已作废，不要使用。

## 回滚与归档

1.2.22 已在 `docs/archive/vnav_fix_1.2.22/` 保存一份与当前工作区一致的实现
（`src/` 6 个文件、两个测试文件、`CHANGELOG.md`、`CMakeLists.txt`，版本号
1.2.22）。如需再次回滚到 1.2.21：

1. 关闭 X-Plane。
2. 从 `64\win.xpl` 读取当前 1.2.21 二进制并备份（安装中的 DLL 允许读取）。
3. 恢复 1.2.21 源文件；`version.h`、`CMakeLists.txt`、`CHANGELOG.md` 同步回
   1.2.21。
4. 重新跑三套测试，检查数应回到 guidance 145 / CDU 35 / CIFP 18。
5. 将 1.2.21 二进制复制回 `64\win.xpl`、`_tools\lvnv_build\win.xpl` 与
   `E:\FF757 enhance\64\win.xpl`，重建 zip 并核对包内 DLL 哈希。

再次启用 1.2.22 时，把归档文件覆盖回工作区并改版本号为 1.2.22，重新编译、
跑测试、安装即可。

编译与测试命令（zig 工具链）：

```powershell
$root='<plugin root>'
$zig="$root\_tools\zig-x86_64-windows-0.14.1\zig.exe"
& $zig c++ -std=c++17 -O1 -I"$root\src" "$root\tests\guidance_tests.cpp" `
  "$root\src\guidance.cpp" "$root\src\config.cpp" -o "$root\_tools\lvnv_build\guidance_tests.exe"
& $zig c++ -std=c++17 -O1 -I"$root\src" "$root\tests\cdu_tests.cpp" `
  "$root\src\cdu_parser.cpp" -o "$root\_tools\lvnv_build\cdu_tests.exe"
& $zig c++ -std=c++17 -O1 -I"$root\src" "$root\tests\cifp_tests.cpp" `
  "$root\src\cifp_parser.cpp" "$root\src\nav_database.cpp" -o "$root\_tools\lvnv_build\cifp_tests.exe"
```

## 仍需飞行验证

- 真实下降中 `reqVS` 是否与 FMC 路径一致，状态窗口是否不再长时间出现
  4000 fpm 级别的需求。
- 若仍出现“跟剖面上/下偏差”，需结合 PFD 的 V/DEV 和 `targetAlt` 判断是
  目标选择还是俯仰权限问题（FD pitch 写入被 FF757 覆盖、或慢车/自动油门
  限制）。
- 自动 250 kt 的减速窗口阈值（`to10k <= autoLead + 1.0`）可按实际体验微调。
- 下限坡度保护使用 318 ft/NM 作为“到 10000 ft”的近似，若有实际 VNAV
  爬升/下降率数据，可改为按当前梯度计算。
