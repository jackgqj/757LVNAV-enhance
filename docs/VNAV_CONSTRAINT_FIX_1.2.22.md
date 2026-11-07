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
| `guidance_tests` | 158 | 0 failures |
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
（3510/827 行，0 条 RF 弧，无崩溃）。三套测试仍为 158 / 39 / 18。

## 构建产物（1.2.22，含崩溃修复）

- `win.xpl`：1,114,112 字节
- SHA256：`782B19AE53AD27B45D40062CDD04522895280A063445070A9E4E001A9E29CCBD`
- 安装位置：`plugins\757LVNAVPlus\64\win.xpl`
- 发布 zip：511,019 字节
- zip SHA256：`303614C8CE806F90EE85DDCAF51750C21385D60FB374A59E20D8FC7B08ABD99B`
  （包内 DLL 与上面一致）

> 中间版本（限定符 + 坡度保护，但 Proc 解析仍会崩溃）为
> `2164FD8A5B94C380F04D829D23B7402EAB8A383A01E774DF81839BA964C2D1A0`，
> 已作废，不要使用。

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
