# 757 LNAV/VNAV Plus

 NEW request to ALL developer who wanna help this program:LNAV need some update，if anyone make the LNAV better，please contact me:Gongqijun823@outlook.com

**An unofficial X-Plane plugin that improves LNAV and VNAV guidance for the FlightFactor 757/767 — without touching SASL, the aircraft files, or the FF avionics plugin.**

> 这是一个独立的 X-Plane 插件，用来增强 FlightFactor 757/767 的 LNAV / VNAV 表现。
> 它**不修改 SASL**、不改飞机文件、也不改 FF 的 avionics 插件；它只读取 FF757 公开的
> 数据引用，并写入 X-Plane 标准的导航/飞行指引目标数据引用。

---

## What it does

The FlightFactor 757 has a deep custom FMS and autopilot. In some situations its
LNAV can overshoot a turn, hunt the course, or fail to anticipate the next leg,
and its VNAV pitch command can be noisy. This plugin adds a small **supervisory
guidance layer** on top of the aircraft's own autopilot:

| Layer | What it reads | What it writes | Effect |
| --- | --- | --- | --- |
| **Route** | private FF757 route arrays when the build exposes them; otherwise the CDU LEGS page through the public CDU display datarefs | CDU page keys (`757Avionics/CDU{,2}/prev_page` / `next_page`) only while scanning the LEGS page, then restores the original page | Gives LNAV/VNAV real waypoint coordinates, altitudes, and active/next leg data |
| **LNAV** | route geometry, active-leg data, GPS CDI, FD roll | `sim/cockpit/autopilot/nav_steer_deg_mag` as a target; optional FD-roll bank correction (`flight_director_roll` + `override_flightdir_roll`, **off by default**) | A smoother, turn-anticipating target for the FF757's existing LNAV mode |
| **VNAV** | route altitude constraints, vertical path, FD pitch, MCP selected altitude | `sim/cockpit/autopilot/flight_director_pitch` + `override_flightdir_ptch`; `sim/cockpit2/autopilot/flight_director_pitch_deg` only if the build accepts writes | A damped, rate-limited vertical-path correction blended into the existing FD pitch. In VNAV SPD (climb) the plugin passes pitch through unchanged, because that mode holds speed with pitch |

The plugin is a **pass-through** by design:

- It only writes guidance when the FF757 autopilot is engaged and its LNAV/VNAV
  mode is detected.
- LNAV publishes an improved course target every frame while the FF757 is in
  LNAV. On builds where `override_nav_heading=1` the aircraft consumes that
  target directly; on this build the optional FD-roll bridge provides the bank
  authority.
- VNAV only writes while a VNAV pitch mode is detected, and only inside a
  hard correction limit (default ±2.5° pitch). In VNAV SPD/climb it passes the
  FF757 pitch command through untouched; in path modes it assists a vertical
  path the aircraft is already flying and never initiates a climb or descent
  against the MCP selected altitude. Level-off, MCP hold, pre-TOD and
  small-error (<150 fpm) cases also pass through, so the plugin cannot turn
  into a pitch/AoA feedback loop.
- If any required dataref is missing, the corresponding layer goes idle and
  the aircraft continues to fly exactly as it did before.

No aircraft files, SASL scripts, `.secured` modules, or FF binaries are read or
modified. The only `757Avionics/*` datarefs written are the CDU page keys used
to page through the LEGS display while a route scan is running.

---

## Compatibility

- **X-Plane 12** (primary target). XP11.30+ may work because the plugin uses
  only stable XPLM APIs, but XP11 is not tested.
- **FlightFactor 757 v2** (all -200/-300/F variants that use the
  `757767Avionics` plugin). The 767 shares the same avionics and is detected
  too; it is not the primary target.
- Windows, macOS and Linux builds are provided by CI. The local build in this
  repository was produced on Windows.

The FF757 route datarefs used here are **undocumented**. They are read through
best-effort binding and always validated against the standard GPS CDI. If a
build does not expose the private route tree at all (the FF757 v2 build tested
here does not), the plugin reads the CDU **LEGS page** instead: it captures the
displayed waypoint idents, altitudes, and leg distances, resolves the
coordinates from the FF757 navdata, and restores the CDU page it started from.
If even the CDU display is unavailable, LNAV falls back to the GPS CDI and
remains usable.

A route scan needs one of the two CDUs to be showing a LEGS page when the
plugin starts scanning. The right-hand CDU is preferred so the captain's CDU
stays free; if only the left CDU is on LEGS, that one is used. While scanning,
the plugin briefly pages through the LEGS pages and then returns the CDU to the
page it found.

By default the scanner triggers on **visible LEGS page content changes**: it
compares the displayed page with the cache and scans once when idents,
altitudes, or leg distances change. The same page content is never rescanned
until it changes again, and there is no periodic timer by default
(`cdu_periodic_rescan=0`, `cdu_rescan_interval_s=0`). A MOD page is scanned
once when its content changes, but the
scanner keeps the previous ACT cache because a MOD route is not yet active; the
EXEC (`ACT RTE`) content then scans and replaces the cache. Use the menu
**Rescan CDU route** for a manual scan; **Dump diagnostics** only writes the log
and never starts a scan. Set `cdu_scan_on_exec_only=1` if you prefer to skip
MOD-page content changes and scan only after EXEC.

The CDU parser reads the live FF757 format: the line above a waypoint is
`COURSE + DISTANCE`, and the waypoint line itself carries the speed/altitude
estimate (for example `EKIMU  .806/FL315`). Every scan writes `CDU scan begin` /
`CDU scan done` to `Output/757LVNAVPlus.log`. A partial scan never replaces a
complete cached route, and the scanner cannot remain stuck in the restore state.

The active waypoint is additionally synchronised from the FF757's own
`AirnavDis` + `AirnavBrg` every 0.5 s. Matching both distance and bearing can
jump several legs and works even while a CDU is left on a page other than
LEGS page 1. This keeps LNAV route-based after sequencing instead of falling
back to GPS-CDI steering.

---

## Installation

1. Download the release zip for your platform (or build from source).
2. Copy the `64/` folder into:

   ```
   X-Plane 12/Aircraft/Laminar Research/Boeing757-Full/plugins/757LVNAVPlus/
   ```

   The final layout should look like:

   ```
   Boeing757-Full/
     plugins/
       757LVNAVPlus/
         64/
           win.xpl   (or mac.xpl / lin.xpl)
         README.md
         LICENSE
   ```

3. Start X-Plane, load the FF757, and check the X-Plane `Log.txt` for:


   ```
   [757LVNAV+] 757 LNAV/VNAV Plus v1.1.1 starting
   [757LVNAV+] FF avionics datarefs: found
   [757LVNAV+] route datarefs: ...
   ```

   X-Plane 12 may show an unsigned-plugin warning for community builds. This
   is expected; if your simulator is configured to block unsigned plugins,
   allow this plugin in the plugin security settings.

4. Open **Plugins → 757 LNAV/VNAV Plus** and enable the layers you want.

The plugin creates:

```
X-Plane 12/Output/preferences/757LVNAVPlus.cfg
X-Plane 12/Output/757LVNAVPlus.log
```

Delete the config file to restore defaults.

---

## Usage

A complete bilingual (中文 / English) step-by-step guide is in
[`docs/USAGE.md`](docs/USAGE.md).

1. Load a route in the FF757 FMS and leave one CDU on a LEGS page (the right
   CDU is preferred) so the plugin can scan it.
2. Engage LNAV/VNAV normally. The status window shows the detected FF757 modes,
   route source (`rteData` or `CDU`), route state, cross-track, desired course,
   required vertical speed, and the applied correction.
3. Use the menu to toggle the whole plugin, LNAV, VNAV, or VNAV smoothing.
   Use **Rescan CDU route** after changing the route while the plugin is already
   running.
4. If the status window shows a different raw mode value on your build, adjust
   `lnav_roll_mode`, `vnav_pitch_mode_a/b/c`, or `ap_cmd_mode` in the config
   file and reload it. The FF757 VNAV pitch modes observed so far are 0, 5, and
   6; the pitch enum only says VNAV is engaged. Mode 5 has been seen with both
   `VNAV SPD` and `VNAV PTH`, so the plugin infers SPD/PTH from `vnav_phase`
   (0–2 climb/SPD, 3+ path) and actual vertical speed.

### Default behavior

- **LNAV** is enabled. It improves course tracking and turn anticipation.
- **VNAV** is enabled. It applies a small vertical-path correction and smooths
  the FD pitch command.
- **VNAV smoothing** is enabled. When no vertical path is available, it only
  filters the FD pitch baseline (no correction).
- The plugin is disabled automatically on the ground, when the AP is off, when
  the FF757 nav/vertical mode is not detected, or when the route/CDI data is
  invalid.

### Safety limits (defaults)

| Limit | Default |
| --- | --- |
| LNAV intercept angle | 25° |
| LNAV course error | 60° |
| LNAV bank used for turn anticipation | 25° |
| VNAV pitch correction | ±2.5° |
| VNAV pitch rate | 3°/s |
| VNAV required VS | ±3000 fpm |
| VNAV smoothing time constant | 0.12 s |

All of these are configurable. See [`docs/CONFIG.md`](docs/CONFIG.md).

---

## Menu, commands and datarefs

Menu items:

- Enable plugin
- LNAV enhancement
- VNAV enhancement
- VNAV smoothing
- LNAV FD roll (experimental)
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
| `route_valid` | int R | FF757 route model validated |
| `route_source` | int R | 0 none, 1 private `rteData`, 2 CDU LEGS page |
| `route_legs`, `route_wpts` | int R | Route size |
| `active_wpt`, `active_leg`, `next_leg` | int R | Active indices |
| `ap_mode`, `roll_mode`, `pitch_mode`, `vnav_phase` | int R | Raw FF757 values |
| `ap_engaged`, `lnav_override` | int R | Detected state |
| `xtk_nm`, `distance_nm` | float R | Cross-track / distance |
| `base_course_deg`, `desired_course_deg` | float R | Course target |
| `lnav_correction_deg` | float R | LNAV correction |
| `target_alt_ft`, `required_vs_fpm`, `vs_error_fpm` | float R | VNAV path |
| `vnav_correction_deg` | float R | VNAV correction |
| `cdi_xtk_nm` | float R | Raw CDI cross-track |

These are useful for DataRefTool, hardware bindings, or a custom UI.

---

## How it works

```
FF757 avionics (read only)                 Standard X-Plane (write only)
┌────────────────────────────────┐         ┌─────────────────────────────────┐
│ route arrays (rteData/...)     │         │ nav_steer_deg_mag               │
│ or CDU LEGS page (fallback)    │  ────►  │ flight_director_pitch (legacy)  │
│ AirnavDtk / AirnavDis / ...     │         │ override_flightdir_ptch         │
│ 757Avionics/ap/* modes         │         │                                 │
│ FD pitch (cockpit2)            │         │                                 │
└────────────────────────────────┘         └─────────────────────────────────┘
                 ▲                                        │
                 │                                        ▼
          ┌──────┴──────────────────────────────────────────────────┐
          │  757 LNAV/VNAV Plus (this plugin, separate XPLM DLL)   │
          │  - route parser + validation against GPS CDI            │
          │  - turn anticipation / cross-track / course clamp       │
          │  - VNAV path VS error + acceleration damping             │
          │  - rate limiting, mode gating and safety limits          │
          └─────────────────────────────────────────────────────────┘
```

### LNAV

In the observed FF757 build, LNAV is `roll_mode=2` and the aircraft does **not**
set `override_nav_heading`. The plugin therefore supports two output paths:

- if `override_nav_heading=1`, write the improved
  `sim/cockpit/autopilot/nav_steer_deg_mag` target;
- **optionally** (`enable_lnav_fd_roll=1`) add a small, bounded bank correction
  to the FD roll that the FF757 already produces
  (`sim/cockpit/autopilot/flight_director_roll` with
  `override_flightdir_roll`, plus cockpit2 FD roll if the probe says it is
  writable). This is off by default because some states report FD roll 0; the
  plugin only blends into a bank the FF757 is already commanding (|FD roll| ≥
  2°).

The plugin computes a better target:

- current leg course and cross-track error from the FF757 route (private
  `rteData`, CDU LEGS page, or GPS CDI fallback);
- when a CDU rescan starts at the active waypoint, the inbound fix from the
  previous scan is kept; if none is available, the inbound course is
  reconstructed from the FF757 desired track through the active waypoint and
  the cross-track is computed from the waypoint geometry (not from CDI dots);
- next-leg course and a fly-by turn lead distance based on ground speed and
  the configured bank limit;
- a smoothstep turn blend, then a bounded intercept correction;
- a final course-error clamp so the command never becomes extreme.

It does **not** replace the FF757 roll law, bank limiter, or AP mode logic.

### VNAV

The FF757 publishes its FD pitch in
`sim/cockpit2/autopilot/flight_director_pitch_deg` and its vertical path /
altitude constraints in its route data. The plugin:

- reads the **avionics-owned** cockpit2 FD pitch as a baseline (never its own
  written value, to avoid feedback);
- computes a required vertical speed from the next altitude constraint or the
  FF757 `vtk` value;
- adds a bounded, rate-limited pitch correction with vertical-acceleration
  damping;
- low-pass filters the baseline and writes the legacy FD pitch with
  `override_flightdir_ptch`;
- probes whether the FF757 build also accepts writes to the cockpit2 FD pitch
  dataref (the FF757 itself writes it even though DataRefs.txt marks it
  read-only). If the probe succeeds, the plugin writes both FD paths; if not,
  it reports `cockpit2 FD read-only; legacy FD path` and continues with the
  legacy path.

If the FF757 build does not use either FD/servo path, the VNAV layer will
simply have no authority; LNAV remains unaffected. This is why VNAV is a
separate, optional layer and why the status window reports whether the FD pitch
was actually writable.

---

## Build from source

Requirements: CMake 3.20+, a C++17 compiler, and the
[X-Plane SDK](https://github.com/X-Plane/xpsdk) (CMake fetches it automatically
if `XPLANE_SDK_DIR` is not set).

```bash
cmake -S . -B build -DXPLANE_SDK_DIR=/path/to/xpsdk -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

The binary is written to `build/dist/64/`:

- `win.xpl` on Windows
- `mac.xpl` on macOS
- `lin.xpl` on Linux

Copy it into `plugins/757LVNAVPlus/64/` in the aircraft folder.

### Windows build without Visual Studio

The repository was built locally with Zig 0.13 as a portable C++ compiler:

```powershell
zig c++ -std=c++17 -O2 -DIBM=1 -DXPLM200=1 -DXPLM210=1 -DXPLM300=1 `
  -DXPLM301=1 -DXPLM303=1 -I xpsdk/CHeaders/XPLM -I xpsdk/CHeaders/Widgets `
  -c src/plugin.cpp -o plugin.obj
# ... repeat for config.cpp, route_reader.cpp, guidance.cpp, then:
zig c++ -shared -O2 -o win.xpl plugin.obj config.obj route_reader.obj `
  guidance.obj XPLM_64.lib
```

The local `64/win.xpl` (if present) is a convenience build and is ignored by
git; GitHub releases are built by the CI workflow.

---

## Why no SASL changes

SASL is the encrypted scripting environment used by the aircraft. Modifying it
would be fragile, license-sensitive, and would break on every FF757 update.
This plugin is deliberately **out-of-process with respect to the aircraft**:

- it is a normal XPLM plugin in `plugins/757LVNAVPlus/64/`;
- it never loads, decrypts, replaces or patches `.secured` files;
- it uses only public X-Plane datarefs and the FF757 datarefs that the avionics
  plugin already publishes;
- it can be removed by deleting one folder.

That also means the plugin cannot add new FMS pages or change the FF757 CDU.
It improves the guidance that the existing FF757 autopilot already consumes.

---

## Limitations and disclaimer

- **Experimental.** It has been compiled and unit-tested, but it has not been
  flight-tested in every FF757 variant. Always fly with a safety pilot or in a
  disposable session first.
- X-Plane runs plugin flight loops in load order. The folder name
  `757LVNAVPlus` sorts after `757767Avionics`, which is normally enough for the
  LNAV target write to land after the FF757's own write. If LNAV shows
  `written=1` but has no effect, rename the plugin folder to
  `zzz_757LVNAVPlus` and restart so its callback runs last.
- The FF757 route datarefs are undocumented; validation against the GPS CDI is
  the guardrail. If the private route tree is missing, the CDU LEGS page is
  read instead (which briefly pages the CDU and restores it). If validation
  fails, the plugin falls back to CDI-only LNAV and does not use route turn
  anticipation.
- The CDU scan needs a LEGS page visible on one of the two CDUs. It does not
  navigate from an arbitrary CDU page to LEGS, so after loading a route leave
  the right CDU on LEGS (or use the menu **Rescan CDU route**).
- If the private route altitude/vtk data is unavailable, VNAV remains active
  as an FD-pitch smoothing layer (`src=FD SMOOTH`, correction 0) instead of
  reporting idle. This is intentional: the mode is engaged and the FD pitch is
  still stabilized, but there is no path-deviation correction without a target.
- VNAV effectiveness depends on the FF757 build using the legacy X-Plane FD
  pitch path. If the status window shows `written=0`, VNAV is not reaching the
  servo layer on your build; use LNAV-only or report the dataref values.
- The plugin is not affiliated with FlightFactor, StepToSky, Laminar Research,
  or Boeing. Boeing 757/767 names are trademarks of their respective owners.
- Use at your own risk. This is a flight-simulation enhancement, not a
  certified navigation or autopilot system.

---

## License

MIT, copyright **JACKGONG**. See [`LICENSE`](LICENSE).

This plugin may be redistributed freely — original or modified, including
commercial redistribution — but the author name **JACKGONG** must stay
credited in the documentation, about box, or credits. The attribution may not
be removed or replaced. The startup line in `Output/757LVNAVPlus.log` records
this notice on every run.

The full bilingual statement is in [`NOTICE.md`](NOTICE.md)
(中文 / English).

Third-party build dependencies and notices are listed in
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).

---

## Credits

- **JACKGONG** — author and maintainer of 757 LNAV/VNAV Plus.
- FlightFactor / StepToSky for the 757/767 Professional series.
- Laminar Research for the X-Plane SDK.
- The FF757 community for dataref observations and flight-testing feedback.

If you test this plugin, please include the status-window contents and the
`[757LVNAV+]` lines from `Log.txt` in your report. That is the fastest way to
make the mode detection and gain defaults better for everyone.
