# Configuration

The config file is created at:

```
X-Plane 12/Output/preferences/757LVNAVPlus.cfg
```

Delete it to restore defaults. After editing, use **Plugins → 757 LNAV/VNAV
Plus → Reload config** (or restart X-Plane).

## Full default file

```ini
# 757 LNAV/VNAV Plus configuration
# Delete this file to restore defaults.

enabled=1
enable_lnav=1
enable_vnav=1
enable_vnav_smoothing=1
enable_lnav_fd_roll=0

lnav_gain=0.65
lnav_max_intercept_deg=25
lnav_turn_lead_factor=1.0
lnav_max_bank_deg=25
lnav_max_course_error_deg=60
lnav_intercept_distance_nm=1.5
lnav_min_speed_kts=60
lnav_min_distance_nm=0.05
lnav_roll_gain=0.25
lnav_max_roll_correction_deg=6
lnav_roll_rate_limit_deg_s=5
lnav_roll_smoothing_tau_s=0.1

vnav_vs_gain=0.8
vnav_accel_gain=0.0015
vnav_max_correction_deg=2.5
vnav_rate_limit_deg_s=3
vnav_smoothing_tau_s=0.12
vnav_max_vs_fpm=3000
vnav_min_distance_nm=0.3
vnav_max_alt_delta_ft=50000
vnav_augment_speed_mode=0
vnav_respect_selected_alt=1

cdu_rescan_interval_s=0
cdu_periodic_rescan=0
cdu_scan_on_exec_only=0

lnav_roll_mode=2
vnav_pitch_mode_a=0
vnav_pitch_mode_b=6
vnav_pitch_mode_c=5
ap_cmd_mode=1
lnav_wide_mode=0
force_lnav=0
force_vnav=0
debug=0
```

## Tuning guide

### LNAV

| Key | Meaning | Tuning |
| --- | --- | --- |
| `lnav_gain` | Cross-track intercept gain | Lower (0.4–0.6) for smoother, slower capture; higher (0.8–1.0) for tighter tracking |
| `lnav_max_intercept_deg` | Maximum intercept angle | 25° is a good airliner default; reduce to 15° if your sim setup is twitchy |
| `lnav_turn_lead_factor` | How early the turn starts relative to the tangent point | 1.0 = tangent (no corner cut, no rollout overshoot). Increase only if the aircraft still overshoots fixes; decrease below 1.0 makes it turn earlier |
| `lnav_max_bank_deg` | Bank used to compute turn radius | Match your normal LNAV bank limit (25° default) |
| `lnav_max_course_error_deg` | Hard course-error clamp | 60° default; prevents extreme commands |
| `lnav_intercept_distance_nm` | Minimum distance used for the intercept denominator | 1.5 nm default. Raise it to soften last-second turns near a fix; lower it only if capture is too sluggish |
| `lnav_min_speed_kts` | Below this, LNAV is idle | Avoid taxi/ground guidance |
| `lnav_min_distance_nm` | CDI-only minimum distance | Prevents chasing a sequenced fix |
| `enable_lnav_fd_roll` | Enable the experimental FD-roll bridge | 0 default; can also be toggled from the menu `LNAV FD roll (experimental)`. Turn on only after a controlled flight test |
| `lnav_roll_gain` | Bank correction per degree of improved-course error | 0.25 default; lower for gentler FD roll blending |
| `lnav_max_roll_correction_deg` | Hard bank-correction limit | 4° default; do not exceed 6° without flight testing |
| `lnav_roll_rate_limit_deg_s` | FD roll correction slew rate | 3°/s default |
| `lnav_roll_smoothing_tau_s` | FD roll baseline low-pass | 0.10 s default |

### VNAV

| Key | Meaning | Tuning |
| --- | --- | --- |
| `vnav_vs_gain` | Pitch correction per 1000 fpm of VS error | 0.8 default; lower to 0.5 for gentler path capture |
| `vnav_accel_gain` | Vertical-acceleration damping | 0.0015 default; increase to 0.003 if VNAV porpoises |
| `vnav_max_correction_deg` | Hard pitch correction limit | 2.5° default; do not exceed 4–5° without flight testing |
| `vnav_rate_limit_deg_s` | Max correction slew rate | 3°/s default |
| `vnav_smoothing_tau_s` | Baseline FD pitch low-pass time constant | 0.12 s; increase for smoother but laggier response |
| `vnav_max_vs_fpm` | Required-VS clamp | 3000 fpm default |
| `vnav_min_distance_nm` | Do not use path data inside this distance | 0.3 nm default |
| `vnav_max_alt_delta_ft` | Hard cap for implausible altitude constraints | 50000 ft default; also scales with distance to the fix (8000 ft minimum, 1200 ft/nm) |
| `vnav_augment_speed_mode` | Allow path corrections while the FF757 is in VNAV SPD | **0 default**. VNAV SPD holds speed with pitch; a path correction there fights the speed loop. Set to 1 only for flight testing |
| `vnav_respect_selected_alt` | Treat the MCP altitude as a climb ceiling / descent floor | 1 default. Prevents the plugin from descending past a level-off (for example when 34100 ft is selected) |

### Route cache

| Key | Meaning | Tuning |
| --- | --- | --- |
| `cdu_scan_on_exec_only` | Scan only an executed route | 0 default: scan once whenever the visible LEGS page content changes. Set to 1 to wait for `ACT RTE` (MOD → ACT) and skip MOD-page content changes |
| `cdu_periodic_rescan` | Enable the periodic rescan timer | 0 default. The default trigger is page-content changes only; set to 1 with a non-zero interval to add periodic rescans |
| `cdu_rescan_interval_s` | Periodic rescan interval | Only used when `cdu_periodic_rescan=1`. 0 default; set 5–600 |

The scanner never replaces a complete cached route with a partial scan. Every
scan writes one `CDU scan begin` / `CDU scan done` line to
`Output/757LVNAVPlus.log`, so a stalled or periodic scan is visible in the log.

### FF757 mode codes

The FF757 avionics binary exposes these as enums whose numeric values are not
documented. The defaults below are **observed from a live FF757 v2 build**
with the FMA showing `LNAV`, `VNAV SPD`/`VNAV PTH`, `CMD`:

- AP: `CMD=1` (observed)
- lateral: `LNAV=2` (observed)
- vertical: pitch modes `0`, `5`, and `6` observed. The pitch enum only
  indicates that VNAV is engaged; it does not split SPD from PTH. Mode `5` has
  been observed with an FMA showing both `VNAV SPD` (climb phase) and
  `VNAV PTH` (phase 3, level/descent path). The plugin infers the split from
  `vnav_phase` (0–2 = climb/SPD, 3+ = path) and actual vertical speed. All
  three slots are enabled by default (`vnav_pitch_mode_a=0`, `_b=6`, `_c=5`).
  `HDG`/`ALT` values are not required for normal LNAV/VNAV operation.

If the status window shows different raw values on your build, change:

| Key | Default | Meaning |
| --- | --- | --- |
| `lnav_roll_mode` | 2 | Roll-mode enum for LNAV (observed) |
| `vnav_pitch_mode_a` | 0 | First VNAV engagement slot (observed in the initial climb) |
| `vnav_pitch_mode_b` | 6 | Second VNAV engagement slot |
| `vnav_pitch_mode_c` | 5 | Third VNAV engagement slot (observed in both `VNAV SPD` and `VNAV PTH`) |
| `ap_cmd_mode` | 1 | AP mode enum for CMD (observed) |
| `lnav_wide_mode` | 0 | If 1, every non-zero roll mode is treated as LNAV (debug only) |
| `force_lnav` | 0 | Bypass the LNAV mode gate (debug only; writes may be ignored) |
| `force_vnav` | 0 | Bypass the VNAV mode gate (debug only) |
| `debug` | 0 | Write extra lines to `Output/757LVNAVPlus.log` |

Set a VNAV pitch-mode slot to `-1` to disable it if your build uses a
different value.

**LNAV mode gate.** When the FF757 publishes `roll_mode_engaged`, the plugin
requires it to equal `lnav_roll_mode` (default 2, observed LNAV). This prevents
it from fighting a localizer or rollout mode. If your build uses a different
LNAV enum value, change `lnav_roll_mode`; `lnav_wide_mode=1` treats every
non-zero roll mode as LNAV and is a diagnostic fallback, not the recommended
setting.

The FF757 build observed here does **not** set `override_nav_heading` in
LNAV. The plugin therefore has two LNAV output paths:

1. if `override_nav_heading=1`, write the improved `nav_steer_deg_mag`;
2. **only if `enable_lnav_fd_roll=1`**, add a small bank correction to the FD
   roll (`flight_director_roll` legacy + cockpit2 if writable). The FD-roll
   bridge is off by default because some FF757 states report FD roll 0; writing
   a bank there can start an unwanted turn. When enabled, the plugin only
   blends into a bank the FF757 is already commanding (|FD roll| ≥ 2°).

VNAV uses the pitch-mode enum because there is no equivalent public "override"
gate for vertical guidance.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Status: `FF avionics datarefs: missing` | Is this the FF757? Is `757767Avionics` loaded? Check `Log.txt`. |
| Status: `Route not valid yet` | Load an active FMS route and wait one second. |
| `route datarefs: none` | The private route layout changed; LNAV falls back to GPS CDI. |
| Route never rescans | Check `Output/757LVNAVPlus.log` for `CDU scan begin` / `CDU scan done`. A LEGS page must be visible on one CDU. With `cdu_scan_on_exec_only=1` the scan runs once after EXEC (`MOD RTE` → `ACT RTE`); while the page shows `MOD RTE` it waits. Use the menu `Rescan CDU route` to force one. |
| `Rescan CDU route` does nothing | Look for `CDU scan queued` in the log. If the CDU is not showing a LEGS page, the plugin reports `CDU is not showing a LEGS page`; open LEGS on the right CDU. If the log shows `CDU scan timeout`, the page keys were not accepted; try the menu rescan again or check `757Avionics/CDU2/prev_page`/`next_page`. |
| LNAV falls back to `GPS CDI` after a fix | The active waypoint is resynced from `AirnavDis`/`AirnavBrg` every 0.5 s. Check the `LNAV authority` line; `CDU active sync: old -> new` should appear around sequence. If no sync line appears, `757Avionics/fms/AirnavBrg` may be missing and only the one-step fallback is active. |
| Route cache stays at 24 legs / old altitudes | The log now prints a `FF757 route begin` block whenever the route content changes. If it never updates, check `CDU scan note` in the log. A partial scan is rejected and the complete cache is kept. |
| VNAV SPD does not hold speed | `vnav_augment_speed_mode` must stay `0`; the plugin passes pitch through in VNAV SPD/climb. Check the status note `VNAV SPD holds speed with pitch; path correction suppressed`. |
| Pitch / AoA oscillates after VNAV engages | The status source should be `VNAV SPD PASS`, `MCP HOLD`, `ALT HOLD`, `ROUTE HOLD`, `FF757 OWNS VS`, `FD PASS` or `VNAV PASS`; all of these write **nothing**. Only `ROUTE PROFILE`/`ROUTE/MCP`/`VTK` write FD pitch, and only with a VS error above 150 fpm. If it still oscillates, set `enable_vnav=0` to confirm the FF757 alone is stable. |
| Overspeed in VNAV | Check that the plugin source is not `ROUTE PROFILE` during a level-off. `MCP HOLD`/`ALT HOLD`/`FF757 OWNS VS` mean the plugin is not commanding a descent. If `selAlt=0`, the MCP altitude was not read and the clamp is inactive. |
| Still descending with 34100 ft selected | The MCP clamp should report `MCP HOLD`. Check the VNAV target/source in the status window; `selAlt=0` means the MCP altitude could not be read. |
| LNAV writes but the aircraft does not turn | Check `override_nav_heading` in the status window. If it is 0, the FF757 is not in nav mode. |
| VNAV `written=0` | Check `override_flightdir_ptch` and whether the legacy FD pitch is writable. Some builds may use a different servo path. |
| Status note `cockpit2 FD read-only; legacy FD path` | The probe could not make the cockpit2 FD value stick. Legacy FD pitch is still written; if the aircraft uses only cockpit2, VNAV has no authority on that build. |
| Cuts the turn / rolls in too early | Lower `lnav_turn_lead_factor` (1.0 is the tangent point; 0.9 rolls in slightly later). Check that `lnav_max_bank_deg` matches the aircraft's bank limit. |
| Overshoots the turn or the new course | Raise `lnav_intercept_distance_nm` (softer intercept in the last mile) or raise `lnav_turn_lead_factor` slightly (1.0–1.2). |
| S-turns on a straight leg | Decrease `lnav_gain`. |
| VNAV porpoises | Increase `vnav_accel_gain`, decrease `vnav_vs_gain`, or increase `vnav_smoothing_tau_s`. |
| VNAV never engages | Look at the raw `pitch_mode` in the status window and add it to `vnav_pitch_mode_a/b/c` (observed VNAV values: 0, 5, 6). |

When reporting an issue, include:

1. the full status-window text;
2. the `[757LVNAV+]` lines from `Output/Log.txt`;
3. `Output/757LVNAVPlus.log`;
4. the FF757 variant and X-Plane version.
