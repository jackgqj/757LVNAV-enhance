# Changelog

## 1.1.1 — 2026-11-05

LNAV turn-geometry fixes after flight testing (overshoot and corner cutting).

- Turn anticipation now starts at the exact tangent point by default
  (`lnav_turn_lead_factor=1.0`, was 1.35). The old default rolled in 35 %
  early, cut the corner and forced a longer bank on roll-out, which showed up
  as an overshoot. Existing configs holding the legacy 1.35 value are migrated
  once to 1.0.
- The cross-track intercept is now computed per leg: the next leg's cross-track
  is divided by the distance to *its* fix (new `lnav_intercept_distance_nm`,
  default 1.5 NM), not by the distance to the current fix. That removed the
  25°-clamped last-second turn at the corner.
- Intercept angles near a fix are floored by `lnav_intercept_distance_nm`, so
  the `atan2` term can no longer explode in the last mile.
- LNAV now flies the fly-by turn as a real tangent arc, the way a 757/767 FMC
  does: radius `R = V²/(g·tan φ)` (400 kt at 25° ≈ 5 NM), turn starts
  `R·tan(Δ/2)` before the fix, and the arc centre sits `R` inside the inbound
  course measured from that tangent point. The previous ramp used the straight
  distance to the fix, which shrinks and then grows again inside a fly-by turn:
  that is what un-turned the target on roll-out and overshot.
- Corner cutting fix: the arc centre was placed `R` inside the *fix*, which
  makes the circle tangent to the inbound leg at the fix. The aircraft then
  reached "the end of the arc" while still `R·tan(Δ/2)` miles short, so LNAV
  rolled onto the outbound course at the tangent point and cut the corner.
  The centre is now built from the tangent point, and the arc is measured by
  the bearing seen from that centre instead of a chord, so a lateral offset no
  longer reads as a turn already half flown.
- The turn now starts on the remaining distance *along the leg*, not on the
  straight distance to the fix, so a cross-track offset cannot trigger it early.
  The arc stays tangent-course only, without stacking a cross-track intercept
  on top of the turn command.
- Passing a fix without a sequence no longer un-turns the target (a bearing test
  detects that the fix is behind the aircraft, also for synthetic inbound legs).
- The status window now names the detected VNAV submode (`SPD` / `PTH`, matching
  the PFD FMA) and labels hands-off states `STBY` instead of `PASS`, so a VNAV
  PTH hold at the MCP altitude no longer looks like a detection failure.

## 1.1.0 — 2026-11-01

Route-scanning and VNAV correctness fixes found in live flight testing.

- Fixed the CDU LEGS-page altitude parser. The FF757 prints course/distance on
  the line above a waypoint and the speed/altitude estimate on the waypoint
  line itself (`EKIMU  .806/FL315`). The old parser read the course as an
  altitude (`336` → 33600 ft, `010` → 1000 ft) and swapped speed/altitude in
  constraints. Route altitudes now match the CDU (FL315, FL329, FL197, FL341).
- VNAV SPD/climb now passes the FF757 pitch command through unchanged by
  default (`vnav_augment_speed_mode=0`); injecting a vertical-path correction
  into a mode that holds speed with pitch broke speed hold.
- The MCP selected altitude is read from `757Avionics/ap/alt_act` (falling
  back to `sim/cockpit/autopilot/altitude`) and respected as a climb ceiling /
  descent floor. A route constraint below the selected altitude no longer
  commands a descent during a level-off (`MCP HOLD`).
- VNAV never initiates a vertical path while the aircraft is level, and never
  fights an existing vertical direction; it only assists a path already being
  flown (`FF757 OWNS VS`). Level-off, MCP hold, VNAV SPD and no-path cases all
  pass the FF757 pitch command through instead of writing a smoothed value.
- Added a 150 fpm vertical-speed-error deadband: small corrections no longer
  write FD pitch, removing a feedback path that produced a visible pitch/AoA
  porpoising oscillation and let speed run away.
- Any established climb (VS > +300 fpm) is treated as VNAV SPD speed mode even
  if the FF757 phase label lags, so no path correction is injected into the
  speed loop.
- CDU scans now run periodically in the air as well as on the ground
  (`cdu_rescan_interval_s=45` default). The visible LEGS page is compared with
  the cache every 0.5 s, so an altitude/leg edit triggers an early rescan.
- Fixed a scanner deadlock: after the 20 s timeout in `Restore`, the state
  machine could stay in `Restore` forever and the CDU cache never refreshed.
  There is now a 25 s overall limit plus an 8 s restore limit, page stalls are
  detected after eight key presses, and a partial scan never overwrites a
  complete cache.
- Active waypoint is now synchronised from the FF757's own `AirnavDis` +
  `AirnavBrg` every 0.5 s. Matching distance and bearing against the cached
  route can jump several legs and works while the CDU is left on a page other
  than LEGS page 1. This fixes LNAV falling back to GPS-CDI steering after
  passing a fix (for example after PIKAS).
- Added `LNAV authority` / `VNAV authority` log lines that fire only when the
  guidance source changes, so a CDI fallback or a pitch pass-through is visible
  without dumping every frame.
- Diagnostics now report the CDU scan state, cached leg/page counts, VNAV
  speed mode and the selected MCP altitude.

## 1.0.0 — 2026-11-01

Initial experimental release.

- Separate XPLM plugin; no SASL or aircraft-file modifications.
- FF757 route reader with best-effort undocumented dataref binding.
- CDU LEGS-page route fallback for builds that do not expose the private
  `rteData` tree: the right CDU is preferred, pages are read through the
  `1-sim/cduL|R/display/*` character grids, waypoint coordinates are resolved
  from the FF757 navdata, and the CDU is returned to its original page.
- Published `757lvnv/route_source` (0 none, 1 `rteData`, 2 CDU LEGS).
- LNAV geometry is now computed from the scanned route when a CDU rescan
  starts at the active waypoint: the previous scan's inbound fix is preserved,
  and otherwise a synthetic inbound course is derived from the FF757 desired
  track so cross-track/turn anticipation stay route-based.
- CDU scanning is driven by **visible page-content changes** (default) with no
  periodic timer (`cdu_periodic_rescan=0`, `cdu_rescan_interval_s=0`). The same
  page content is never rescanned twice; edits on a MOD page are scanned once
  but the previous ACT cache is kept until EXEC makes the route active.
- Optional `cdu_scan_on_exec_only=1` keeps the MOD → ACT (EXEC) gate for users
  who do not want MOD-page content scans. The FF757's CDU EXEC input dataref is
  not updated by physical CDU clicks, so the page title is the usable signal.
- `Dump diagnostics` no longer starts a route scan; `Rescan CDU route` remains
  the explicit manual trigger.
- VNAV SPD/PTH detection no longer treats pitch mode 5 as unconditionally
  `VNAV SPD`. Mode 5 is observed with both `VNAV SPD` and `VNAV PTH`; SPD is now
  inferred from `vnav_phase` 0–2 and actual climbing vertical speed. This fixes
  the status staying at `VNAV SPD PASS` after the FMA changed to `VNAV PTH`.
- Route snapshots are re-written to the log whenever a rescan changes the
  route shape.
- Fixed the legacy config migration, which disabled the observed VNAV pitch
  mode 6 (`vnav_pitch_mode_b=-1`); it now migrates to `6`.
- Added a third VNAV pitch-mode slot (`vnav_pitch_mode_c=5`) for the observed
  `VNAV SPD` mode, alongside `0` (early climb) and `6` (`VNAV PTH`).
- Route validation against the FF757 GPS CDI.
- LNAV course tracking, cross-track interception, and fly-by turn anticipation
  written through `sim/cockpit/autopilot/nav_steer_deg_mag`.
- VNAV vertical-path correction from route altitude constraints / `vtk`,
  blended into the legacy FD pitch with `override_flightdir_ptch`.
- FD pitch low-pass smoothing and rate limiting.
- Mode detection and safety gating for AP, LNAV, VNAV, ground, pause, and
  invalid data.
- Status window, menu, commands, published datarefs, persistent config.
- Windows/macOS/Linux CMake build and GitHub Actions workflow.
- Unit tests for geodesy, LNAV, turn anticipation, CDI fallback, and VNAV.
