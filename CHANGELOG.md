# Changelog

## 1.2.4 — 2026-11-06

Fixes the VNAV pitch tug-of-war during descent.

- The FF757's VNAV and the plugin write the same FD pitch dataref.  Because
  the plugin dropped to pass-through whenever a single frame looked level or
  the FD baseline read near zero, the two controllers alternated authority
  frame by frame (visible pitch oscillation in DES).
- New `vnav_priority` (default 1): the plugin latches pitch authority on the
  first frame with a valid vertical path and holds it until the path has been
  inactive for ~2 s.  The log records `VNAV pitch priority latched/released`,
  and the status notes show `plugin pitch priority`.
- While priority is held the plugin no longer chases the FF757's moving FD
  command: it keeps the reference captured at takeover and only re-centres it
  while its own correction is below 0.5 deg.  The near-zero-baseline
  pass-through guard is skipped in this state, since the zero is the plugin's
  own read-back.
- Set `vnav_priority=0` to restore FF757 priority.

## 1.2.3 — 2026-11-06

Fixes the remaining turn problem: a long leg with a small offset used to
command a large intercept and swing the aircraft.

- The leg course is now anchored on the **FMS/CDI course** when the two agree
  within 15°.  Our course is derived from resolved coordinates and can sit a
  degree off; over a 60 NM leg one degree became a ~1 NM phantom cross-track,
  which turned into a 21° intercept at 400 kt.  Re-anchoring the line through
  the fix removes both the phantom XTK and the false `NAV UNABLE RNP`.
- Direct homing at small offsets now uses the real distance to the fix
  (`atan(xtk / distance)`) instead of a 6 NM-clamped denominator, so the angle
  decays as the offset closes instead of holding ~12° 50 NM out.  The
  constant 30° capture now hands over only beyond `lnav_xtk_blend_nm`
  (blended over ±0.4× that value).
- The unused `lnav_intercept_distance_nm` / `lnav_capture_distance_nm` knobs
  are removed; `lnav_xtk_blend_nm` is now the single tuning point.
- New regression tests replay the reported leg (UU402→BOKIR, 305 KT IAS /
  400 KT GS): correction stays under 5° and the cross-track no longer inflates.

## 1.2.2 — 2026-11-06

Removes the last navdata file scan from the flight loop.

- The FF757 navdata is now read **once** per session into an in-memory index
  (~390,000 fixes, 16-byte records, about 6.3 MB), then every ident lookup is
  a binary search.  Measuring: index build 362 ms once at route load, 8 real
  fixes resolved in 0.001 ms afterwards, 0.000 ms per repeated lookup.
- `earth_fix.dat` stays lazy: it is opened only when a fix genuinely cannot be
  found in the FF757 navdata, and each such ident is searched at most once.
- Index size is hard-capped at 600,000 records (~9.6 MB) so memory stays well
  below the 10 MB budget.
- Status window label changed from `ANP=` to `ANP(trk)=`: the plugin measures
  track-keeping error against the active path, not the real ANP (the FMS's
  95% position-error estimate from GPS/IRS).

## 1.2.1 — 2026-11-06

Performance fix for the frame-rate drop reported with 1.2.0.

- `NavDatabase::resolve` now remembers every ident it has already searched
  (found or not).  Before, one unresolved ident made each route rebuild
  re-read the whole navdata set: Waypoints.txt 9.3 MB, Airports.txt 3.5 MB,
  Navaids.txt 0.8 MB and earth_fix.dat 14 MB.  Measured 340 ms per rebuild
  with the files cold, i.e. a guaranteed frame hitch every second; now
  0.0001 ms per call.
- The navdata scans now stop as soon as every requested ident has a
  candidate, so `earth_fix.dat` is only opened when the FF757 navdata really
  cannot resolve a fix.
- CIFP RF lookups cache "no arc for this fix pair" per session, and the six
  nearest airports are selected in one pass instead of sorting the whole
  airport list on every leg.
- `computeVnav` builds its status note with plain `snprintf`; the previous
  `std::to_string` concatenation allocated on every flight-loop frame.

## 1.2.0 — 2026-11-06

RF (constant-radius-to-fix) leg tracking.  The FF757 flies RF legs as a
polygon chain; the plugin now resolves the real arc from X-Plane's CIFP
procedure database and tracks the circle itself.

- New CIFP parser: RF rows (entry/exit fix, turn direction, radius, centre
  fix) are read from `Resources/default data/CIFP/<ICAO>.dat` (with
  `Custom Data/CIFP` overrides) for the airports nearest each route leg.
- Route legs matching an RF row are annotated with the arc geometry.  The
  centre fix (e.g. CFBRK) is resolved via the nav database, now extended with
  an `earth_fix.dat` fallback for computed procedure fixes; if that fails the
  centre is reconstructed from the entry fix, inbound course and radius.
- Guidance tracks the arc: desired course follows the tangent at the current
  radial plus a bounded radial correction back onto the circle; progress is
  unwrapped so arcs beyond 180 deg cannot wrap; the roll-out lead freezes the
  target on the outbound tangent exactly like fly-by turns.  Works for both
  rteData complex legs and CDU-synthesised straight legs.
- The status window shows RF progress (`RFarc=%`), and the log note reports
  radius, turn direction, degrees remaining and the bank angle the published
  radius needs at the current groundspeed (flagged when it exceeds
  `lnav_max_bank_deg`).
- RNP/ANP monitoring now also runs on RF legs, using the radial error as the
  lateral deviation (approach RNP 0.3 applies on final RF segments).
- New config key `lnav_rf_enabled` (default 1); set 0 to pass RF legs through
  to the FF757 like any other complex leg.
- VNAV speed-limit handling (FAR 250 KT / 10000 ft auto-enforcement +
  CDU-declared DECEL points).  Uses the Boeing 777 "1 NM per 10 KT slowdown"
  deceleration lead: when a speed target coincides with the next altitude
  target the descent VS is softened so the aircraft reaches the fix at the
  right speed instead of having to brake hard.  New config keys:
  `vnav_speed_limits_enabled` (default 1),
  `vnav_speed_decel_factor_nm_per_10kt` (default 1.0),
  `vnav_hard_250_below_ft` (default 10000).  DECEL also disables the 150 fpm
  FD-pass deadband (the softened VS IS the plan, not a residual error).
- Status window now shows the next speed limit, its distance, and the
  deceleration lead.

## 1.1.4 — 2026-11-06

Boeing-style course capture replaces the gain-based intercept, which was the
source of the "wild swing when cutting the course" complaints.

- |XTK| beyond `lnav_xtk_blend_nm` (default 2.5 NM) commands a constant
  intercept back to the course line - `lnav_max_intercept_deg` now defaults
  to 30 deg, the 777 value (config files holding the old 25 are migrated).
- Small offsets home direct to the fix, allowed up to
  `lnav_direct_intercept_deg` (default 45 deg).
- The two modes blend smoothly on |XTK| (smoothstep), so the capture
  transitions to 0 deg at the course without snapping.  The near-fix
  distance floor and the far-field capture-distance cap still apply to the
  homing angle.
- `lnav_gain` now only affects the CDI-only fallback path.

## 1.1.3 — 2026-11-06

LNAV overshoot fixes (observed in flight: the aircraft rolled out ~0.3 NM
right of the new course and the offset never converged).

- Roll-out lead: the target course is frozen on the outbound leg a few
  degrees before the tangent arc ends (default 3 deg,
  `lnav_rollout_lead_deg`).  The autopilot's own capture lag then levels the
  wings on course instead of carrying the turn past it.
- Long-range capture: the intercept denominator is now clamped to
  [interceptDistance, captureDistance] instead of growing with the distance
  to the fix.  A 0.3 NM offset 50 NM out used to command 0.2 deg and never
  closed; with `lnav_capture_distance_nm=6` it commands a real intercept and
  converges within roughly 10 NM.  Near-fix behaviour is unchanged (the 1.5
  NM floor still prevents last-second turns).

## 1.1.2 — 2026-11-06

RNP / ANP monitoring (Boeing 777 FMC logic).

- The plugin now estimates ANP (actual navigation performance) as a 95%
  containment proxy: twice the low-passed mean |cross-track| while route-based
  LNAV is active.
- RNP is selected automatically by flight phase: enroute 2.0 NM, terminal
  1.0 NM within 30 NM of the destination (the AC 91-006 alerting boundary),
  approach 0.3 NM on the final segment. `rnp_nm` overrides it for a specific
  procedure; `rnp_enabled=0` turns the feature off.
- When ANP exceeds RNP for ~2 s, the status window shows `NAV UNABLE RNP` in
  amber/red and the event is written to Log.txt; the alert clears only after
  ANP is back below 80 % of RNP (hysteresis), mirroring the 777 FMC caution.
  The alert is advisory only - steering is never degraded by it.

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
