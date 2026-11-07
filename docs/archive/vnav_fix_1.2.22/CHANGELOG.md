# Changelog

## 1.2.22 — 2026-11-07

Fixes VNAV descent fighting the FMC profile.

- Altitude constraints are now read with their CDU qualifier.  `FL157A` means
  "at or above" and `6890B` means "at or below"; previously both were treated
  as exact crossing restrictions.  A live route (`AA146 FL157A` 43 NM ahead at
  FL340) was therefore demanding ~4000 fpm - a descent the aircraft can
  neither fly nor needs to fly - and the plugin pulled the aircraft off the
  FMC's idle path.  A satisfied "at or above" floor is now skipped and the
  next exact/at-or-below constraint becomes the path target, giving a
  flyable gradient.
- The automatic 250 KT / 10000 ft limit is armed only once the aircraft is
  within its braking lead of passing 10000 ft.  It used to point at the first
  below-10000 ft waypoint regardless of distance, so the status showed
  "250 kt @ 106 NM" during cruise.
- The qualifier is parsed from both route sources: the CDU LEGS fallback and
  the private `rteData` altitude strings.
- No change to LNAV or RF guidance.

## 1.2.21 — 2026-11-07

Fixes a load failure on some machines and a CDU-scan crash.

- The CDU reader constructor is a global/static initializer, so it ran inside
  DllMain during `LoadLibrary`.  It called `XPLMFindDataRef` and
  `XPLMGetSystemPath`, which is undefined from DllMain; X-Plane reported it as
  `Error Code = 1114 : DLL initialization routine failed` on a user's machine.
  The constructor no longer touches XPLM - refs bind lazily on the first flight-loop
  tick.
- The CDU screen buffer is always allocated for a full 32x64 screen instead of
  the currently reported line/width size.  A custom CDU display accessor that
  writes past `inMaxLength` right after a page change can no longer corrupt the
  heap; the local crash resolved to a string helper during the route rebuild.
- No behaviour change in guidance.

## 1.2.20 — 2026-11-07

Softer L1 cross-track term.

- 1.2.19's `lnav_l1_factor=0.8` put L1 near 4 NM at cruise, so a 0.15 NM
  offset commanded about 1.7°.  The default factor is now **1.6**, which
  reaches the 8 NM L1 cap above ~400 KT: the same offset commands about 1°
  (0.3° base homing + ~0.7° L1 term), which still closes it in well under a
  minute without a visible swing.
- `lnav_l1_factor` remains configurable; lower it (e.g. 0.8) for a stronger
  correction, or set 0 to disable.

## 1.2.19 — 2026-11-07

Closes the small steady cross-track offset on long straight legs.

- With heading-direct guidance a 0.15 NM offset on a 28 NM leg commanded only
  `atan(0.15 / 28.6) ≈ 0.3°`, so closing it took minutes and any residual wind
  error left it standing.
- Added an L1 look-ahead term on straight legs (787 reference): the correction
  uses `L1 = factor × turn radius` (0.5–8 NM) and adds the difference to the
  base homing angle, clamped to ±4°. RF arcs are excluded (they already use the
  radial correction).
- New `lnav_l1_factor` (default 0.8; 0 disables). Large offsets still use the
  existing 30° / 2.5 NM Boeing capture.

## 1.2.18 — 2026-11-07

Heading-direct guidance now covers every LNAV leg, not just RF arcs.

- On straight legs the plugin previously only wrote `nav_steer_deg_mag` while
  `override_nav_heading` stayed 0, so X-Plane's nav controller ignored the
  target and the aircraft flew the FF757's own LNAV (a small wind-induced
  track offset was visible: TRK 327 vs a 331.8 target with the CDI at 0.14 dot).
- New `lnav_heading_direct` (default **1**): on every LNAV leg the plugin holds
  `override_nav_heading=1`, writes `nav_steer_deg_mag` as a **heading**
  (desired track + wind-correction angle + track damping), and skips FD roll so
  only one lateral controller commands roll. Released when LNAV drops out, the
  AP disengages, the sim pauses or the plugin is disabled.
- `lnav_heading_direct=0` restores the previous behaviour (straight legs pass
  through to the FF757; the override remains RF-only via
  `lnav_rf_nav_override`).
- Status note on straight legs reads `heading direct (override=1)`; the
  status window's `navOverride` reflects the plugin-held flag.

## 1.2.17 — 2026-11-07

Fixed straight legs being mistaken for RF arcs (ZULS "越飞越偏 / 误识别为RF").

- The exit-only RF fallback (added for snapshots that start at the active fix)
  matched any arc in the airport's procedure database whose exit fix happened
  to equal the leg's end fix.  On ZULS, LS428 and LS819 are RF exits in **other**
  SIDs, so straight legs such as LS516→LS428 and LS612→LS819 were annotated as
  arcs.  The aircraft then flew another procedure's circle and drifted away
  (`src=RF ARC` on a leg that the CDU shows as a plain TF leg).
- The fallback now runs **only when the snapshot has no usable entry fix**.
  If a valid entry fix exists but the pair is not an RF leg, the leg is left as
  a normal straight leg.

## 1.2.15 — 2026-11-07

RF arcs now use heading-direct guidance modelled on the 787 reference script.

- On RF arcs the plugin holds `sim/operation/override/override_nav_heading=1`
  every frame (the model clearing it each frame is normal; it is re-asserted)
  and writes `nav_steer_deg_mag` with a **heading**, not a track:
  `desired track + damp + WCA`, where `WCA = hdg - trk` and
  `damp = -0.35 x (trk - desired track)` clamped to ±10°.
- FD roll is not written while this mode is live.  1.2.12 wrote the override
  **and** FD roll (`override_flightdir_roll`) at the same time, replacing the
  nav controller's own roll loop with a ±4° nudge; that broken loop, plus the
  track-as-heading error, produced the abnormal circling.
- New `lnav_track_damp_gain` (0.35) and `lnav_track_damp_max_deg` (10) tune the
  damping.  `lnav_rf_nav_override=0` falls back to the ±4° FD roll path.
- Status note reads `RF arc: heading direct (override=1)`; the status window's
  `navOverride` now also reflects the plugin-held flag.

## 1.2.14 — 2026-11-07

Stopped the RF nav-override tug-of-war that produced abnormal circling.

- Log evidence from the reported flight: `RF arc: override_nav_heading=1`
  appeared **once per frame** while the status window still read
  `navOverride=0`, meaning the FF757 clears the flag every frame.  The plugin
  re-asserted it at 20 Hz, so X-Plane's nav controller and the model's own nav
  logic alternated - the aircraft rolled into a continuous turn.
- `lnav_rf_nav_override` now defaults to **0**.  Forcing the override is opt-in
  only; the ±4° RF FD-roll path (1.2.11 behaviour) is the default.
- When it is enabled, the plugin now watches whether its write survives.  If
  the model resets it three frames in a row, the plugin stops forcing it for
  that arc, restores the flag to 0 and logs "FD roll only" instead of fighting.
- While the override is actually live, the FD roll nudge is skipped so only one
  lateral controller is in command.

## 1.2.13 — 2026-11-07

Active-waypoint recognition follows CDU LEGS page 1 exactly.

- On the FF757 the **first line of LEGS page 1 is the next waypoint** (the one
  being flown to).  The reader now treats that row as authoritative in both
  directions, so an over-sequenced index is pulled back immediately instead of
  sticking until the FMS sequences again.  MOD/reference pages are ignored.
- The geometric sequencing fallback was over-advancing on short, tightly
  turning departures: "bearing to the fix opposes the leg course" flagged
  several nearby fixes as behind during the initial climb, which pushed the
  active index past LS606 (it reported LS612 while the CDU still showed
  LS606/2 NM).  It now only advances when the aircraft is genuinely **past the
  fix along the outbound leg**, measured as an along-track projection, and it
  still requires > 0.3 NM from the fix.

## 1.2.12 — 2026-11-07

RF arcs now take over the nav controller instead of only nudging roll.

- While an RF arc is tracked the plugin forces
  `sim/operation/override/override_nav_heading=1`.  Per X-Plane's own
  description ("override raw heading flown by nav (for GPS that fly arcs)"),
  the nav controller then flies the plugin's `nav_steer_deg_mag` target rather
  than the model's course, so the arc no longer depends on the ±4° roll nudge
  alone.  The flag is only taken when `nav_steer` was actually written, so the
  controller can never chase a stale target.
- The override is released the moment the arc ends or LNAV drops out, and is
  restored to 0 only if the plugin was the one that set it.  Pause, disable,
  stop, exception handling and menu toggles all release it too.
- New `lnav_rf_nav_override` (default 1) controls it.  Set 0 to leave the nav
  controller entirely to the model.
- Status note reads `RF arc: nav override=1` (or `RF arc: nav override + FD
  roll (max 4 deg)` when both are active).

## 1.2.11 — 2026-11-07

RF arcs get a small FD roll correction of their own.

- The FF757 leaves `override_nav_heading` at 0, so the nav target alone cannot
  hold a published RF radius and the model flies its polygonal version.  While
  the plugin is tracking an RF arc it now adds a roll correction to the FD roll
  baseline it already reads, limited to **±4°** and applied only on that leg.
- New `lnav_rf_fd_roll` (default 1) controls it; set 0 to leave roll entirely to
  the FF757.  The global `lnav_fd_roll` stays off by default, so no other leg is
  affected.
- Status note reads `RF arc FD roll (max 4 deg)` when the nudge is active.

## 1.2.10 — 2026-11-07

Fixes RF legs never activating because the CDU snapshot stayed on waypoint 0.

- The snapshot's active index now has a **geometric sequencing fallback**: while
  the active fix is clearly behind the aircraft (bearing to it opposes the leg
  course, and it is more than 0.3 NM away), the index advances.  Previously
  sequencing relied on Airnav distance/bearing or the visible LEGS page, so a
  CDU left on another page could leave the route pointing at the departure
  runway while the FMS had already reached LS608 — and an RF leg could never be
  reached.
- Applies only to CDU-derived routes and advances monotonically, so it cannot
  jump backwards or skip an active fix.

## 1.2.9 — 2026-11-07

RF geometry now comes from the FF757's own procedure database first.

- `Custom Data/STSFF/navdata/Proc/<ICAO>.txt` is parsed for RF rows:
  `RF,<fix>,<lat>,<lon>,<dir>,<centre>,<course>,<radius>`.  The direction field
  is the value behind the CDU's ARC LEFT (1) / ARC RIGHT (2), so the traced
  circle now matches the magenta path the FMS draws, including direction.
- FF757 data is preferred; X-Plane's CIFP remains the fallback for airports or
  cycles where the model has no procedure file.  Verified at ZULS: LS608 →
  R = 2.70 NM, right arc, 130.1°, centre LS016 from both sources.
- Parser unit tests cover the FF row format, block resets and the
  1 = left / 2 = right mapping.

## 1.2.8 — 2026-11-07

Stops LNAV from dropping to GPS CDI steering over transient events.

- The CDI cross-check now needs a **sustained** disagreement (~2 s / 40
  frames) before the route is rejected.  A single frame during a CDU rescan,
  a sequencing step, or a lagging FMS course no longer takes the route away.
  The note reports `route course differs from CDI; holding route`.
- FMS course anchoring only applies to differences of 8° or less.  A larger
  difference means the FMS is on another leg or procedure, and its course no
  longer overrides our own geometry.
- New `ROUTE HOLD`: when the route snapshot is momentarily unusable, the last
  route-based command (course, cross-track, distance) is held for ~1 s before
  the CDI fallback is allowed, so the aircraft keeps flying our path instead
  of switching sources several times a minute.

## 1.2.7 — 2026-11-07

Fixes RF arcs that were missed when their entry fix was absent from the
route snapshot (ZULS DEP28R, LS606 → LS608).

- The CDU LEGS cache starts at the active waypoint, so an RF leg can be
  active while the fix before it is missing.  `AnnotateRfArcs` now falls back
  to an **exit-only CIFP match** and recovers the entry coordinates and the
  centre from the CIFP row.  Verified offline with the real ZULS CIFP:
  LS608 active → R = 2.70 NM, right turn, 130.1°, centre LS016.
- `RouteLeg` carries the entry fix coordinates (`rfEntryId`, `rfEntryLat/Lon`,
  `rfEntryValid`), so guidance no longer needs the entry waypoint in the
  snapshot.
- The RF lead-in now targets the CIFP entry point and builds its inbound line
  from the arc's own entry tangent when no start fix exists, instead of
  falling back to a synthetic FMS course.  This removes the `ROUTE/SYNTH`
  path that produced the 4.25 NM phantom cross-track and the false
  `NAV UNABLE RNP`.

## 1.2.6 — 2026-11-07

Corrects the RF capture gate from 1.2.5: the FF757 cannot fly RF legs (its
LNAV chains straight segments), so 1.2.5's "hand the leg back until capture"
was wrong - it would just fly the arc as a polygon.

- The plugin now flies the **lead-in** itself: the inbound course to the arc
  entry fix, with no fly-by anticipation (the arc is tangent to the inbound
  course). The status note reads `RF arc lead-in to <fix>`.
- **The RF segment no longer references the FF757 FMS route at all.** On RNP
  legs the model's own route computation is unreliable, so neither the course
  anchoring nor the CDI course/distance validation runs on the lead-in, and
  the arc is captured from CIFP geometry alone (on the circle, at or past the
  entry radial). ANP(trk) therefore reflects our own path, not the model's.
- The arc is picked up at the entry radial when the aircraft is on the circle
  (1.5 NM band, 3 NM once tracking).
- Hand-back only remains for the genuine cases: RF disabled, or an arc whose
  geometry cannot be captured (off the circle past the entry).

## 1.2.5 — 2026-11-07

Fixes RF arcs being hijacked far before their entry, reported at ZULS
(EO28R RNAV departure).

- An RF leg becomes active many miles before its arc entry.  The plugin used
  to start tracking the tangent immediately, commanding a course 50-60 deg off
  the inbound leg (observed: 82 deg vs the FMS course of 33 deg), which looked
  like a turn deviation and inflated the cross-track to ~7 NM.
- RF tracking now only engages when the aircraft is actually on the circle
  (within 1.5 NM, 3 NM once tracking), at or past the entry radial.
- The arc's entry tangent must agree with the FMS/CDI course within 30 deg,
  so a CIFP row matched to the wrong procedure can no longer take over.
- The false `NAV UNABLE RNP` at that moment came from the bogus 6.92 NM
  cross-track feeding the ANP estimate; with the gate it is no longer sampled.

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
