# Datarefs used by 757 LNAV/VNAV Plus

Everything here is read-only unless explicitly marked **WRITE**. The plugin
does not write any `757Avionics/*` dataref except the CDU page keys used to
scan the LEGS page, and does not touch SASL.

## Standard X-Plane datarefs (read)

| Dataref | Use |
| --- | --- |
| `sim/flightmodel/position/latitude` | Aircraft position |
| `sim/flightmodel/position/longitude` | Aircraft position |
| `sim/flightmodel/position/true_psi` | True heading / magnetic-variation reference |
| `sim/flightmodel/position/mag_psi` | Magnetic heading |
| `sim/flightmodel/position/hpath` | Ground track |
| `sim/flightmodel/position/groundspeed` | Ground speed (m/s) |
| `sim/flightmodel/position/vh_ind_fpm` | Vertical speed |
| `sim/flightmodel/position/elevation` | Fallback altitude (m) |
| `sim/cockpit2/gauges/indicators/altitude_ft_pilot` | Baro altitude (ft) |
| `sim/cockpit2/gauges/indicators/airspeed_kts_pilot` | IAS |
| `sim/flightmodel/failures/onground_any` | Ground gating |
| `sim/time/paused` | Pause gating |
| `sim/cockpit2/autopilot/autopilot_on` | AP engaged fallback |
| `sim/cockpit2/autopilot/servos_on` | Servo fallback |
| `sim/cockpit2/autopilot/heading_mode` | Standard lateral mode fallback |
| `sim/cockpit2/autopilot/altitude_mode` | Standard vertical mode fallback |
| `sim/cockpit2/autopilot/nav_status` | Standard nav status |
| `sim/cockpit2/autopilot/vnav_status` | Standard VNAV status |
| `sim/cockpit2/autopilot/fms_vnav` | FMS vertical control |
| `sim/cockpit/autopilot/altitude` | Selected (MCP) altitude fallback (read) |
| `sim/cockpit2/autopilot/flight_director_pitch_deg` | **Baseline** FD pitch (read only) |
| `sim/cockpit2/autopilot/flight_director_roll_deg` | Status / diagnostics |
| `sim/operation/override/override_nav_heading` | LNAV active gate |
| `sim/cockpit/radios/gps_course_degtm` | GPS desired track (mag) |
| `sim/cockpit/radios/gps_dme_dist_m` | GPS distance to active waypoint (nm) |
| `sim/cockpit/radios/gps_hdef_dot` | GPS CDI deflection (dots) |
| `sim/cockpit/radios/gps_hdef_nm_per_dot` | CDI scale (nm/dot) |
| `sim/cockpit/radios/gps_fromto` | GPS from/to flag |
| `sim/aircraft/view/acf_relative_path` | FF757 detection |
| `sim/aircraft/view/acf_ICAO` | FF757 detection fallback |

## Standard X-Plane datarefs (**WRITE**)

| Dataref | When | Notes |
| --- | --- | --- |
| `sim/cockpit/autopilot/nav_steer_deg_mag` | LNAV, only when `override_nav_heading=1` | This replaces only the target heading |
| `sim/cockpit/autopilot/flight_director_roll` | LNAV active | Legacy FD roll target; a small bank correction is added to the avionics baseline |
| `sim/operation/override/override_flightdir_roll` | LNAV active | Set to 1 while writing, restored to 0 if the plugin set it |
| `sim/cockpit2/autopilot/flight_director_roll_deg` | LNAV active, only if the probe succeeds | Used as baseline and written only when the value sticks |
| `sim/cockpit/autopilot/flight_director_pitch` | VNAV active / smoothing | Legacy FD pitch target |
| `sim/operation/override/override_flightdir_ptch` | VNAV active / smoothing | Set to 1 while writing, restored to 0 if the plugin was the one that set it |
| `sim/cockpit2/autopilot/flight_director_pitch_deg` | VNAV active, **only if the probe succeeds** | DataRefs.txt marks this read-only, but the FF757 writes it. The plugin probes with a small offset once and writes it only if the value sticks. It is used as the feedback-free baseline when not written. |

## FF757 avionics datarefs (read)

| Dataref | Assumed meaning |
| --- | --- |
| `757Avionics/ap/ap_mode` | AP mode enum (F/D, CMD) |
| `757Avionics/ap/roll_mode_engaged` | Lateral mode enum (HDG HOLD, HDG SEL, LNAV, ROLLOUT) |
| `757Avionics/ap/pitch_mode_engaged` | Vertical mode enum (V/S, ALT HOLD, ALT CAP, VNAV PTH, VNAV SPD, G/S, FLARE) |
| `757Avionics/ap/roll_mode_armed` | Armed lateral mode |
| `757Avionics/ap/pitch_mode_armed` | Armed vertical mode |
| `757Avionics/fms/vnav_phase` | VNAV phase (climb/cruise/descent/etc.) |
| `757Avionics/ap/leftFdState`, `rghtFdState` | FD state |
| `757Avionics/ap/warn_flags` | AP warnings |
| `757Avionics/ap/at_servos_is_active` | Autothrottle servo state |
| `757Avionics/ap/alt_act` | Selected (MCP) altitude, used as a climb ceiling / descent floor |
| `757Avionics/fms/AirnavDtk` | Active-leg desired track |
| `757Avionics/fms/AirnavDis` | Distance to active waypoint |
| `757Avionics/fms/AirnavXtk` | Cross-track |
| `757Avionics/fms/AirnavBrg` | Bearing to active waypoint |
| `757Avionics/fms/AirnavTke` | Track-angle error |
| `757Avionics/fms/AirnavAtd` | Along-track distance |
| `757Avionics/fms/AirnavFromTo` | From/to flag |
| `757Avionics/fms/xtk_per_dot` | Cross-track scale |
| `757Avionics/fms/anp`, `rnp` | Navigation performance |
| `757Avionics/fms/crz_alt`, `trans_alt` | Cruise / transition altitude |
| `757Avionics/fms/vmo`, `vmin` | Speed limits |

The literal `757Avionics` prefix is stable in the current FF757/767 avionics
plugin. These are **not** officially documented; the plugin treats them as
best-effort inputs.

## FF757 CDU datarefs (route fallback)

When the private route tree is unavailable, the CDU LEGS page becomes the
route source. The right-hand CDU is preferred; the left-hand CDU is used when
the right one is not on a LEGS page.

| Dataref | Use |
| --- | --- |
| `1-sim/cduL/display/numOfLines` | Left CDU line count (read) |
| `1-sim/cduL/display/numOfSymbolsInLine` | Left CDU line width (read) |
| `1-sim/cduL/display/symbols` | Left CDU character buffer (read) |
| `1-sim/cduR/display/numOfLines` | Right CDU line count (read) |
| `1-sim/cduR/display/numOfSymbolsInLine` | Right CDU line width (read) |
| `1-sim/cduR/display/symbols` | Right CDU character buffer (read) |
| `757Avionics/CDU/prev_page` | Left CDU PREV key (**WRITE**, pulse) |
| `757Avionics/CDU/next_page` | Left CDU NEXT key (**WRITE**, pulse) |
| `757Avionics/CDU2/prev_page` | Right CDU PREV key (**WRITE**, pulse) |
| `757Avionics/CDU2/next_page` | Right CDU NEXT key (**WRITE**, pulse) |

The scan only runs while the private `rteData` tree is missing (or while a
previously started scan is finishing). The current LEGS page is compared with
the cached route every 0.5 s; an altitude/leg edit or a route-name change
triggers an early rescan. A full rescan also runs every `cdu_rescan_interval_s`
(45 s default), in the air as well as on the ground.

Each key is pressed for about 60 ms, at most once per 0.35 s, and the CDU is
returned to the page it was on before the scan. Stall detection aborts a scan
that stops advancing after eight key presses; the overall scan limit is 25 s,
and page restore gives up after 8 s. A partial scan never replaces a complete
cached route.

## FF757 route datarefs (read, undocumented)

The avionics plugin publishes a route data tree. The base name observed in the
binary is:

```
1-sim/for-internal-usage-only/fms
```

The plugin tries these prefixes, in order:

```
1-sim/for-internal-usage-only/fms/<field>
757Avionics/fms/<field>
1-sim/fms/<field>
```

Fields:

| Field | Expected type | Meaning |
| --- | --- | --- |
| `rteData/rteCount` | int | Route leg count |
| `rteData/rteOffsets` | int[] | Leg start offsets into the waypoint arrays |
| `rteData/rteTypes` | int[] | Raw ARINC-style leg type |
| `rteData/rteSizes` | int[] | Number of points in each leg |
| `rteData/wptsLat` | float[] | Waypoint latitudes |
| `rteData/wptsLon` | float[] | Waypoint longitudes |
| `rteData/wptsEstimatedAlt` | float[] or string buffer | Estimated / constraint altitude |
| `rteData/wptsEstimatedAltStart` | int[] | Start offset into the altitude buffer |
| `rteData/wptsEstimatedAltLen` | int[] | Length of each altitude string |
| `rteData/wptsId` | byte/string buffer | Concatenated waypoint identifiers |
| `rteData/wptsIdStart` | int[] | Start offset into the identifier buffer |
| `rteData/wptsIdLen` | int[] | Length of each identifier |
| `rteData/actWpt/idx` | int | Active waypoint index |
| `rteData/actWpt/id` | string | Active waypoint identifier |
| `rteData/actWpt/distToGo` | float | Distance to go |
| `rteData/vtk` | float | Vertical track |

The plugin builds `legs[i] = [rteOffsets[i], rteOffsets[i] + rteSizes[i] - 1]`
and uses the active waypoint index to select the current and next leg. Multi-
point legs (arcs, holds, procedure turns) are treated conservatively: the
route model is rejected for LNAV and the GPS CDI fallback is used. Waypoint
altitudes are parsed from either a float array or a string buffer such as
`35000`, `FL350`, or `-----`.

Every route model is validated against the FF757's own GPS output:

- the computed current-leg course must agree with `gps_course_degtm` within
  25°;
- the computed distance to the active waypoint must agree with
  `gps_dme_dist_m` within `max(10 nm, 50%)`.

If validation fails, the plugin does not use route-based turn anticipation and
falls back to the GPS CDI. This is the main guardrail against a changed or
misinterpreted private route layout.

## Published datarefs

See the table in [`README.md`](../README.md#menu-commands-and-datarefs).
