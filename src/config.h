#pragma once

#include <string>

namespace lvnv {

struct Config {
    // master switches
    bool enabled = true;
    bool enableLnav = true;
    bool enableVnav = true;
    bool enableVnavSmoothing = true;
    // Safety default: the FD-roll bridge is opt-in until it has been flight
    // tested.  The FF757 build observed here reports FD roll 0 in several
    // states; writing a bank correction there can start an unwanted turn.
    bool enableLnavFdRoll = false;

    // LNAV
    double lnavGain = 0.65;
    // Boeing course capture: |xtk| beyond lnavXtkBlendNm commands a constant
    // lnavMaxInterceptDeg intercept back to the course line; small offsets
    // home direct to the fix, allowed up to lnavDirectInterceptDeg.
    double lnavMaxInterceptDeg = 30.0;
    double lnavDirectInterceptDeg = 45.0;
    double lnavXtkBlendNm = 2.5;
    // 1.0 = the turn starts exactly at the tangent point of a steady arc, so
    // the aircraft neither cuts the corner nor overshoots the next course.
    double lnavTurnLeadFactor = 1.0;
    double lnavMaxBankDeg = 25.0;
    // Bank assumed when turning a fly-by corner.  The FF757's LNAV usually
    // rolls to only ~15-18 deg at cruise speed, while lnavMaxBankDeg (25) is
    // the structural/bank-limit value also used by RF arcs.  Using 25 for the
    // fly-by radius started the turn ~1.7 NM early at 400 KT: the aircraft cut
    // inside the new course and then overshot the following corner.  The value
    // is scheduled towards lnavTurnBankHighDeg (18) as speed rises, because
    // the AFDS bank limit decreases with airspeed (AUTO 15-25 deg by TAS).
    double lnavTurnBankDeg = 17.0;
    double lnavTurnBankLowKts = 180.0;
    double lnavTurnBankHighKts = 260.0;
    // Bank assumed at/above the high-speed threshold.  The AFDS reduces its
    // bank limit with airspeed; raise this only if the aircraft is observed
    // holding a steeper bank in LNAV at cruise speed.
    double lnavTurnBankHighDeg = 18.0;
    // Hold the previous leg while the FMS has sequenced early (fly-by
    // sequencing happens up to a minute before the fix at high speed).  When
    // false, guidance follows the FMS-active leg immediately (old behaviour).
    bool lnavTurnLatch = true;
    double lnavMaxCourseErrorDeg = 60.0;
    // Degrees of arc completed early before the target course is frozen on the
    // outbound leg, so the autopilot's own capture lag rolls out on course
    // instead of overshooting through it.
    double lnavRolloutLeadDeg = 4.0;
    // RF (constant-radius-to-fix) leg tracking from the CIFP procedure
    // database.  When false, RF legs pass through to the FF757's polygonal
    // approximation like any other complex leg.
    bool lnavRfEnabled = true;
    // Allow a small FD roll nudge on RF arcs only.  The FF757 often leaves
    // override_nav_heading at 0, so the nav target alone cannot hold the
    // published radius; a +/-4 deg roll correction actually bends the path
    // without touching any other leg.
    bool lnavRfFdRoll = true;
    // Force sim/operation/override/override_nav_heading=1 while an RF arc is
    // tracked (heading-direct guidance).  X-Plane's nav controller then flies
    // the plugin's nav_steer target, converted from track to heading with the
    // wind-correction angle and track damping (see lnavTrackDamp*).  The model
    // clears the flag every frame, so it is re-asserted every frame like the
    // 787 reference script; FD roll is skipped while it is live so only one
    // lateral controller commands roll.
    bool lnavRfNavOverride = true;
    // Heading-direct guidance on **all** LNAV legs, not just RF arcs.  The
    // plugin holds override_nav_heading=1 and writes nav_steer as a heading
    // (desired track + wind correction + track damping), so the lateral path is
    // actually flown by this plugin instead of only monitored.  Disable to
    // leave straight legs to the FF757 and keep the override RF-only.
    bool lnavHeadingDirect = true;
    // Force the GPS CDI as the lateral source instead of the parsed route.
    // Useful when the CDU route geometry is suspect (mis-resolved fixes or
    // procedure waypoints): the plugin then tracks the FF757 GPS CDI directly,
    // exactly like its automatic fallback, until the switch is turned off.
    bool lnavForceCdi = false;
    // L1-style cross-track term on straight legs (787 reference).  The basic
    // course capture homes at atan(xtk / distance-to-fix), which on a long leg
    // commands only a fraction of a degree and leaves a small steady offset
    // (e.g. 0.15 NM).  This adds the difference to an L1 look-ahead correction
    // (L1 = factor x turn radius, 0.5-8 NM), clamped to +/-4 deg, in
    // heading-direct mode only.  The 1.6 factor puts L1 at the 8 NM cap above
    // ~400 KT, so a 0.15 NM offset commands about 1 deg rather than 1.7 deg.
    // Set 0 to disable.
    double lnavL1Factor = 1.6;
    // Cap on the extra L1 look-ahead correction added to the heading target.
    // 4 deg was too small after a high-speed turn: with a 5-6 NM overshoot the
    // L1 term saturated at 4 deg while the base 30 deg intercept could not
    // close the offset before the next fix.  8 deg keeps the post-turn capture
    // firm without changing the small-offset behaviour (where the term is far
    // below the cap).
    double lnavL1MaxDeg = 8.0;
    // Heading-direct RF command shaping (787 reference): damp = -gain *
    // (actual track - desired track), clamped to +/- max, then converted to a
    // heading with the wind-correction angle.  Suppresses oscillation and
    // wind-induced drift while the nav controller flies the command.
    double lnavTrackDampGain = 0.35;
    double lnavTrackDampMaxDeg = 10.0;
    double lnavMinSpeedKts = 60.0;
    double lnavMinDistanceNm = 0.05;
    double lnavRollGain = 0.25;
    double lnavMaxRollCorrectionDeg = 4.0;
    double lnavRollRateLimitDegPerSec = 3.0;
    double lnavRollSmoothingTauSec = 0.10;
    // Direct bank mode (RNAV1 experiment): write an absolute FD roll target
    // computed from track error and cross-track instead of only a nav_steer
    // heading.  The FF757 AP's own bank limit still applies, but this avoids
    // the heading-direct controller rolling out before the turn is complete.
    bool lnavBankDirect = true;
    double lnavBankKTrack = 1.4;
    double lnavBankKXtk = 0.8;
    double lnavBankMaxDeg = 25.0;
    double lnavBankRateDegPerSec = 6.0;
    // Small deadband/low-pass on the direct-bank command.  Track and
    // cross-track are instantaneous values, so turbulence otherwise feeds
    // straight into the bank target.
    double lnavBankDeadbandDeg = 0.5;
    double lnavBankDeadbandNm = 0.02;
    double lnavBankFilterTauSec = 0.10;
    // Mild track-rate damping: oppose a developing turn rate so a large
    // capture does not keep 25 deg of bank past the course.
    double lnavBankTrackRateDamp = 0.5;

    // RNP / ANP monitoring (Boeing 777 FMC logic).  RNP is the 95% lateral
    // accuracy requirement of the flight phase; ANP is the 95% estimate of
    // the actual navigation performance.  When ANP exceeds RNP the FMC raises
    // "NAV UNABLE RNP"; the plugin mirrors that in the status window and log.
    bool rnpEnabled = true;
    // 0 = automatic by flight phase: enroute 2.0 NM, terminal 1.0 NM within
    // 30 NM of the destination, approach 0.3 NM on the final segment.
    double rnpNm = 0.0;

    // VNAV
    // 1.2 deg of FD pitch per 1000 fpm of VS error is about 85% of the
    // physical inverse model at 400 KT (1 deg ~ 707 fpm), so the old 0.8
    // gain felt slow: a 945 fpm error commanded only 0.75 deg (~450 fpm).
    double vnavVsGain = 1.20;
    double vnavAccelGain = 0.0015;
    double vnavMaxCorrectionDeg = 2.5;
    double vnavRateLimitDegPerSec = 3.0;
    double vnavSmoothingTauSec = 0.10;
    double vnavMaxVsFpm = 3000.0;
    double vnavMinDistanceNm = 0.30;
    double vnavMaxAltDeltaFt = 50000.0;
    // VNAV SPD holds speed with pitch; injecting a path correction there
    // breaks the speed loop.  Opt in only for flight testing.
    bool vnavAugmentSpeedMode = false;
    // Respect the MCP selected altitude as a climb ceiling / descent floor.
    bool vnavRespectSelectedAlt = true;
    // Plugin pitch priority during a valid descent path.  The FF757's own
    // VNAV and the plugin both write the FD pitch dataref, so without a latch
    // they alternate ("夺权").  With this enabled the plugin holds pitch
    // authority for as long as its vertical path is valid and only releases
    // after ~2 s of inactivity.  Set 0 to give the FF757 priority again.
    bool vnavPriority = true;
    // VNAV speed-limit handling.  When true, the plugin searches ahead for
    // waypoints with a hard speed constraint ("250/10000 ft" or a DECEL fix)
    // and computes the deceleration lead distance (1 NM per 10 KT slowdown,
    // matching the 777 FMS rule).  The pitch path is then softened so the
    // aircraft reaches the constraint at the published speed instead of
    // having to brake hard at the fix.  The 10000 ft hard restriction
    // (250 KT) is also enforced automatically even when the CDU does not
    // list a constraint, matching FAR 91.117.
    bool vnavSpeedLimitsEnabled = true;
    // Deceleration lead: NM per 10 KT of slowdown.  777 uses 1.0; a heavier
    // bird may want a bit more (e.g. 1.1).  0 disables the lead, which forces
    // the speed to be met exactly at the fix.
    double vnavSpeedDecelFactorNmPer10Kt = 1.0;
    // 250 KT restriction ceiling below 10000 ft.  0 disables auto-enforcement;
    // otherwise the limit is applied to every waypoint below this altitude.
    double vnavHard250BelowFt = 10000.0;

    // Zibo/777-style vertical profile (opt-in).  Instead of chasing the next
    // altitude constraint with a single chord, build one continuous idle
    // descent path backwards from the last constraint, with every constraint
    // becoming a breakpoint and an E/D at the bottom.  Pitch follows the local
    // path gradient plus a deviation term, which is what the real FMC does in
    // VNAV PTH.  Legacy target logic is untouched and is used when this is off
    // or the profile is not usable (climb, no constraints, implausible path).
    bool vnavPathProfile = false;
    // Nominal idle descent angle used between constraints.
    double vnavPathAngleDeg = 3.0;
    // Extra fpm per foot of path deviation (- above path, + below path).
    double vnavPathDevGain = 0.6;

    // Route cache
    // 0 disables the periodic timer; the default trigger is a visible LEGS
    // page content change (once per change).
    double cduRescanIntervalSec = 0.0;
    // Periodic rescans are opt-in.  The default trigger is a visible page
    // content change; this flag must be set for cduRescanIntervalSec to apply.
    bool cduPeriodicRescan = false;
    // EXEC-only scanning: wait for the CDU to show ACT RTE (MOD -> ACT) and
    // scan once.  Default false: scan once whenever the page content changes.
    bool cduScanOnExecOnly = false;

    // CIFP departure procedure overlay.  When enabled, the active SID is
    // detected by matching the route's first fixes against the airport's CIFP
    // SIDs; the procedure prefix (including RF arcs) is merged ahead of the
    // FMS route at the last common fix.  Disable to use the raw FMS/CDU route.
    bool cifpProcedureRoute = true;

    // FF757 mode codes.  These are undocumented enum values; the defaults
    // match the order of the labels in the FF757 avionics binary.
    // Values observed in the FF757 v2 build: AP CMD=1, LNAV roll mode=2,
    // VNAV pitch modes 0, 5 and 6.  The pitch enum only says "VNAV engaged";
    // the SPD/PTH split comes from vnav_phase (0-2 = climb/SPD, 3+ = path)
    // and actual vertical speed.  -1 disables a slot if a build reports a
    // different value.
    int lnavRollMode = 2;
    int vnavPitchModeA = 0;
    int vnavPitchModeB = 6;
    int vnavPitchModeC = 5;
    int apCmdMode = 1;
    bool lnavWideMode = false;  // treat every non-zero roll mode as LNAV
    bool forceLnav = false;
    bool forceVnav = false;

    // diagnostics
    bool debug = false;

    bool load(const std::string& path);
    bool save(const std::string& path) const;
    void clampValues();
};

} // namespace lvnv
