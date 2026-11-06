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
    double lnavMaxCourseErrorDeg = 60.0;
    // Degrees of arc completed early before the target course is frozen on the
    // outbound leg, so the autopilot's own capture lag rolls out on course
    // instead of overshooting through it.
    double lnavRolloutLeadDeg = 3.0;
    // RF (constant-radius-to-fix) leg tracking from the CIFP procedure
    // database.  When false, RF legs pass through to the FF757's polygonal
    // approximation like any other complex leg.
    bool lnavRfEnabled = true;
    double lnavMinSpeedKts = 60.0;
    double lnavMinDistanceNm = 0.05;
    double lnavRollGain = 0.25;
    double lnavMaxRollCorrectionDeg = 4.0;
    double lnavRollRateLimitDegPerSec = 3.0;
    double lnavRollSmoothingTauSec = 0.10;

    // RNP / ANP monitoring (Boeing 777 FMC logic).  RNP is the 95% lateral
    // accuracy requirement of the flight phase; ANP is the 95% estimate of
    // the actual navigation performance.  When ANP exceeds RNP the FMC raises
    // "NAV UNABLE RNP"; the plugin mirrors that in the status window and log.
    bool rnpEnabled = true;
    // 0 = automatic by flight phase: enroute 2.0 NM, terminal 1.0 NM within
    // 30 NM of the destination, approach 0.3 NM on the final segment.
    double rnpNm = 0.0;

    // VNAV
    double vnavVsGain = 0.80;
    double vnavAccelGain = 0.0015;
    double vnavMaxCorrectionDeg = 2.5;
    double vnavRateLimitDegPerSec = 3.0;
    double vnavSmoothingTauSec = 0.12;
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
