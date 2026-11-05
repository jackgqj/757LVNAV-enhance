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
    double lnavMaxInterceptDeg = 25.0;
    // 1.0 = the turn starts exactly at the tangent point of a steady arc, so
    // the aircraft neither cuts the corner nor overshoots the next course.
    double lnavTurnLeadFactor = 1.0;
    double lnavMaxBankDeg = 25.0;
    double lnavMaxCourseErrorDeg = 60.0;
    // Minimum distance used as the intercept denominator.  Without it the
    // atan2 term explodes in the last mile before a fix.
    double lnavInterceptDistanceNm = 1.5;
    double lnavMinSpeedKts = 60.0;
    double lnavMinDistanceNm = 0.05;
    double lnavRollGain = 0.25;
    double lnavMaxRollCorrectionDeg = 4.0;
    double lnavRollRateLimitDegPerSec = 3.0;
    double lnavRollSmoothingTauSec = 0.10;

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
