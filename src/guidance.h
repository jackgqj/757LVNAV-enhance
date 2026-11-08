#pragma once

#include <string>

#include "config.h"
#include "route_model.h"

namespace lvnv {

struct AircraftState {
    bool valid = false;
    bool onGround = true;
    bool apEngaged = false;
    bool lnavOverride = false;
    bool lnavModeDetected = false;
    bool vnavMode = false;
    bool vnavSpeedMode = false;  // VNAV SPD: pitch is used to hold speed

    double lat = 0.0;
    double lon = 0.0;
    double trackTrueDeg = 0.0;
    double magVarDeg = 0.0;
    double gsKts = 0.0;
    double altFt = 0.0;
    double vsFpm = 0.0;
    double vsAccelFpmPerSec = 0.0;
    double iasKts = 0.0;
    double selectedAltFt = 0.0;  // MCP altitude (0 when unavailable)
    double fdRollDeg = 0.0;      // FF757-owned FD roll (positive right)
};

struct CdiInput {
    bool valid = false;
    bool xtkValid = false;
    double courseMagDeg = 0.0;
    double distanceNm = 0.0;
    double xtkNm = 0.0;          // CDI dot sign, or correction sign when signIsCorrection
    bool signIsCorrection = false;
    int fromTo = 0;
};

struct LnavResult {
    bool active = false;
    bool routeBased = false;
    bool cdiFallback = false;
    int activeWpt = -1;
    int activeLeg = -1;
    int nextLeg = -1;
    double xtkNm = 0.0;          // positive = right of course (route convention)
    double distanceNm = 0.0;
    double baseCourseMagDeg = 0.0;
    double desiredCourseMagDeg = 0.0;
    double correctionDeg = 0.0;
    double rollCorrectionDeg = 0.0;
    double turnProgress = 0.0;
    // RNP / ANP monitoring (777 FMC style).  rnpNm = 0 when monitoring is off.
    double rnpNm = 0.0;              // required navigation performance in effect
    double anpNm = 0.0;              // actual navigation performance estimate
    bool rnpUnable = false;          // ANP exceeded RNP, sustained (hysteresis)
    std::string source;
    std::string note;
};

struct VnavResult {
    bool active = false;
    bool pathBased = false;
    bool passThrough = false;    // true = do not write FD pitch at all
    double targetAltFt = 0.0;
    double requiredVsFpm = 0.0;
    double vsErrorFpm = 0.0;
    double correctionDeg = 0.0;
    // Integrated vertical-path deviation (ft, negative = above the profile).
    // Used to hold pitch authority while the aircraft is genuinely off path
    // instead of releasing into the FD deadband and handing pitch back to the
    // FF757 on the first small VS-error frame.
    double pathErrorFt = 0.0;
    bool pathHeld = false;
    // Speed-limit summary (FAR 250 KT below 10000 ft or DECEL point).  Used
    // for the status window and for pitch softening so the path ends at the
    // constraint at the right speed instead of forcing a hard brake at the
    // fix.  targetSpdKt = 0 means no speed constraint ahead.
    double targetSpdKt = 0.0;
    double targetSpdDistNm = 0.0;  // distance along the route to the fix
    double targetSpdLeadNm = 0.0;  // deceleration lead distance, 0 when none
    std::string source;
    std::string note;
};

class Guidance {
public:
    LnavResult computeLnav(const AircraftState& state,
                           const RouteSnapshot& route,
                           const CdiInput& cdi,
                           const Config& cfg);

    VnavResult computeVnav(const AircraftState& state,
                           const RouteSnapshot& route,
                           const Config& cfg,
                           double dtSec = 0.0);

    void reset();

    int cdiSign() const { return cdiSign_; }

private:
    int cdiSign_ = 1;   // +1: positive CDI dot means "turn right"
    int cdiDisagreeCount_ = 0;
    int fdRollDisagreeCount_ = 0;
    // Set once a trusted route cross-track has confirmed the dot sign.  The
    // FD-roll bootstrap must not flip it afterwards: during a recovery turn
    // the FF757 banks towards its own course while the aircraft is still off
    // the CDI, which reads as the opposite sign.
    bool cdiSignVerified_ = false;

    // CDI cross-check hysteresis.  A single frame of disagreement (route
    // rescan, sequencing, a lagging FMS course) must not drop LNAV to GPS CDI
    // steering; both checks need ~2 s of continuous disagreement.
    int cdiCourseRejectCount_ = 0;
    int cdiDistRejectCount_ = 0;

    // Short route gap: hold the last route-based command for ~1 s when the
    // snapshot is momentarily unusable instead of falling back to CDI dots.
    bool holdValid_ = false;
    int holdFrames_ = 0;
    double holdCourseMagDeg_ = 0.0;
    double holdXtkNm_ = 0.0;
    double holdDistNm_ = 0.0;

    // ANP estimator: low-passed mean |cross-track|, doubled for a ~95%
    // containment proxy.  The alert uses hysteresis so a noisy edge does not
    // flash the message.
    double anpMeanAbsXtk_ = 0.0;
    bool anpInit_ = false;
    int rnpExceedCount_ = 0;
    int rnpRecoverCount_ = 0;
    bool rnpUnable_ = false;

    // RF arc tracking state.  Arc progress is unwrapped incrementally so arcs
    // longer than 180 deg do not wrap; the state is re-initialised whenever a
    // different leg becomes active.
    int rfLegIdx_ = -1;
    double rfLastRadialDeg_ = 0.0;
    double rfProgressDeg_ = 0.0;

    // VNAV pitch-authority state.  vnavPathHeld_ keeps the plugin on pitch
    // after a correction starts, with hysteresis, until both the VS error and
    // the integrated path deviation are small.  vnavPathErrorFt_ integrates
    // vsError (ft, negative = above profile) so a small instantaneous error
    // cannot hide a persistent altitude deviation.
    bool vnavPathHeld_ = false;
    double vnavPathErrorFt_ = 0.0;
    int vnavTargetIndex_ = -1;

    // Shared RNP/ANP monitor: phase RNP selection, ANP low-pass and the
    // hysteresis alert.  absLateralErrNm is |cross-track| on straight legs or
    // |radial error| on RF arcs.
    void updateRnp(const Config& cfg, double absLateralErrNm,
                   double distToDestNm, LnavResult& out);
};

} // namespace lvnv
