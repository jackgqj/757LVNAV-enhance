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
                           const Config& cfg);

    void reset();

    int cdiSign() const { return cdiSign_; }

private:
    int cdiSign_ = 1;   // +1: positive CDI dot means "turn right"
    int cdiDisagreeCount_ = 0;
    int fdRollDisagreeCount_ = 0;
};

} // namespace lvnv
