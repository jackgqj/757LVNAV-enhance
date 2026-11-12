#include "bank_controller.h"

#include "geo.h"

#include <algorithm>
#include <cmath>

namespace lvnv {

double ComputeBankCommandDeg(double desiredTrackDeg, double trackDeg,
                             double xtkNm, double distanceNm, double kTrack,
                             double kXtk, double maxBankDeg) {
    const double trackErr = normalize180(desiredTrackDeg - trackDeg);
    const double lookahead = clampd(0.5 * distanceNm, 1.5, 5.0);
    // Positive xtk = aircraft right of course -> bank left (negative).
    const double xtkAngle = rad2deg(std::atan2(-xtkNm, lookahead));
    double bank = kTrack * trackErr + kXtk * xtkAngle;
    if (std::abs(trackErr) < 0.3 && std::abs(xtkNm) < 0.05) bank = 0.0;
    const double limit = std::max(1.0, maxBankDeg);
    return clampd(bank, -limit, limit);
}

} // namespace lvnv
