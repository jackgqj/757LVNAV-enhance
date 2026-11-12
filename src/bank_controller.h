#pragma once

namespace lvnv {

// Pure bank command used by the direct-bank (RNAV1) lateral mode.  Positive
// bank is right wing down.  `desiredTrackDeg` and `trackDeg` are in the same
// reference frame; `xtkNm` is positive when the aircraft is right of course.
double ComputeBankCommandDeg(double desiredTrackDeg, double trackDeg,
                             double xtkNm, double distanceNm, double kTrack,
                             double kXtk, double maxBankDeg);

} // namespace lvnv
