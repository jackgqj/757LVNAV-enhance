#pragma once

#include <algorithm>
#include <cmath>

namespace lvnv {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusNm = 3440.065;      // mean earth radius
constexpr double kMetersPerNm = 1852.0;
constexpr double kFeetPerNm = 6076.12;
constexpr double kFtPerMinPerVs = 1.0;
constexpr double kKtPerMps = 1.9438444924406;
constexpr double kMpsPerKt = 0.5144444444444;

inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / kPi; }

inline double clampd(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

inline double normalize180(double deg) {
    deg = std::fmod(deg + 180.0, 360.0);
    if (deg < 0.0) deg += 360.0;
    return deg - 180.0;
}

inline double normalize360(double deg) {
    deg = std::fmod(deg, 360.0);
    if (deg < 0.0) deg += 360.0;
    return deg;
}

inline double smoothstep01(double t) {
    t = clampd(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

inline double distanceNm(double lat1, double lon1, double lat2, double lon2) {
    const double phi1 = deg2rad(lat1);
    const double phi2 = deg2rad(lat2);
    const double dphi = deg2rad(lat2 - lat1);
    const double dlam = deg2rad(lon2 - lon1);
    const double a = std::sin(dphi * 0.5) * std::sin(dphi * 0.5) +
                     std::cos(phi1) * std::cos(phi2) *
                         std::sin(dlam * 0.5) * std::sin(dlam * 0.5);
    const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(std::max(0.0, 1.0 - a)));
    return kEarthRadiusNm * c;
}

inline double bearingTrue(double lat1, double lon1, double lat2, double lon2) {
    const double phi1 = deg2rad(lat1);
    const double phi2 = deg2rad(lat2);
    const double dlam = deg2rad(lon2 - lon1);
    const double y = std::sin(dlam) * std::cos(phi2);
    const double x = std::cos(phi1) * std::sin(phi2) -
                     std::sin(phi1) * std::cos(phi2) * std::cos(dlam);
    return normalize360(rad2deg(std::atan2(y, x)));
}

// Signed cross-track distance, positive when the point is to the right of the
// great-circle course A -> B.
inline double crossTrackNm(double lat, double lon,
                           double latA, double lonA,
                           double latB, double lonB) {
    const double d13 = distanceNm(latA, lonA, lat, lon) / kEarthRadiusNm;
    const double brg13 = deg2rad(bearingTrue(latA, lonA, lat, lon));
    const double brg12 = deg2rad(bearingTrue(latA, lonA, latB, lonB));
    const double x = std::asin(std::max(-1.0, std::min(1.0,
                              std::sin(d13) * std::sin(brg13 - brg12))));
    return x * kEarthRadiusNm;
}

inline double alongTrackNm(double lat, double lon,
                           double latA, double lonA,
                           double latB, double lonB) {
    const double d13 = distanceNm(latA, lonA, lat, lon) / kEarthRadiusNm;
    const double xtk = crossTrackNm(lat, lon, latA, lonA, latB, lonB) / kEarthRadiusNm;
    const double c = std::cos(d13) / std::cos(xtk);
    if (c <= -1.0 || c >= 1.0) {
        // Degenerate / pole case: fall back to the planar projection.
        return d13 * kEarthRadiusNm *
               std::cos(deg2rad(bearingTrue(latA, lonA, lat, lon) -
                                bearingTrue(latA, lonA, latB, lonB)));
    }
    return kEarthRadiusNm * std::acos(std::max(-1.0, std::min(1.0, c)));
}

inline double turnRadiusNm(double gsKts, double bankDeg) {
    const double v = std::max(30.0, gsKts) * kMpsPerKt;
    const double bank = deg2rad(clampd(bankDeg, 5.0, 45.0));
    const double rMeters = (v * v) / (9.80665 * std::tan(bank));
    return rMeters / kMetersPerNm;
}

// Point reached from (lat, lon) after travelling distNm along bearingDeg.
inline void destinationPoint(double lat, double lon, double bearingDeg,
                              double distNm, double& outLat, double& outLon) {
    const double d = std::max(0.0, distNm) / kEarthRadiusNm;
    const double brg = deg2rad(bearingDeg);
    const double phi1 = deg2rad(lat);
    const double sinPhi2 = std::sin(phi1) * std::cos(d) +
                           std::cos(phi1) * std::sin(d) * std::cos(brg);
    const double phi2 = std::asin(clampd(sinPhi2, -1.0, 1.0));
    const double dlon = std::atan2(std::sin(brg) * std::sin(d) * std::cos(phi1),
                                   std::cos(d) - std::sin(phi1) * std::sin(phi2));
    outLat = rad2deg(phi2);
    outLon = normalize180(rad2deg(deg2rad(lon) + dlon));
}

inline double magneticFromTrue(double trueDeg, double magVarDeg) {
    return normalize360(trueDeg + magVarDeg);
}

inline double trueFromMagnetic(double magDeg, double magVarDeg) {
    return normalize360(magDeg - magVarDeg);
}

// X-Plane reports vertical path (vpath) as a flight path angle in degrees.
inline double vsForPathDeg(double pathDeg, double gsKts) {
    return std::tan(deg2rad(clampd(pathDeg, -20.0, 20.0))) *
           std::max(0.0, gsKts) * 101.27;
}

} // namespace lvnv
