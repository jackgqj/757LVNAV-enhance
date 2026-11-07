#include "guidance.h"

#include "geo.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdio>

namespace lvnv {

namespace {

double currentMagTrack(const AircraftState& s) {
    return magneticFromTrue(s.trackTrueDeg, s.magVarDeg);
}

double clampCourseError(double desiredMag, double currentMag, double maxError) {
    double err = normalize180(desiredMag - currentMag);
    if (std::abs(err) > maxError) {
        err = (err > 0.0) ? maxError : -maxError;
    }
    return normalize360(currentMag + err);
}

double lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

struct VnavTarget {
    bool valid = false;
    int index = -1;
    double distanceNm = 0.0;   // along the route from the current position
    int altFt = 0;
    std::string id;
};

// Walks forward from the active waypoint and finds the first altitude that
// differs from the current altitude.  The distance is accumulated along the
// route, so a target several legs ahead is handled correctly.
VnavTarget FindVnavTarget(const RouteSnapshot& route, const AircraftState& state,
                          const Config& cfg) {
    VnavTarget target;
    if (!route.valid || route.activeWpt < 0) return target;

    const RouteWaypoint* active = route.waypoint(route.activeWpt);
    if (!active) return target;

    double cum = distanceNm(state.lat, state.lon, active->lat, active->lon);
    if (!std::isfinite(cum) || cum < 0.05) cum = 0.05;

    // Do not chase an altitude target that is only a few miles ahead; the
    // required vertical speed would be physically impossible and would make
    // the FD pitch oscillate.
    const double minDistance = std::max(cfg.vnavMinDistanceNm, 3.0);

    for (int i = route.activeWpt; i < route.wptCount; ++i) {
        const RouteWaypoint* w = route.waypoint(i);
        if (!w) break;
        if (i > route.activeWpt) {
            const RouteWaypoint* prev = route.waypoint(i - 1);
            if (!prev || !std::isfinite(prev->lat) || !std::isfinite(prev->lon) ||
                !std::isfinite(w->lat) || !std::isfinite(w->lon)) {
                break;
            }
            cum += distanceNm(prev->lat, prev->lon, w->lat, w->lon);
        }
        if (!w->altValid) continue;
        if (std::abs(static_cast<double>(w->altFt) - state.altFt) < 100.0) continue;
        if (cum < minDistance) continue;

        target.valid = true;
        target.index = i;
        target.distanceNm = cum;
        target.altFt = w->altFt;
        target.id = w->id;
        break;
    }
    return target;
}

// A speed-limit target ahead of the aircraft.  Either a CDU-declared
// constraint ("250" printed after the leg's NM) or the FAR hard limit of
// 250 KT below 10000 ft applied to the first low-altitude waypoint.
// decelLeadNm = 0 when there is no deceleration needed; otherwise 1 NM per
// 10 KT slowdown from the current IAS, matching the 777 FMS rule.
struct SpeedTarget {
    bool valid = false;
    int index = -1;
    double distanceNm = 0.0;
    int speedKt = 0;
    double decelLeadNm = 0.0;
    bool auto250 = false;
    std::string id;
};

SpeedTarget FindSpeedTarget(const RouteSnapshot& route, const AircraftState& state,
                            const Config& cfg) {
    SpeedTarget t;
    if (!cfg.vnavSpeedLimitsEnabled) return t;
    if (!route.valid || !route.sourceRoute || route.activeWpt < 0) return t;

    const RouteWaypoint* active = route.waypoint(route.activeWpt);
    if (!active) return t;

    double cum = distanceNm(state.lat, state.lon, active->lat, active->lon);
    if (!std::isfinite(cum) || cum < 0.05) cum = 0.05;

    // Current reference speed for computing deceleration lead.  Use IAS when
    // available, fall back to ground speed, but keep the delta anchored to
    // something physical - the same 1 NM / 10 KT ratio the 777's FMS uses.
    const double curKt = (state.iasKts > 50.0 && state.iasKts < 500.0)
                             ? state.iasKts
                             : state.gsKts;

    // FAR 250 KT below 10000 ft auto-enforcement applies only to waypoints
    // the aircraft has not yet passed through.  Skip any waypoint that is
    // currently behind us (within ~0.3 NM of the active position).
    const double leadFactor = cfg.vnavSpeedDecelFactorNmPer10Kt;

    for (int i = route.activeWpt; i < route.wptCount; ++i) {
        const RouteWaypoint* w = route.waypoint(i);
        if (!w) break;
        if (i > route.activeWpt) {
            const RouteWaypoint* prev = route.waypoint(i - 1);
            if (!prev || !std::isfinite(prev->lat) || !std::isfinite(prev->lon) ||
                !std::isfinite(w->lat) || !std::isfinite(w->lon)) break;
            cum += distanceNm(prev->lat, prev->lon, w->lat, w->lon);
        }

        int limit = 0;
        bool autoFlag = false;
        if (w->constraintSpeedKt > 0 && w->constraintSpeedKt <= 500) {
            limit = w->constraintSpeedKt;
        } else if (cfg.vnavHard250BelowFt > 0.0 && w->altValid &&
                   w->altFt < cfg.vnavHard250BelowFt && curKt > 270.0) {
            // FAR 91.117(a): 250 KT below 10000 ft.  Only apply when we are
            // actually above 270 KT; the aircraft is already slower, no need.
            limit = 250;
            autoFlag = true;
        }
        if (limit <= 0 || limit >= curKt - 2.0) continue;  // no slowdown needed

        t.valid = true;
        t.index = i;
        t.distanceNm = cum;
        t.speedKt = limit;
        t.auto250 = autoFlag;
        t.id = w->id;
        if (leadFactor > 0.0) {
            const double deltaKt = std::max(0.0, curKt - static_cast<double>(limit));
            t.decelLeadNm = leadFactor * deltaKt / 10.0;
        } else {
            t.decelLeadNm = 0.0;
        }
        return t;
    }
    return t;
}

} // namespace

void Guidance::reset() {
    cdiSign_ = 1;
    cdiDisagreeCount_ = 0;
    fdRollDisagreeCount_ = 0;
    cdiCourseRejectCount_ = 0;
    cdiDistRejectCount_ = 0;
    holdValid_ = false;
    holdFrames_ = 0;
    holdCourseMagDeg_ = 0.0;
    holdXtkNm_ = 0.0;
    holdDistNm_ = 0.0;
    anpMeanAbsXtk_ = 0.0;
    anpInit_ = false;
    rnpExceedCount_ = 0;
    rnpRecoverCount_ = 0;
    rnpUnable_ = false;
    rfLegIdx_ = -1;
    rfLastRadialDeg_ = 0.0;
    rfProgressDeg_ = 0.0;
}

// Remaining route distance from a waypoint to the destination, seeded with
// the distance from the aircraft to that waypoint.
namespace {
double DistanceToDestination(const RouteSnapshot& route, int fromWptIdx,
                             double initialNm) {
    double d = initialNm;
    for (int i = fromWptIdx + 1; i < route.wptCount; ++i) {
        const RouteWaypoint* a = route.waypoint(i - 1);
        const RouteWaypoint* b = route.waypoint(i);
        if (!a || !b || !a->posValid || !b->posValid) break;
        d += distanceNm(a->lat, a->lon, b->lat, b->lon);
    }
    return d;
}
} // namespace

void Guidance::updateRnp(const Config& cfg, double absLateralErrNm,
                         double distToDestNm, LnavResult& out) {
    // RNP is the 95% lateral accuracy requirement of the flight phase; ANP is
    // the 95% estimate of actual performance.  Phase defaults: enroute RNP
    // 2.0, terminal RNP 1.0 within 30 NM of the destination (the AC 91-006
    // terminal alerting boundary), approach RNP 0.3 on the final segment.
    double rnp = cfg.rnpNm;
    if (rnp <= 0.0) {
        rnp = 2.0;
        if (distToDestNm < 30.0) rnp = 1.0;
        if (distToDestNm < 8.0)  rnp = 0.3;
    }

    // 95% containment proxy: low-pass the mean |cross-track| and double it.
    // computeLnav runs once per flight-loop frame (~20 Hz), so alpha 0.02
    // gives a ~2.5 s time constant - fast enough to follow a rejoin, slow
    // enough not to chase single-frame noise.
    if (!anpInit_) {
        anpMeanAbsXtk_ = absLateralErrNm;
        anpInit_ = true;
    } else {
        anpMeanAbsXtk_ += (absLateralErrNm - anpMeanAbsXtk_) * 0.02;
    }
    const double anp = 2.0 * anpMeanAbsXtk_;

    if (cfg.rnpEnabled) {
        // ~2 s of continuous violation raises the alert; clearing needs ANP
        // back below 80% of RNP, so a noisy edge cannot flash the message.
        if (anp > rnp) {
            if (++rnpExceedCount_ >= 40) rnpUnable_ = true;
            rnpRecoverCount_ = 0;
        } else if (anp < 0.8 * rnp) {
            if (++rnpRecoverCount_ >= 40) rnpUnable_ = false;
            rnpExceedCount_ = 0;
        }
        out.rnpNm = rnp;
    } else {
        rnpUnable_ = false;
        rnpExceedCount_ = 0;
        rnpRecoverCount_ = 0;
        out.rnpNm = 0.0;
    }
    out.anpNm = anp;
    out.rnpUnable = rnpUnable_;
}

LnavResult Guidance::computeLnav(const AircraftState& state,
                                 const RouteSnapshot& route,
                                 const CdiInput& cdi,
                                 const Config& cfg) {
    LnavResult out;

    if (!state.valid) {
        out.note = "aircraft state invalid";
        return out;
    }
    if (state.onGround) {
        out.note = "on ground";
        return out;
    }
    if (!state.apEngaged) {
        out.note = "autopilot not engaged";
        return out;
    }
    if (!state.lnavModeDetected && !cfg.forceLnav) {
        out.note = "FF757 LNAV mode not detected (check lnav_roll_mode)";
        return out;
    }
    if (state.gsKts < cfg.lnavMinSpeedKts) {
        out.note = "ground speed too low";
        return out;
    }

    const double currentTrackMag = currentMagTrack(state);
    const double maxError = cfg.lnavMaxCourseErrorDeg;

    bool useRoute = route.valid && route.sourceRoute &&
                    route.activeWpt >= 0 && route.activeLeg >= 0;
    bool complexLeg = false;
    double routeXtkRight = 0.0;
    double routeDist = 0.0;
    double routeCourseMag = 0.0;
    double turnProgress = 0.0;

    if (useRoute) {
        const RouteLeg* leg = route.leg(route.activeLeg);

        // An RF leg becomes active many miles before its arc entry.  Tracking
        // the tangent from there commanded a course up to 50-60 deg off the
        // inbound leg (seen at ZULS: 82 deg vs the FMS course of 33 deg) and
        // produced a ~7 NM phantom cross-track plus a false NAV UNABLE RNP.
        // Take over only once the aircraft is actually on the circle, at or
        // past the entry radial, and only when the arc's entry tangent agrees
        // with the FMS course.
        auto rfEngaged = [&](const RouteLeg* l) {
            if (!l->rfArc || !cfg.lnavRfEnabled || l->rfTurn == 0) return false;
            if (l->rfRadiusNm < 0.3 || l->rfTotalSweptDeg < 5.0) return false;
            const RouteWaypoint* exitW = route.waypoint(l->lastWpt);
            if (!exitW || !exitW->posValid) return false;
            const double dir = static_cast<double>(l->rfTurn);
            const double radialNow =
                bearingTrue(l->rfCenterLat, l->rfCenterLon, state.lat, state.lon);
            const double radialErr =
                distanceNm(l->rfCenterLat, l->rfCenterLon, state.lat, state.lon) -
                l->rfRadiusNm;
            // Before capture the aircraft may be miles off the circle; once we
            // are tracking the arc, allow a wider band so a bank-limited arc
            // does not drop out every frame.
            const bool alreadyTracking =
                rfLegIdx_ == route.activeLeg && rfProgressDeg_ > 0.5;
            if (std::abs(radialErr) > (alreadyTracking ? 3.0 : 1.5)) return false;
            const double swept =
                normalize180(dir * (radialNow - l->rfRadialStartDeg));
            // Purely geometric gate: no FMS/CDI cross-check.  On RNP segments
            // the FF757's own route computation is unreliable, so the arc is
            // accepted from the CIFP geometry alone.
            return swept >= -1.0;
        };

        // The FF757's own LNAV cannot fly RF legs (it chains straight segments),
        // so an RF leg must never be handed back.  Before the arc entry the
        // plugin flies the inbound course to the entry fix itself, and takes
        // over the circle once the aircraft is actually on it.
        const bool rfNow = leg && rfEngaged(leg);
        int rfEntryIdx = -1;
        int rfLeadStartIdx = -1;
        bool rfLeadIn = false;
        if (leg && leg->rfArc && cfg.lnavRfEnabled && !rfNow && leg->rfTurn != 0 &&
            leg->rfEntryValid) {
            // The CDU LEGS cache starts at the active waypoint, so an RF leg
            // may be active with its entry fix absent from the route: the entry
            // coordinates come from the CIFP row in that case.
            const int idx = leg->rfEntryWpt;
            const double dir = static_cast<double>(leg->rfTurn);
            const double swept = normalize180(
                dir * (bearingTrue(leg->rfCenterLat, leg->rfCenterLon,
                                   state.lat, state.lon) -
                       leg->rfRadialStartDeg));
            // Lead in only while still short of the entry radial; past it the
            // entry fix is behind the aircraft and steering there would turn
            // it around.
            if (swept < -0.5) {
                rfLeadIn = true;
                rfEntryIdx = idx;
                const RouteWaypoint* prevW = route.waypoint(idx - 1);
                if (idx >= 0 && prevW && prevW->posValid) rfLeadStartIdx = idx - 1;
            }
        }

        if (!leg) {
            useRoute = false;
            out.note = "active leg missing";
        } else if (rfNow) {
            // RF constant-radius-to-fix arc (geometry resolved from CIFP).
            // The reference path is the circle itself: steer the tangent at
            // the current radial plus a small correction that pulls the
            // aircraft back onto the circle, instead of the FF757's polygonal
            // approximation of the same leg.
            const RouteWaypoint* exitW = route.waypoint(leg->lastWpt);
            if (!exitW || !exitW->posValid) {
                useRoute = false;
                complexLeg = true;
                out.note = "RF arc exit waypoint missing; passing through";
            } else {
                const double dir = static_cast<double>(leg->rfTurn);
                const double total = leg->rfTotalSweptDeg;
                const double radialNow =
                    bearingTrue(leg->rfCenterLat, leg->rfCenterLon,
                                state.lat, state.lon);

                // Progress is unwrapped incrementally so arcs beyond 180 deg
                // cannot wrap; a leg change or an implausible radial jump
                // (route rescan) re-anchors the unwrap at the entry radial.
                const double step =
                    normalize180(dir * (radialNow - rfLastRadialDeg_));
                if (route.activeLeg != rfLegIdx_ || std::abs(step) > 10.0) {
                    rfLegIdx_ = route.activeLeg;
                    rfProgressDeg_ = clampd(
                        normalize180(dir * (radialNow - leg->rfRadialStartDeg)),
                        0.0, total);
                } else {
                    rfProgressDeg_ =
                        clampd(rfProgressDeg_ + step, 0.0, total);
                }
                rfLastRadialDeg_ = radialNow;

                const double remaining = total - rfProgressDeg_;
                const double rolloutLead =
                    std::min(cfg.lnavRolloutLeadDeg, total * 0.25);

                // Positive = outside the circle.
                const double radialErr =
                    distanceNm(leg->rfCenterLat, leg->rfCenterLon,
                               state.lat, state.lon) -
                    leg->rfRadiusNm;

                double desiredTrue = 0.0;
                if (remaining <= rolloutLead) {
                    // Roll-out: freeze on the outbound tangent so the AP
                    // levels the wings on the next course instead of
                    // overshooting through it.
                    desiredTrue =
                        normalize360(leg->rfRadialEndDeg + dir * 90.0);
                    turnProgress = 1.0;
                } else {
                    const double corr = clampd(
                        rad2deg(std::atan2(radialErr, 1.0)), -30.0, 30.0);
                    desiredTrue =
                        normalize360(radialNow + dir * (90.0 + corr));
                    turnProgress = clampd(
                        rfProgressDeg_ /
                            std::max(1e-6, total - rolloutLead),
                        0.0, 1.0);
                }

                // Distance to go along the arc, not the chord.
                routeDist = leg->rfRadiusNm * deg2rad(std::max(0.0, remaining));
                routeXtkRight = -dir * radialErr;  // route convention
                routeCourseMag = magneticFromTrue(
                    normalize360(leg->rfRadialStartDeg + dir * 90.0),
                    state.magVarDeg);

                updateRnp(cfg, std::abs(radialErr),
                          DistanceToDestination(route, leg->lastWpt, routeDist),
                          out);

                double desiredMag =
                    magneticFromTrue(desiredTrue, state.magVarDeg);
                desiredMag =
                    clampCourseError(desiredMag, currentTrackMag, maxError);

                out.active = true;
                out.routeBased = true;
                out.cdiFallback = false;
                out.activeWpt = route.activeWpt;
                out.activeLeg = route.activeLeg;
                out.nextLeg = route.nextLeg;
                out.xtkNm = routeXtkRight;
                out.distanceNm = routeDist;
                out.baseCourseMagDeg = routeCourseMag;
                out.desiredCourseMagDeg = desiredMag;
                out.correctionDeg = normalize180(desiredMag - routeCourseMag);
                out.rollCorrectionDeg =
                    clampd(cfg.lnavRollGain * out.correctionDeg,
                           -cfg.lnavMaxRollCorrectionDeg,
                           cfg.lnavMaxRollCorrectionDeg);
                out.turnProgress = turnProgress;
                out.source = "RF ARC";

                // Bank the published radius actually needs at this
                // groundspeed; above the bank limit the arc cannot be held.
                const double v = state.gsKts * kMpsPerKt;
                const double reqBank = rad2deg(std::atan2(
                    v * v, 9.80665 * leg->rfRadiusNm * kMetersPerNm));
                char note[180];
                std::snprintf(
                    note, sizeof(note),
                    "RF arc R=%.2f nm %s, %.0f deg to go, needs %.0f deg bank%s",
                    leg->rfRadiusNm, leg->rfTurn > 0 ? "R" : "L", remaining,
                    reqBank,
                    reqBank > cfg.lnavMaxBankDeg ? " (exceeds bank limit)" : "");
                out.note = note;
            }
        } else if (leg->complex && !rfLeadIn) {
            // Arcs / holds / procedure turns need the full leg geometry.  An
            // RF leg is handled above (arc, or plugin-owned lead-in), so only
            // a genuinely uncapturable arc falls through to the FF757 here.
            useRoute = false;
            complexLeg = true;
            out.note = leg->rfArc
                           ? "RF arc not captured yet; passing through"
                           : "active leg is a multi-point leg; passing through";
        } else {
            // RF lead-in tracks the inbound course to the arc entry fix; a
            // normal leg uses its own endpoints.
            int endIdx = rfLeadIn ? rfEntryIdx : leg->lastWpt;
            int startIdx = rfLeadIn ? (rfLeadStartIdx >= 0 ? rfLeadStartIdx : rfEntryIdx)
                                    : leg->firstWpt;

            // A one-waypoint leg's course is defined by the previous fix.
            if (leg->firstWpt == leg->lastWpt && !rfLeadIn) {
                startIdx = findPreviousLegEnd(route, route.activeLeg);
            }

            // An RF lead-in targets the arc entry fix even when that fix is not
            // in the route snapshot (CDU LEGS starts at the active waypoint).
            RouteWaypoint entryFallback;
            const RouteWaypoint* end = nullptr;
            if (rfLeadIn && rfEntryIdx < 0) {
                entryFallback.id = leg->rfEntryId;
                entryFallback.lat = leg->rfEntryLat;
                entryFallback.lon = leg->rfEntryLon;
                entryFallback.posValid = true;
                end = &entryFallback;
            } else {
                end = route.waypoint(endIdx);
            }
            if (!end || !end->posValid) {
                useRoute = false;
                out.note = "active waypoint missing";
            } else {
                const double dist = distanceNm(state.lat, state.lon, end->lat, end->lon);

                double startLat = state.lat;
                double startLon = state.lon;
                bool syntheticInbound = false;
                const RouteWaypoint* start = route.waypoint(startIdx);
                if (start && startIdx != endIdx && start->posValid) {
                    startLat = start->lat;
                    startLon = start->lon;
                } else if (rfLeadIn) {
                    // No start fix available: build the inbound line from the
                    // arc's own entry tangent (radial +/- 90 deg).  This keeps
                    // the lead-in independent of the FF757 route.
                    const double dir = static_cast<double>(leg->rfTurn);
                    const double tangent =
                        normalize360(leg->rfRadialStartDeg + dir * 90.0);
                    const double behind = std::max(dist + 10.0, 20.0);
                    destinationPoint(end->lat, end->lon,
                                     normalize360(tangent + 180.0), behind,
                                     startLat, startLon);
                } else if (cdi.valid && cdi.distanceNm >= 0.1) {
                    // The CDU LEGS cache starts at the active waypoint, so the
                    // inbound fix is missing.  Reconstruct the inbound course
                    // line from the FF757 desired track through the active
                    // waypoint, then compute cross-track from the route
                    // geometry instead of falling back to CDI dots.
                    const double courseTrue =
                        trueFromMagnetic(cdi.courseMagDeg, state.magVarDeg);
                    const double behind = std::max(dist + 10.0, 20.0);
                    destinationPoint(end->lat, end->lon,
                                     normalize360(courseTrue + 180.0), behind,
                                     startLat, startLon);
                    syntheticInbound = true;
                }

                double courseTrue = bearingTrue(startLat, startLon, end->lat, end->lon);
                double courseMag = magneticFromTrue(courseTrue, state.magVarDeg);

                double xtk = crossTrackNm(state.lat, state.lon,
                                          startLat, startLon,
                                          end->lat, end->lon);
                bool courseFromCdi = false;

                // The FF757's route computation is unreliable on RNP segments,
                // so an RF lead-in is driven purely by CIFP geometry: no course
                // anchoring and no CDI validation.
                if (cdi.valid && !rfLeadIn) {
                    const double dCourse = normalize180(courseMag - cdi.courseMagDeg);
                    const double absCourse = std::abs(dCourse);
                    const double tolerance = std::max(10.0, cdi.distanceNm * 0.5);
                    const bool distAgrees = std::abs(dist - cdi.distanceNm) <= tolerance;

                    if (absCourse > 25.0) {
                        // Only a sustained disagreement falls back to CDI; a
                        // single frame (rescan, sequencing, lagging FMS course)
                        // must not take the route away.
                        if (++cdiCourseRejectCount_ >= 40) {
                            useRoute = false;
                            out.note = "route course disagrees with CDI (sustained)";
                        } else {
                            out.note = "route course differs from CDI; holding route";
                        }
                    } else {
                        cdiCourseRejectCount_ = 0;
                        if (!distAgrees) {
                            if (++cdiDistRejectCount_ >= 40) {
                                useRoute = false;
                                out.note = "route distance disagrees with CDI (sustained)";
                            } else {
                                out.note = "route distance differs from CDI; holding route";
                            }
                        } else {
                            cdiDistRejectCount_ = 0;
                            // Anchor the course line on the FMS course only for
                            // small differences.  A large one means the FMS is
                            // on another leg or procedure, and its value must
                            // not override our own geometry.
                            if (absCourse > 0.15 && absCourse <= 8.0 && !syntheticInbound) {
                                // One degree of course difference over a 60 NM
                                // leg is a ~1 NM phantom cross-track, which
                                // used to command a 20 deg intercept at 400 kt.
                                courseMag = cdi.courseMagDeg;
                                courseTrue = trueFromMagnetic(courseMag, state.magVarDeg);
                                const double behind = std::max(dist + 10.0, 20.0);
                                destinationPoint(end->lat, end->lon,
                                                 normalize360(courseTrue + 180.0),
                                                 behind, startLat, startLon);
                                xtk = crossTrackNm(state.lat, state.lon,
                                                   startLat, startLon,
                                                   end->lat, end->lon);
                                courseFromCdi = true;
                            }
                        }
                    }
                }

                if (useRoute) {
                    routeXtkRight = xtk;
                    routeDist = dist;
                    routeCourseMag = courseMag;

                    // Calibrate the CDI dot sign while the route is trusted.
                    if (cdi.valid && !cdi.signIsCorrection &&
                        std::abs(xtk) > 0.05 && std::abs(cdi.xtkNm) > 0.05) {
                        double routeCorrection = -xtk;   // positive = turn right
                        double cdiCorrection = cdiSign_ * cdi.xtkNm;
                        if (routeCorrection * cdiCorrection < 0.0) {
                            if (++cdiDisagreeCount_ >= 5) {
                                cdiSign_ = -cdiSign_;
                                cdiDisagreeCount_ = 0;
                            }
                        } else {
                            cdiDisagreeCount_ = 0;
                        }
                    }

                    // --- RNP / ANP monitoring (Boeing 777 FMC logic) ---
                    // Shared with the RF arc branch: phase RNP selection, ANP
                    // low-pass and the hysteresis NAV UNABLE RNP alert.
                    updateRnp(cfg, std::abs(xtk),
                              DistanceToDestination(route, endIdx, dist), out);

                    const RouteLeg* next = route.leg(route.nextLeg);
                    double nextCourseTrue = courseTrue;
                    bool haveNext = false;
                    const RouteWaypoint* nextEnd = nullptr;

                    if (next && !next->complex) {
                        nextEnd = route.waypoint(next->lastWpt);
                        if (nextEnd && nextEnd->posValid && next->lastWpt != endIdx) {
                            nextCourseTrue = bearingTrue(end->lat, end->lon,
                                                         nextEnd->lat, nextEnd->lon);
                            haveNext = true;
                        }
                    }

                    // A fix is behind the aircraft when its bearing has flipped
                    // past 90 deg from the inbound course.  Direction-based, so
                    // it also works for a synthetic inbound leg.
                    const bool pastFix =
                        std::abs(normalize180(
                            bearingTrue(state.lat, state.lon, end->lat, end->lon) -
                            courseTrue)) > 90.0;

                    double desiredTrue = courseTrue;

                    // Boeing course capture: well off the track, command a
                    // constant intercept (default 30 deg) back to the course
                    // line; close to the track, home direct to the fix
                    // (allowed up to 45 deg).  The hand-over is smooth and the
                    // homing angle decays as the offset closes, so the capture
                    // never snaps and never commands a long-range S-turn.
                    auto legIntercept = [&](double legXtk, double legDist) {
                        const double ax = std::abs(legXtk);
                        // Direct-to-fix geometry: when the offset is small the
                        // course that reaches the fix differs from the leg
                        // course by atan(xtk / distance-to-fix), so the
                        // correction decays as the aircraft closes on the fix
                        // instead of holding a fixed angle.  A short distance
                        // floor only guards the divide near the fix.
                        const double homing = rad2deg(
                            std::atan2(ax, std::max(legDist, 0.3)));
                        const double nearAng =
                            std::min(homing, cfg.lnavDirectInterceptDeg);
                        const double farAng = cfg.lnavMaxInterceptDeg;
                        // Hand over to the constant capture angle only when the
                        // aircraft really is off the course line.  The old
                        // blend (ax / 2.5) already commanded ~21 deg for a
                        // 1.2 NM offset 50 NM from the fix, which at 400 kt
                        // swung the aircraft through a needless S-turn.
                        const double blend = std::max(0.1, cfg.lnavXtkBlendNm);
                        const double t =
                            smoothstep01((ax - 0.8 * blend) / (0.4 * blend));
                        const double ang = nearAng + (farAng - nearAng) * t;
                        return clampd(ang, 0.0, 60.0) *
                               (legXtk >= 0.0 ? 1.0 : -1.0);
                    };
                    double interceptDeg = legIntercept(xtk, dist);

                    // No fly-by anticipation during an RF lead-in: the arc is
                    // tangent to the inbound course, so the turn starts exactly
                    // at the entry and the arc itself handles it.
                    if (haveNext && !rfLeadIn) {
                        const double delta = normalize180(nextCourseTrue - courseTrue);
                        if (std::abs(delta) > 3.0 && std::abs(delta) < 120.0) {
                            const double absDelta = std::abs(delta);
                            const double sign = delta > 0.0 ? 1.0 : -1.0;

                            // Fly-by geometry as used by the 757/767 FMC family:
                            // a steady arc of radius R (groundspeed at the bank
                            // limit) is tangent to both legs.  Its tangent points
                            // sit R*tan(delta/2) before the fix on each leg, and
                            // the centre is R inside the inbound course measured
                            // from that tangent point.
                            //
                            // The centre used to be placed R inside the *fix*
                            // instead.  That circle is tangent to the inbound leg
                            // at the fix, so the aircraft reached the end of the
                            // arc while still `tangent` miles short, LNAV rolled
                            // out immediately and the corner was cut.
                            const double radius =
                                turnRadiusNm(state.gsKts, cfg.lnavMaxBankDeg);
                            const double tangent =
                                clampd(radius * std::tan(deg2rad(absDelta * 0.5)) *
                                           cfg.lnavTurnLeadFactor,
                                       0.3, 20.0);

                            double tLat = 0.0, tLon = 0.0, cLat = 0.0, cLon = 0.0;
                            destinationPoint(end->lat, end->lon,
                                             normalize360(courseTrue + 180.0),
                                             tangent, tLat, tLon);
                            destinationPoint(tLat, tLon,
                                             normalize360(courseTrue + sign * 90.0),
                                             radius, cLat, cLon);

                            // Arc progress is the bearing seen from the centre.
                            // Chord/straight distance is not a position measure
                            // once the aircraft is off the circle: with a lateral
                            // offset it reads as "half the turn already done" at
                            // the tangent point.
                            const double radialStart =
                                normalize360(courseTrue - sign * 90.0);
                            const double radialNow =
                                bearingTrue(cLat, cLon, state.lat, state.lon);

                            // Remaining distance along the leg (not the straight
                            // line to the fix) so a lateral offset cannot start
                            // the turn early.
                            const double legLength =
                                distanceNm(startLat, startLon, end->lat, end->lon);
                            const double remainAlong =
                                legLength - alongTrackNm(state.lat, state.lon,
                                                         startLat, startLon,
                                                         end->lat, end->lon);
                            const bool onArcZone =
                                pastFix || remainAlong <= tangent + 0.1;

                            if (onArcZone) {
                                const double arcAngle = clampd(
                                    normalize180(sign * (radialNow - radialStart)),
                                    0.0, absDelta);

                                // Roll-out lead: freeze the target on the
                                // outbound course a few degrees before the arc
                                // ends.  The autopilot needs time to level the
                                // wings; without the lead its own capture lag
                                // carried the turn past the new course.
                                const double rolloutLead =
                                    std::min(cfg.lnavRolloutLeadDeg,
                                             absDelta * 0.25);
                                turnProgress = clampd(
                                    arcAngle /
                                        std::max(1e-6, absDelta - rolloutLead),
                                    0.0, 1.0);

                                if (arcAngle >= absDelta) {
                                    // Roll-out: track the outbound leg with its
                                    // own cross-track and distance to go.
                                    desiredTrue = nextCourseTrue;
                                    const double xtkNext = crossTrackNm(
                                        state.lat, state.lon,
                                        end->lat, end->lon,
                                        nextEnd->lat, nextEnd->lon);
                                    const double distNext = distanceNm(
                                        state.lat, state.lon,
                                        nextEnd->lat, nextEnd->lon);
                                    interceptDeg = legIntercept(xtkNext, distNext);
                                } else if (arcAngle >= absDelta - rolloutLead) {
                                    // Frozen on the outbound course while the
                                    // AP levels the wings; no intercept yet.
                                    desiredTrue = nextCourseTrue;
                                    interceptDeg = 0.0;
                                } else if (arcAngle > 0.0) {
                                    // On the arc the path itself is the
                                    // reference: steer the tangent course and
                                    // add no cross-track intercept, which used
                                    // to double the turn command.
                                    desiredTrue = courseTrue + sign * arcAngle;
                                    interceptDeg = 0.0;
                                } else {
                                    turnProgress = 0.0;
                                }
                            } else {
                                turnProgress = 0.0;
                            }
                        }
                    }

                    // Positive xtk means the aircraft is right of course, so the
                    // intercept correction is to the left.
                    desiredTrue -= interceptDeg;

                    double desiredMag = magneticFromTrue(desiredTrue, state.magVarDeg);
                    desiredMag = clampCourseError(desiredMag, currentTrackMag, maxError);

                    out.active = true;
                    out.routeBased = true;
                    out.cdiFallback = false;
                    out.activeWpt = route.activeWpt;
                    out.activeLeg = route.activeLeg;
                    out.nextLeg = route.nextLeg;
                    out.xtkNm = routeXtkRight;
                    out.distanceNm = routeDist;
                    out.baseCourseMagDeg = routeCourseMag;
                    out.desiredCourseMagDeg = desiredMag;
                    out.correctionDeg = normalize180(desiredMag - routeCourseMag);
                    out.rollCorrectionDeg =
                        clampd(cfg.lnavRollGain * out.correctionDeg,
                               -cfg.lnavMaxRollCorrectionDeg,
                               cfg.lnavMaxRollCorrectionDeg);
                    out.turnProgress = turnProgress;
                    out.source = syntheticInbound ? "ROUTE/SYNTH" : "ROUTE";
                    if (out.note.empty()) {
                        out.note = syntheticInbound
                                       ? "route geometry with synthetic inbound course"
                                       : "FF757 route + turn anticipation";
                    }
                    if (rfLeadIn) {
                        out.note = "RF arc lead-in to ";
                        out.note += leg->rfEntryId.empty() ? "entry fix"
                                                           : leg->rfEntryId;
                    }
                    if (courseFromCdi) out.note += " (course from FMS)";
                }
            }
        }
    }

    if (out.active && out.routeBased) {
        // Refresh the hold: a route command stays valid for ~1 s (20 frames),
        // so a CDU rescan or a sequencing gap does not drop LNAV to CDI dots.
        holdValid_ = true;
        holdFrames_ = 20;
        holdCourseMagDeg_ = out.desiredCourseMagDeg;
        holdXtkNm_ = out.xtkNm;
        holdDistNm_ = out.distanceNm;
    }

    if (!useRoute && !complexLeg && holdValid_ && holdFrames_ > 0) {
        // Route snapshot momentarily unusable: keep steering the last known
        // route instead of falling back to GPS CDI steering.
        out.active = true;
        out.routeBased = true;
        out.cdiFallback = false;
        out.xtkNm = holdXtkNm_;
        out.distanceNm = holdDistNm_;
        out.baseCourseMagDeg = holdCourseMagDeg_;
        const double desired =
            clampCourseError(holdCourseMagDeg_, currentTrackMag, maxError);
        out.desiredCourseMagDeg = desired;
        out.correctionDeg = normalize180(desired - holdCourseMagDeg_);
        out.rollCorrectionDeg = clampd(cfg.lnavRollGain * out.correctionDeg,
                                       -cfg.lnavMaxRollCorrectionDeg,
                                       cfg.lnavMaxRollCorrectionDeg);
        out.source = "ROUTE HOLD";
        out.note = "route temporarily unavailable; holding last route guidance";
        --holdFrames_;
        if (holdFrames_ <= 0) holdValid_ = false;
        return out;
    }

    if (!useRoute && !complexLeg && cdi.valid && cdi.distanceNm >= cfg.lnavMinDistanceNm) {
        // Calibrate the CDI dot sign against the bank the FF757 is actually
        // commanding.  This keeps the plugin aligned with the aircraft's own
        // LNAV sense even when the private route data is unavailable.
        if (!cdi.signIsCorrection && std::abs(state.fdRollDeg) > 8.0 &&
            std::abs(cdi.xtkNm) > 0.1) {
            const double cdiCorrection = cdiSign_ * cdi.xtkNm;
            if (cdiCorrection * state.fdRollDeg < 0.0) {
                if (++fdRollDisagreeCount_ >= 10) {
                    cdiSign_ = -cdiSign_;
                    fdRollDisagreeCount_ = 0;
                }
            } else {
                fdRollDisagreeCount_ = 0;
            }
        }

        const double dist = std::max(cdi.distanceNm, 0.2);
        const double correctionNm = cdi.signIsCorrection
                                        ? cdi.xtkNm
                                        : cdiSign_ * cdi.xtkNm;  // positive = turn right
        const double intercept =
            rad2deg(std::atan2(cfg.lnavGain * correctionNm, dist));
        const double clampedIntercept =
            clampd(intercept, -cfg.lnavMaxInterceptDeg, cfg.lnavMaxInterceptDeg);

        double desiredMag = normalize360(cdi.courseMagDeg + clampedIntercept);
        desiredMag = clampCourseError(desiredMag, currentTrackMag, maxError);

        out.active = true;
        out.routeBased = false;
        out.cdiFallback = true;
        out.activeWpt = route.activeWpt;
        out.activeLeg = route.activeLeg;
        out.nextLeg = route.nextLeg;
        out.xtkNm = -correctionNm;   // report in the route convention
        out.distanceNm = cdi.distanceNm;
        out.baseCourseMagDeg = cdi.courseMagDeg;
        out.desiredCourseMagDeg = desiredMag;
        out.correctionDeg = normalize180(desiredMag - cdi.courseMagDeg);
        out.rollCorrectionDeg =
            clampd(cfg.lnavRollGain * out.correctionDeg,
                   -cfg.lnavMaxRollCorrectionDeg,
                   cfg.lnavMaxRollCorrectionDeg);
        out.turnProgress = 0.0;
        out.source = "GPS CDI";
        if (out.note.empty()) out.note = "route fallback: GPS CDI";
        return out;
    }

    if (!out.active && out.note.empty()) {
        out.note = "no usable FMS route or CDI guidance";
    }
    return out;
}

VnavResult Guidance::computeVnav(const AircraftState& state,
                                 const RouteSnapshot& route,
                                 const Config& cfg) {
    VnavResult out;

    if (!state.valid) {
        out.note = "aircraft state invalid";
        return out;
    }
    if (state.onGround) {
        out.note = "on ground";
        return out;
    }
    if (!state.apEngaged) {
        out.note = "autopilot not engaged";
        return out;
    }
    if (!state.vnavMode && !cfg.forceVnav) {
        out.note = "VNAV mode not detected";
        return out;
    }

    // In VNAV SPD the FF757 uses pitch to hold the FMC target speed.  Adding
    // a vertical-path correction here directly fights the speed loop, which
    // shows up as "VNAV SPD but the speed runs away".  Pass through untouched
    // unless the user explicitly opts in.
    if (state.vnavSpeedMode && !cfg.vnavAugmentSpeedMode) {
        out.active = true;       // report the mode, but do not write pitch
        out.pathBased = false;
        out.passThrough = true;
        out.source = "VNAV SPD PASS";
        out.note = "VNAV SPD holds speed with pitch; path correction suppressed";
        return out;
    }

    bool usedAltitude = false;
    const VnavTarget target = FindVnavTarget(route, state, cfg);
    const SpeedTarget spd = FindSpeedTarget(route, state, cfg);
    if (spd.valid) {
        out.targetSpdKt = static_cast<double>(spd.speedKt);
        out.targetSpdDistNm = spd.distanceNm;
        out.targetSpdLeadNm = spd.decelLeadNm;
    }

    // Deceleration lead: when the speed constraint's required braking
    // distance exceeds the natural descent distance for an ahead target,
    // soften the descent path by commanding a shallower profile so we reach
    // the fix at the right speed instead of having to brake hard on the
    // short final.  The same 1 NM / 10 KT factor is used on the pitch side
    // (treating extra deceleration distance as level-out distance).
    double speedHoldLead = 0.0;
    if (spd.valid && spd.decelLeadNm > 0.0 && target.valid && spd.index == target.index) {
        // Aligned: the altitude target IS the DECEL fix.  Use the decel lead
        // to pull out of the descent earlier, at a gentler VS.
        speedHoldLead = spd.decelLeadNm;
    } else if (spd.valid && spd.distanceNm < 1.5) {
        // Already inside the decel fix itself: don't keep descending hard.
        speedHoldLead = std::max(speedHoldLead, 0.5);
    }
    if (target.valid) {
        // The MCP altitude is a ceiling in a climb and a floor in a descent.
        // Without this clamp a route constraint below the selected altitude
        // makes the plugin command a descent even though the FF757 is level
        // at the MCP altitude (for example 34100 ft).
        double profileAlt = static_cast<double>(target.altFt);
        const double selectedAlt =
            (state.selectedAltFt > 1000.0 && state.selectedAltFt < 60000.0)
                ? state.selectedAltFt
                : 0.0;
        bool mcpLimited = false;
        if (selectedAlt > 0.0 && cfg.vnavRespectSelectedAlt) {
            if (profileAlt > state.altFt) {
                profileAlt = std::min(profileAlt, selectedAlt);
            } else {
                profileAlt = std::max(profileAlt, selectedAlt);
            }
            mcpLimited = std::abs(profileAlt - static_cast<double>(target.altFt)) > 1.0;
        }

        const double deltaAlt = profileAlt - state.altFt;
        const double gs = std::max(state.gsKts, 60.0);
        const double timeHr = target.distanceNm / gs;
        const double rawRequired = deltaAlt / std::max(timeHr * 60.0, 1.0e-3);

        // Guard against a mis-parsed three-digit value ("350" meaning either
        // 350 ft or FL350) while at cruise altitude.
        const bool suspiciousLowAlt = (profileAlt < 1000 && state.altFt > 5000.0);
        const double maxPlausibleDelta =
            std::min(cfg.vnavMaxAltDeltaFt,
                     std::max(8000.0, target.distanceNm * 1200.0));
        if (suspiciousLowAlt) {
            out.note = "low altitude target ignored at high altitude";
        } else if (std::abs(deltaAlt) > maxPlausibleDelta) {
            out.note = "altitude target is implausible for the distance";
        } else if (std::abs(deltaAlt) < 100.0) {
            // The effective target has been clamped to the MCP altitude, or
            // the route constraint is effectively the current altitude.
            out.active = true;
            out.pathBased = false;
            out.passThrough = true;   // the FF757 owns this level-off
            out.targetAltFt = state.altFt;
            out.requiredVsFpm = 0.0;
            out.vsErrorFpm = 0.0;
            out.correctionDeg = 0.0;
            out.source = mcpLimited ? "MCP HOLD" : "ALT HOLD";
            char note[180];
            std::snprintf(note, sizeof(note), "%s: holding %.0f ft",
                          mcpLimited ? "MCP altitude" : "route altitude",
                          state.altFt);
            out.note = note;
            usedAltitude = true;
        } else {
            bool beforeTod = false;
            if (deltaAlt < -100.0) {
                // A nominal 3-degree path loses about 318 ft/NM.  If the
                // target is farther away than that, the aircraft is still
                // before TOD and should hold altitude rather than start a
                // shallow, early descent.
                const double descentDistance = std::abs(deltaAlt) / 318.0;
                beforeTod = target.distanceNm > descentDistance + 2.0;
            }

            if (beforeTod) {
                out.active = true;
                out.pathBased = false;
                out.passThrough = true;   // level until the FF757 starts the descent
                out.targetAltFt = state.altFt;
                out.requiredVsFpm = 0.0;
                out.vsErrorFpm = 0.0;
                out.correctionDeg = 0.0;
                out.source = "ROUTE HOLD";
                char note[220];
                std::snprintf(note, sizeof(note),
                              "before TOD: %s %d ft at %.1f NM (3 deg needs %.1f NM)%s%s",
                              target.id.empty() ? "WPT" : target.id.c_str(),
                              target.altFt, target.distanceNm,
                              std::abs(deltaAlt) / 318.0,
                              spd.valid ? " | decel lead " : "",
                              spd.valid ? std::to_string(spd.decelLeadNm).c_str() : "");
                out.note = note;
                usedAltitude = true;
            } else {
                // The nominal required VS for the altitude target.  Soften it
                // when a deceleration lead overlaps the descent so we descend
                // shallower than 3 deg.  This is the pitch-side expression of
                // "need braking distance X NM - plan the descent so you cover
                // that distance without hard brakes".
                double finalRequired = rawRequired;
                if (speedHoldLead > 0.0) {
                    // Treat the decel lead as extra level distance inside the
                    // descent, reducing the fpm requirement linearly.
                    const double softened =
                        rawRequired * std::max(0.3,
                                               target.distanceNm /
                                                   (target.distanceNm + speedHoldLead));
                    finalRequired = clampd(softened, -cfg.vnavMaxVsFpm, cfg.vnavMaxVsFpm);
                }
                out.targetAltFt = profileAlt;
                out.requiredVsFpm = finalRequired;

                // Never initiate a vertical path the FF757 is not already
                // flying.  If the aircraft is level while the route target
                // demands a climb/descent, or if the FF757 is moving in the
                // opposite direction, its own VNAV/ALT mode owns the
                // vertical axis.  This prevents a route constraint from
                // starting a descent against an MCP level-off and from
                // fighting a VNAV SPD climb.
                const bool levelAircraft = std::abs(state.vsFpm) < 200.0;
                const bool oppositeDirection =
                    out.requiredVsFpm * state.vsFpm < 0.0;
                const bool letFF757OwnVertical =
                    (levelAircraft && std::abs(out.requiredVsFpm) > 500.0) ||
                    (oppositeDirection && std::abs(out.requiredVsFpm) > 800.0);

                if (letFF757OwnVertical) {
                    out.active = true;
                    out.pathBased = false;
                    out.passThrough = true;   // FF757 owns the vertical axis
                    out.targetAltFt = state.altFt;
                    out.requiredVsFpm = 0.0;
                    out.vsErrorFpm = 0.0;
                    out.correctionDeg = 0.0;
                    out.source = "FF757 OWNS VS";
                    out.note = levelAircraft
                                   ? "aircraft level; vertical path not initiated"
                                   : "FF757 vertical direction differs; path correction suppressed";
                } else {
                    out.vsErrorFpm = out.requiredVsFpm - state.vsFpm;

                    if (std::abs(out.vsErrorFpm) < 150.0 && speedHoldLead <= 0.0) {
                        // Small residual error: leave the FF757's pitch loop
                        // alone instead of injecting a micro-correction that
                        // can turn into a porpoising feedback loop.  But
                        // DO NOT apply this deadband while we are softening
                        // the descent for a DECEL lead - the whole point of
                        // the softening is to intentionally aim for a shallower
                        // VS than the aircraft is currently flying, so a small
                        // difference is not "residual" - it is the plan.
                        out.active = true;
                        out.pathBased = false;
                        out.passThrough = true;
                        out.targetAltFt = state.altFt;
                        out.requiredVsFpm = 0.0;
                        out.vsErrorFpm = 0.0;
                        out.correctionDeg = 0.0;
                        out.source = "FD PASS";
                        out.note = "vertical path within deadband; FF757 owns pitch";
                    } else {
                        double correction = cfg.vnavVsGain * out.vsErrorFpm / 1000.0;
                        correction -= cfg.vnavAccelGain *
                                      clampd(state.vsAccelFpmPerSec, -1000.0, 1000.0);
                        out.correctionDeg = clampd(correction,
                                                   -cfg.vnavMaxCorrectionDeg,
                                                   cfg.vnavMaxCorrectionDeg);
                        out.active = true;
                        out.pathBased = true;
                        out.source = mcpLimited ? "ROUTE/MCP" : "ROUTE PROFILE";
                        // Built with plain snprintf: computeVnav runs on every
                        // flight-loop frame, so string concatenation here
                        // would allocate continuously.
                        char note[260];
                        if (spd.valid && speedHoldLead > 0.0) {
                            std::snprintf(note, sizeof(note),
                                          "VNAV profile to %s, %.1f NM, %.0f ft%s"
                                          " | speed %d kt @ %.0f NM, decel lead %.1f NM%s",
                                          target.id.empty() ? "WPT" : target.id.c_str(),
                                          target.distanceNm, profileAlt,
                                          mcpLimited ? " (MCP limited)" : "",
                                          spd.speedKt, spd.distanceNm,
                                          spd.decelLeadNm,
                                          spd.auto250 ? " (auto 250/10000)" : "");
                        } else if (spd.valid) {
                            std::snprintf(note, sizeof(note),
                                          "VNAV profile to %s, %.1f NM, %.0f ft%s"
                                          " | speed %d kt @ %.0f NM%s",
                                          target.id.empty() ? "WPT" : target.id.c_str(),
                                          target.distanceNm, profileAlt,
                                          mcpLimited ? " (MCP limited)" : "",
                                          spd.speedKt, spd.distanceNm,
                                          spd.auto250 ? " (auto 250/10000)" : "");
                        } else {
                            std::snprintf(note, sizeof(note),
                                          "VNAV profile to %s, %.1f NM, %.0f ft%s",
                                          target.id.empty() ? "WPT" : target.id.c_str(),
                                          target.distanceNm, profileAlt,
                                          mcpLimited ? " (MCP limited)" : "");
                        }
                        out.note = note;
                    }
                }
                usedAltitude = true;
            }
        }
    } else {
        out.note = "no usable altitude target ahead";
    }


    if (!usedAltitude && std::abs(route.vtk) > 0.01) {
        const double gs = std::max(state.gsKts, 60.0);
        const double vtkAbs = std::abs(route.vtk);
        double requiredVs = 0.0;
        std::string source = "VTK";
        if (vtkAbs < 20.0) {
            requiredVs = vsForPathDeg(route.vtk, gs);
            source = "VTK angle";
        } else if (vtkAbs <= 2000.0) {
            // Some builds expose a ft/nm gradient instead of an angle.
            requiredVs = route.vtk * gs / 60.0;
            source = "VTK gradient";
        } else {
            requiredVs = route.vtk;
            source = "VTK fpm";
        }

        out.targetAltFt = 0.0;
        out.requiredVsFpm = clampd(requiredVs, -cfg.vnavMaxVsFpm, cfg.vnavMaxVsFpm);
        out.vsErrorFpm = out.requiredVsFpm - state.vsFpm;

        const bool levelAircraft = std::abs(state.vsFpm) < 200.0;
        const bool oppositeDirection =
            out.requiredVsFpm * state.vsFpm < 0.0;
        const bool letFF757OwnVertical =
            (levelAircraft && std::abs(out.requiredVsFpm) > 500.0) ||
            (oppositeDirection && std::abs(out.requiredVsFpm) > 800.0) ||
            std::abs(out.vsErrorFpm) < 150.0;
        if (letFF757OwnVertical) {
            out.active = true;
            out.pathBased = false;
            out.passThrough = true;
            out.targetAltFt = state.altFt;
            out.requiredVsFpm = 0.0;
            out.vsErrorFpm = 0.0;
            out.correctionDeg = 0.0;
            out.source = "FF757 OWNS VS";
            out.note = "vtk path not initiated or within deadband";
            return out;
        }

        double correction = cfg.vnavVsGain * out.vsErrorFpm / 1000.0;
        correction -= cfg.vnavAccelGain * clampd(state.vsAccelFpmPerSec, -1000.0, 1000.0);
        out.correctionDeg = clampd(correction,
                                   -cfg.vnavMaxCorrectionDeg,
                                   cfg.vnavMaxCorrectionDeg);
        out.active = true;
        out.pathBased = true;
        out.source = source;
        out.note = "vertical path from FF757 vtk";
        return out;
    }

    // VNAV is engaged, but the route altitude/vtk data is unavailable.  Do not
    // write a smoothed pitch command: smoothing a mode that we cannot model is
    // the most likely source of a porpoising FD/AoA oscillation.  Pass through.
    if (!out.active) {
        out.active = true;
        out.pathBased = false;
        out.passThrough = true;
        out.source = "VNAV PASS";
        out.correctionDeg = 0.0;
        out.note = out.note.empty()
                       ? "VNAV mode; FF757 owns pitch"
                       : out.note + "; FF757 owns pitch";
    }
    return out;
}

} // namespace lvnv
