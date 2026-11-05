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

} // namespace

void Guidance::reset() {
    cdiSign_ = 1;
    cdiDisagreeCount_ = 0;
    fdRollDisagreeCount_ = 0;
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
        if (!leg) {
            useRoute = false;
            out.note = "active leg missing";
        } else if (leg->complex) {
            // Arcs / holds / procedure turns need the full leg geometry.  Do
            // not replace the FF757's procedure guidance with a chord or a
            // direct-to course.
            useRoute = false;
            complexLeg = true;
            out.note = "active leg is a multi-point leg; passing through";
        } else {
            int endIdx = leg->lastWpt;
            int startIdx = leg->firstWpt;

            // A one-waypoint leg's course is defined by the previous fix.
            if (leg->firstWpt == leg->lastWpt) {
                startIdx = findPreviousLegEnd(route, route.activeLeg);
            }

            const RouteWaypoint* end = route.waypoint(endIdx);
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

                const double courseTrue = bearingTrue(startLat, startLon, end->lat, end->lon);
                const double courseMag = magneticFromTrue(courseTrue, state.magVarDeg);

                const double xtk = crossTrackNm(state.lat, state.lon,
                                                startLat, startLon,
                                                end->lat, end->lon);

                if (cdi.valid) {
                    if (std::abs(normalize180(courseMag - cdi.courseMagDeg)) > 25.0) {
                        useRoute = false;
                        out.note = "route course disagrees with CDI";
                    } else {
                        double tolerance = std::max(10.0, cdi.distanceNm * 0.5);
                        if (std::abs(dist - cdi.distanceNm) > tolerance) {
                            useRoute = false;
                            out.note = "route distance disagrees with CDI";
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

                    // Intercept angle for one leg: the cross-track is measured
                    // against the remaining distance to that leg's own fix.  A
                    // very small denominator used to demand up to 25 deg of
                    // intercept right at the fix, which caused the last-second
                    // turn and the overshoot; the distance floor keeps that
                    // correction realistic.
                    auto legIntercept = [&](double legXtk, double legDist) {
                        const double d = std::max(legDist, cfg.lnavInterceptDistanceNm);
                        const double ang =
                            rad2deg(std::atan2(cfg.lnavGain * legXtk, d));
                        return clampd(ang, -cfg.lnavMaxInterceptDeg,
                                      cfg.lnavMaxInterceptDeg);
                    };
                    double interceptDeg = legIntercept(xtk, dist);

                    if (haveNext) {
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
                                turnProgress = arcAngle / absDelta;

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
                }
            }
        }
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
                char note[180];
                std::snprintf(note, sizeof(note),
                              "before TOD: %s %d ft at %.1f NM (3 deg needs %.1f NM)",
                              target.id.empty() ? "WPT" : target.id.c_str(),
                              target.altFt, target.distanceNm,
                              std::abs(deltaAlt) / 318.0);
                out.note = note;
                usedAltitude = true;
            } else {
                out.targetAltFt = profileAlt;
                out.requiredVsFpm = clampd(rawRequired, -cfg.vnavMaxVsFpm, cfg.vnavMaxVsFpm);

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

                    if (std::abs(out.vsErrorFpm) < 150.0) {
                        // Small residual error: leave the FF757's pitch loop
                        // alone instead of injecting a micro-correction that
                        // can turn into a porpoising feedback loop.
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
                        char note[180];
                        std::snprintf(note, sizeof(note),
                                      "VNAV profile to %s, %.1f NM, %.0f ft%s",
                                      target.id.empty() ? "WPT" : target.id.c_str(),
                                      target.distanceNm, profileAlt,
                                      mcpLimited ? " (MCP limited)" : "");
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
