#include "config.h"
#include "geo.h"
#include "guidance.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace lvnv;

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, msg) do { \
    ++g_checks; \
    if (!(cond)) { \
        ++g_failures; \
        std::printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); \
    } \
} while (0)

#define CHECK_NEAR(a, b, eps, msg) CHECK(std::abs((a) - (b)) <= (eps), msg)

static RouteSnapshot MakeRoute() {
    RouteSnapshot r;
    r.valid = true;
    r.sourceRoute = true;
    r.wptCount = 3;
    r.legCount = 2;
    r.activeWpt = 1;
    r.activeLeg = 0;
    r.nextLeg = 1;

    r.wpts.resize(3);
    r.wpts[0].lat = 0.0;
    r.wpts[0].lon = 0.0;
    r.wpts[0].id = "AAA";
    r.wpts[1].lat = 0.0;
    r.wpts[1].lon = 1.0;
    r.wpts[1].id = "BBB";
    r.wpts[2].lat = 0.5;
    r.wpts[2].lon = 1.0;
    r.wpts[2].id = "CCC";
    for (auto& w : r.wpts) w.posValid = true;

    r.legs.resize(2);
    r.legs[0] = RouteLeg{0, 0, 1, false};
    r.legs[1] = RouteLeg{0, 1, 2, false};
    return r;
}

static RouteSnapshot MakeCduRoute() {
    RouteSnapshot r;
    r.valid = true;
    r.sourceRoute = true;
    r.wptCount = 3;
    r.legCount = 3;
    r.activeWpt = 0;
    r.activeLeg = 0;
    r.nextLeg = 1;

    r.wpts.resize(3);
    r.wpts[0].lat = 0.0;
    r.wpts[0].lon = 0.0;
    r.wpts[0].id = "AAA";
    r.wpts[1].lat = 0.0;
    r.wpts[1].lon = 0.5;
    r.wpts[1].id = "BBB";
    r.wpts[2].lat = 0.0;
    r.wpts[2].lon = 1.0;
    r.wpts[2].id = "CCC";
    for (auto& w : r.wpts) w.posValid = true;

    r.legs.resize(3);
    r.legs[0] = RouteLeg{0, 0, 0, false};
    r.legs[1] = RouteLeg{0, 0, 1, false};
    r.legs[2] = RouteLeg{0, 1, 2, false};
    return r;
}

static AircraftState MakeState(double lat, double lon, double track) {
    AircraftState s;
    s.valid = true;
    s.onGround = false;
    s.apEngaged = true;
    s.lnavOverride = true;
    s.lnavModeDetected = true;
    s.vnavMode = true;
    s.lat = lat;
    s.lon = lon;
    s.trackTrueDeg = track;
    s.magVarDeg = 0.0;
    s.gsKts = 400.0;
    s.altFt = 10000.0;
    s.vsFpm = 0.0;
    s.vsAccelFpmPerSec = 0.0;
    s.iasKts = 280.0;
    return s;
}

int main() {
    // Basic geodesy.
    CHECK_NEAR(distanceNm(0, 0, 0, 1), 60.04, 0.1, "one degree longitude at equator");
    CHECK_NEAR(bearingTrue(0, 0, 0, 1), 90.0, 0.01, "eastbound bearing");

    double xtk = crossTrackNm(0.1, 0.5, 0.0, 0.0, 0.0, 1.0);
    CHECK(xtk < 0.0, "point north of eastbound course is left of course");
    CHECK_NEAR(std::abs(xtk), 6.0, 0.2, "cross-track magnitude");

    double radius = turnRadiusNm(400.0, 25.0);
    CHECK(radius > 3.0 && radius < 7.0, "turn radius in a sane range");

    Config cfg;
    cfg.lnavMaxCourseErrorDeg = 90.0;
    CHECK(cfg.vnavPitchModeA == 0 && cfg.vnavPitchModeB == 6 &&
          cfg.vnavPitchModeC == 5, "observed VNAV pitch modes are enabled");
    Guidance guidance;

    // Route-based LNAV: left of the 090 course, so the target must turn right.
    {
        RouteSnapshot r = MakeRoute();
        AircraftState s = MakeState(0.1, 0.5, 85.0);
        CdiInput cdi;   // invalid on purpose
        LnavResult ln = guidance.computeLnav(s, r, cdi, cfg);
        CHECK(ln.active, "route LNAV active");
        CHECK(ln.routeBased, "route LNAV uses route");
        CHECK(ln.desiredCourseMagDeg > 90.0, "left of course -> turn right");
        CHECK_NEAR(ln.baseCourseMagDeg, 90.0, 1.0, "route base course");
        CHECK_NEAR(ln.xtkNm, xtk, 0.1, "route xtk matches");
    }

    // Turn anticipation: close to BBB, the 090 course must start turning to 000.
    {
        RouteSnapshot r = MakeRoute();
        AircraftState s = MakeState(0.0, 0.95, 90.0);
        CdiInput cdi;
        LnavResult ln = guidance.computeLnav(s, r, cdi, cfg);
        CHECK(ln.active, "turn anticipation active");
        CHECK(ln.turnProgress > 0.0, "turn progress positive");
        CHECK(ln.desiredCourseMagDeg < 90.0, "next leg is left of current leg");
    }

    // The turn is tracked round a tangent arc, so the progress must never go
    // backwards as the aircraft advances.  The old distance-based ramp
    // un-turned the course on roll-out because the straight distance to a
    // fly-by fix shrinks and then grows again inside the turn.
    {
        RouteSnapshot r = MakeRoute();
        double previous = -1.0;
        for (double lon = 0.80; lon <= 1.0; lon += 0.01) {
            AircraftState s = MakeState(0.0, lon, 90.0);
            CdiInput cdi;
            LnavResult ln = guidance.computeLnav(s, r, cdi, cfg);
            if (previous >= 0.0) {
                CHECK(ln.turnProgress >= previous,
                      "turn progress never decreases through the arc");
            }
            previous = ln.turnProgress;
        }
        CHECK(previous > 0.0, "turn progress grows approaching the fix");
    }

    // The arc must only begin at the tangent point and start from zero
    // progress there.  With the centre placed at the fix instead of at the
    // tangent point, the chord to the tangent point already subtends the full
    // turn on entry, so LNAV rolled straight onto the outbound course and cut
    // the corner.
    {
        RouteSnapshot r = MakeRoute();
        const double radius = turnRadiusNm(400.0, 25.0);
        const double tangent = radius * std::tan(deg2rad(45.0));   // 090 -> 000
        const double tangentDeg = tangent / 60.04;                 // at the equator
        const double tangentLon = 1.0 - tangentDeg;
        CdiInput cdi;

        AircraftState s = MakeState(0.0, tangentLon - 0.02, 90.0);
        LnavResult ln = guidance.computeLnav(s, r, cdi, cfg);
        CHECK(ln.turnProgress == 0.0, "turn has not started before the tangent point");
        CHECK_NEAR(ln.desiredCourseMagDeg, 90.0, 0.5, "still tracking the inbound course");

        s = MakeState(0.0, tangentLon + 0.01, 90.0);
        ln = guidance.computeLnav(s, r, cdi, cfg);
        CHECK(ln.turnProgress > 0.0 && ln.turnProgress < 0.15,
              "turn starts from near-zero progress at the tangent point");
        CHECK(ln.desiredCourseMagDeg < 89.9 && ln.desiredCourseMagDeg > 80.0,
              "arc entry stays close to the inbound course");

        // A lateral offset must not look like turn progress: the bearing from
        // the centre is unchanged, while the chord to the tangent point would
        // read as a quarter of the turn already flown.
        s = MakeState(2.0 / 60.04, tangentLon, 90.0);
        ln = guidance.computeLnav(s, r, cdi, cfg);
        CHECK(ln.turnProgress < 0.05,
              "lateral offset does not fake arc progress");
    }

    // Multi-point legs (holds/arcs) must pass through untouched.
    {
        RouteSnapshot r = MakeRoute();
        r.legs[0] = RouteLeg{0, 0, 1, true};
        AircraftState s = MakeState(0.1, 0.5, 85.0);
        CdiInput cdi;
        cdi.valid = true;
        cdi.courseMagDeg = 90.0;
        cdi.distanceNm = 10.0;
        cdi.xtkNm = 0.5;
        LnavResult ln = guidance.computeLnav(s, r, cdi, cfg);
        CHECK(!ln.active, "complex leg passes through");
    }

    // CDI fallback: positive dot is interpreted as a right correction.
    {
        RouteSnapshot empty;
        AircraftState s = MakeState(0.0, 0.0, 90.0);
        CdiInput cdi;
        cdi.valid = true;
        cdi.courseMagDeg = 90.0;
        cdi.distanceNm = 10.0;
        cdi.xtkNm = 0.5;
        LnavResult ln = guidance.computeLnav(s, empty, cdi, cfg);
        CHECK(ln.active, "CDI fallback active");
        CHECK(ln.cdiFallback, "CDI fallback flag");
        CHECK(ln.desiredCourseMagDeg > 90.0, "positive dot -> turn right");
    }

    // A CDU LEGS cache that starts at the active waypoint has no inbound fix
    // for leg 0.  The geometry layer must reconstruct the inbound course and
    // compute a real cross-track instead of reporting zero XTK.
    {
        RouteSnapshot r = MakeCduRoute();
        AircraftState s = MakeState(0.1, 0.0, 90.0);
        CdiInput cdi;
        cdi.valid = true;
        cdi.courseMagDeg = 90.0;
        cdi.distanceNm = 10.0;
        cdi.xtkNm = 0.5;
        LnavResult ln = guidance.computeLnav(s, r, cdi, cfg);
        CHECK(ln.active, "first CDU leg with no inbound fix active");
        CHECK(ln.routeBased, "first CDU leg is route-based");
        CHECK(!ln.cdiFallback, "first CDU leg does not fall back to CDI dots");
        CHECK(ln.source == "ROUTE/SYNTH", "first CDU leg reports synthetic inbound");
        CHECK(ln.xtkNm < 0.0, "synthetic inbound gives a real cross-track");
        CHECK(ln.desiredCourseMagDeg > 90.0, "left of course -> turn right");
    }

    // CDI sign calibration against the FF757's FD roll.  Negative dot with a
    // right bank must eventually flip to a right correction.
    {
        RouteSnapshot empty;
        AircraftState s = MakeState(0.0, 0.0, 90.0);
        s.fdRollDeg = 15.0;
        CdiInput cdi;
        cdi.valid = true;
        cdi.courseMagDeg = 90.0;
        cdi.distanceNm = 10.0;
        cdi.xtkNm = -0.5;
        LnavResult ln;
        for (int i = 0; i < 12; ++i) {
            ln = guidance.computeLnav(s, empty, cdi, cfg);
        }
        CHECK(guidance.cdiSign() == -1, "FD roll calibrates CDI sign");
        CHECK(ln.desiredCourseMagDeg > 90.0, "calibrated CDI dot turns right");
    }

    // VNAV path: 2000 ft to lose over 6 nm at 400 kt = 2222 fpm (inside TOD).
    // The aircraft must already be descending; the plugin only assists, it
    // never initiates a path while level.
    {
        RouteSnapshot r = MakeRoute();
        r.wpts[1].altFt = 8000;
        r.wpts[1].altValid = true;
        AircraftState s = MakeState(0.0, 0.9, 90.0);
        s.altFt = 10000.0;
        s.vsFpm = -1500.0;
        VnavResult vn = guidance.computeVnav(s, r, cfg);
        CHECK(vn.active, "VNAV active");
        CHECK(vn.pathBased, "VNAV path based");
        CHECK(!vn.passThrough, "established descent writes a correction");
        CHECK_NEAR(vn.requiredVsFpm, -2222.2, 20.0, "required VS");
        CHECK_NEAR(vn.correctionDeg, -0.578, 0.05, "VNAV correction from the VS error");
    }

    // Implausible low altitude at high altitude must not command a dive.
    {
        RouteSnapshot r = MakeRoute();
        r.wpts[1].altFt = 350;       // ambiguous "350" string
        r.wpts[1].altValid = true;
        AircraftState s = MakeState(0.0, 0.667, 90.0);
        s.altFt = 35000.0;
        VnavResult vn = guidance.computeVnav(s, r, cfg);
        CHECK(vn.active && !vn.pathBased && std::abs(vn.correctionDeg) < 0.01,
              "low-altitude guard falls back to FD smoothing, no dive command");
    }

    // A 25000 ft change five miles from the fix is not a plausible path.
    {
        RouteSnapshot r = MakeRoute();
        r.wpts[1].altFt = 10000;
        r.wpts[1].altValid = true;
        AircraftState s = MakeState(0.0, 0.9167, 90.0);  // ~5 nm to BBB
        s.altFt = 35000.0;
        VnavResult vn = guidance.computeVnav(s, r, cfg);
        CHECK(vn.active && !vn.pathBased, "distance-scaled plausibility guard falls back to smoothing");
    }

    // VNAV clamp.
    {
        RouteSnapshot r = MakeRoute();
        r.wpts[1].altFt = 12000;
        r.wpts[1].altValid = true;
        AircraftState s = MakeState(0.1, 0.5, 90.0);
        s.altFt = 10000.0;
        s.vsFpm = 1000.0;
        VnavResult vn = guidance.computeVnav(s, r, cfg);
        CHECK(vn.active, "VNAV clamp active");
        CHECK(std::abs(vn.correctionDeg) <= cfg.vnavMaxCorrectionDeg + 1e-6,
              "VNAV correction clamped");
    }

    // VNAV profile: active altitude equals current, so the next lower leg is
    // used.  Distance is accumulated along the route, not a straight line to
    // the active waypoint.
    {
        RouteSnapshot r = MakeCduRoute();
        r.wpts[0].altFt = 15000;
        r.wpts[0].altValid = true;
        r.wpts[1].altFt = 11000;
        r.wpts[1].altValid = true;
        r.wpts[2].altFt = 6000;
        r.wpts[2].altValid = true;
        AircraftState s = MakeState(0.0, -0.1, 90.0);  // ~6 nm before AAA
        s.altFt = 15000.0;
        s.gsKts = 400.0;
        VnavResult vn = guidance.computeVnav(s, r, cfg);
        CHECK(vn.active && !vn.pathBased, "before-TOD hold active");
        CHECK_NEAR(vn.targetAltFt, 15000.0, 1.0, "before TOD holds current altitude");
        CHECK_NEAR(vn.requiredVsFpm, 0.0, 1.0, "before TOD commands zero VS");
        CHECK(vn.note.find("BBB") != std::string::npos, "before-TOD note names the target waypoint");
    }

    // VNAV descent to the active waypoint, inside the 3-degree TOD window.
    {
        RouteSnapshot r = MakeCduRoute();
        r.wpts[0].altFt = 13500;
        r.wpts[0].altValid = true;
        AircraftState s = MakeState(0.0, -0.0833, 90.0);  // ~5 nm before AAA
        s.altFt = 15000.0;
        s.gsKts = 400.0;
        s.vsFpm = -1500.0;
        VnavResult vn = guidance.computeVnav(s, r, cfg);
        CHECK(vn.active && vn.pathBased, "active-waypoint descent active");
        CHECK_NEAR(vn.targetAltFt, 13500.0, 1.0, "active descent target altitude");
        CHECK_NEAR(vn.requiredVsFpm, -2000.0, 60.0, "active descent required VS");
    }

    // VNAV climb.
    {
        RouteSnapshot r = MakeCduRoute();
        r.wpts[0].altFt = 16000;
        r.wpts[0].altValid = true;
        AircraftState s = MakeState(0.0, -0.5, 90.0);
        s.altFt = 10000.0;
        s.gsKts = 400.0;
        s.vsFpm = 1000.0;
        VnavResult vn = guidance.computeVnav(s, r, cfg);
        CHECK(vn.active && vn.pathBased, "climb profile active");
        CHECK_NEAR(vn.targetAltFt, 16000.0, 1.0, "climb target altitude");
        CHECK_NEAR(vn.requiredVsFpm, 1333.0, 50.0, "climb required VS");
    }

    // VNAV SPD uses pitch to hold speed.  The default must pass through with
    // no FD pitch write; the opt-in restores the path correction.
    {
        RouteSnapshot r = MakeRoute();
        r.wpts[1].altFt = 16000;
        r.wpts[1].altValid = true;
        AircraftState s = MakeState(0.0, -0.5, 90.0);
        s.altFt = 10000.0;
        s.vnavSpeedMode = true;
        VnavResult vn = guidance.computeVnav(s, r, cfg);
        CHECK(vn.passThrough, "VNAV SPD passes pitch through by default");
        CHECK(vn.active, "VNAV SPD still reports the mode as active");
        CHECK_NEAR(vn.correctionDeg, 0.0, 1e-9, "VNAV SPD correction is zero");

        cfg.vnavAugmentSpeedMode = true;
        vn = guidance.computeVnav(s, r, cfg);
        CHECK(!vn.passThrough && vn.active, "VNAV SPD augmentation is opt-in");
        CHECK(vn.pathBased, "opt-in VNAV SPD uses the path");
        cfg.vnavAugmentSpeedMode = false;
    }

    // A level aircraft must never be pushed into a route descent by the
    // plugin; the FF757 owns vertical-path initiation.
    {
        RouteSnapshot r = MakeRoute();
        r.wpts[1].altFt = 8000;
        r.wpts[1].altValid = true;
        AircraftState s = MakeState(0.0, 0.9, 90.0);  // ~6 nm to BBB
        s.altFt = 12000.0;
        s.vsFpm = 0.0;
        VnavResult vn = guidance.computeVnav(s, r, cfg);
        CHECK(vn.active && !vn.pathBased, "level aircraft does not start a descent");
        CHECK(vn.passThrough, "level aircraft passes pitch through");
        CHECK_NEAR(vn.requiredVsFpm, 0.0, 1.0, "level aircraft keeps zero VS");
        CHECK_NEAR(vn.correctionDeg, 0.0, 1e-6, "level aircraft gets no path correction");
        CHECK(vn.source == "FF757 OWNS VS", "level aircraft reports pass-through source");
    }

    // MCP altitude must act as a floor.  A lower route constraint must not
    // command a descent when the aircraft is level at the selected altitude.
    {
        RouteSnapshot r = MakeRoute();
        r.wpts[1].altFt = 8000;
        r.wpts[1].altValid = true;
        AircraftState s = MakeState(0.0, 0.9, 90.0);  // ~6 nm to BBB
        s.altFt = 34100.0;
        s.selectedAltFt = 34100.0;
        s.vsFpm = 0.0;
        VnavResult vn = guidance.computeVnav(s, r, cfg);
        CHECK(vn.active && !vn.pathBased, "MCP altitude suppresses the descent");
        CHECK(vn.passThrough, "MCP hold passes pitch through");
        CHECK_NEAR(vn.targetAltFt, 34100.0, 1.0, "MCP altitude is the effective target");
        CHECK_NEAR(vn.requiredVsFpm, 0.0, 1.0, "MCP hold commands zero VS");
        CHECK_NEAR(vn.correctionDeg, 0.0, 1e-6, "MCP hold commands no pitch");
        CHECK(vn.note.find("MCP") != std::string::npos, "MCP hold note");
    }

    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
