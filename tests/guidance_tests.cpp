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
    cfg.vnavSpeedLimitsEnabled = false;   // legacy VNAV path tests run without speed limits
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
        const double radius = turnRadiusNm(400.0, cfg.lnavTurnBankDeg);
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
    // A fresh Guidance is used because the route hold keeps the previous
    // route command alive for ~1 s after the snapshot disappears.
    {
        Guidance gCdi;
        RouteSnapshot empty;
        AircraftState s = MakeState(0.0, 0.0, 90.0);
        CdiInput cdi;
        cdi.valid = true;
        cdi.courseMagDeg = 90.0;
        cdi.distanceNm = 10.0;
        cdi.xtkNm = 0.5;
        LnavResult ln = gCdi.computeLnav(s, empty, cdi, cfg);
        CHECK(ln.active, "CDI fallback active");
        CHECK(ln.cdiFallback, "CDI fallback flag");
        CHECK(ln.desiredCourseMagDeg > 90.0, "positive dot -> turn right");
    }

    // With no route ever seen, the hold does not engage: after a valid route
    // has been lost for more than the hold window, CDI steering takes over.
    {
        Guidance gHold;
        RouteSnapshot r = MakeRoute();
        CdiInput cdi;
        AircraftState onRoute = MakeState(0.0, 0.5, 90.0);
        gHold.computeLnav(onRoute, r, cdi, cfg);          // seed the hold

        RouteSnapshot empty;
        AircraftState s = MakeState(0.0, 0.5, 90.0);
        cdi.valid = true;
        cdi.courseMagDeg = 90.0;
        cdi.distanceNm = 10.0;
        cdi.xtkNm = 0.5;
        LnavResult ln;
        int heldFrames = 0;
        for (int i = 0; i < 30; ++i) {
            ln = gHold.computeLnav(s, empty, cdi, cfg);
            if (ln.source == "ROUTE HOLD") ++heldFrames;
        }
        CHECK(heldFrames >= 15 && heldFrames <= 22,
              "route hold covers ~20 frames before CDI takes over");
        CHECK(ln.cdiFallback,
              "CDI takes over only after the hold window expires");
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
    // right bank must eventually flip to a right correction.  The check needs
    // the track aligned with the CDI course and ~2 s of continuous
    // disagreement, so a recovery turn cannot flip it.
    {
        Guidance gCdiSign;
        RouteSnapshot empty;
        AircraftState s = MakeState(0.0, 0.0, 90.0);
        s.fdRollDeg = 15.0;
        CdiInput cdi;
        cdi.valid = true;
        cdi.courseMagDeg = 90.0;
        cdi.distanceNm = 10.0;
        cdi.xtkNm = -0.5;
        LnavResult ln;
        for (int i = 0; i < 140; ++i) {
            ln = gCdiSign.computeLnav(s, empty, cdi, cfg);
        }
        CHECK(gCdiSign.cdiSign() == -1, "FD roll calibrates CDI sign");
        CHECK(ln.desiredCourseMagDeg > 90.0, "calibrated CDI dot turns right");
    }

    // Forced GPS CDI source: the sign is frozen, so a disagreeing FD roll must
    // not flip it (live 1.2.23 regression: xtk +0.97 -> -0.89, corr
    // -3.46 -> +3.41 while the aircraft stayed right of course).
    {
        Guidance gForced;
        RouteSnapshot empty;
        AircraftState s = MakeState(0.0, 0.0, 90.0);
        s.fdRollDeg = 15.0;
        CdiInput cdi;
        cdi.valid = true;
        cdi.courseMagDeg = 90.0;
        cdi.distanceNm = 10.0;
        cdi.xtkNm = -0.5;
        Config c = cfg;
        c.lnavForceCdi = true;
        LnavResult ln;
        for (int i = 0; i < 200; ++i) {
            ln = gForced.computeLnav(s, empty, cdi, c);
        }
        CHECK(gForced.cdiSign() == 1, "forced CDI keeps the calibrated sign");
        CHECK(ln.desiredCourseMagDeg < 90.0, "forced CDI dot still turns left");
    }

    // Once a trusted route cross-track confirms the dot sign, the FD-roll
    // bootstrap must stay out of the way even when the route later drops out.
    {
        Guidance gVerified;
        RouteSnapshot r = MakeRoute();
        AircraftState left = MakeState(0.1, 0.5, 90.0);   // left of course
        CdiInput cdi;
        cdi.valid = true;
        cdi.courseMagDeg = 90.0;
        cdi.distanceNm = 10.0;
        cdi.xtkNm = 0.5;   // left of course -> turn right
        gVerified.computeLnav(left, r, cdi, cfg);
        CHECK(gVerified.cdiSign() == 1, "route agreement confirms the sign");

        RouteSnapshot empty;
        AircraftState s = MakeState(0.0, 0.0, 90.0);
        s.fdRollDeg = 15.0;
        for (int i = 0; i < 200; ++i) {
            gVerified.computeLnav(s, empty, cdi, cfg);
        }
        CHECK(gVerified.cdiSign() == 1, "verified sign is not flipped by FD roll");
    }

    // Manual GPS CDI source: the route is valid but the switch forces the
    // CDI path, matching the automatic fallback with a clear note.
    {
        RouteSnapshot r = MakeRoute();
        AircraftState s = MakeState(0.0, 0.5, 90.0);
        CdiInput cdi;
        cdi.valid = true;
        cdi.courseMagDeg = 90.0;
        cdi.distanceNm = 10.0;
        cdi.xtkNm = 0.5;
        Config c = cfg;
        c.lnavForceCdi = true;
        Guidance gCdi;
        LnavResult ln = gCdi.computeLnav(s, r, cdi, c);
        CHECK(ln.active && ln.source == "GPS CDI",
              "forced GPS CDI source is used");
        CHECK(!ln.routeBased, "forced CDI does not use route geometry");
        CHECK(ln.note.find("forced") != std::string::npos,
              "forced CDI note is explicit");
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
        CHECK_NEAR(vn.correctionDeg, -0.867, 0.05, "VNAV correction from the VS error");
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

    // "At or above" constraints are floors, not crossing targets.  Live route:
    // AA146 FL157A 43 NM ahead at FL340 demanded ~4000 fpm (impossible), and
    // the plugin fought the FMC's idle path.  The next exact constraint must
    // become the target instead, giving a flyable gradient.
    {
        RouteSnapshot r = MakeRoute();
        r.wpts[1].altFt = 15700;
        r.wpts[1].altValid = true;
        r.wpts[1].altConstraint = AltAtOrAbove;
        r.wpts[2].altFt = 11765;
        r.wpts[2].altValid = true;
        r.wpts[2].altConstraint = AltAt;

        AircraftState s = MakeState(0.0, 0.7, 90.0);  // 18 NM to BBB
        s.altFt = 28000.0;
        s.vsFpm = -1500.0;
        s.gsKts = 400.0;

        VnavResult vnA = guidance.computeVnav(s, r, cfg);
        CHECK(vnA.active && vnA.pathBased, "A-floor path active");
        CHECK_NEAR(vnA.targetAltFt, 11765.0, 1.0,
                   "satisfied FL157A is skipped, CCC 11765 is the target");
        CHECK(vnA.requiredVsFpm < -1500.0 && vnA.requiredVsFpm > -3000.0,
              "A-floor target gives a flyable descent rate");

        // Same geometry with an exact crossing: steeper demand, proving the
        // qualifier is what changes the target.
        r.wpts[1].altConstraint = AltAt;
        VnavResult vnAt = guidance.computeVnav(s, r, cfg);
        CHECK_NEAR(vnAt.targetAltFt, 15700.0, 1.0,
                   "exact crossing still targets BBB");
        CHECK(vnAt.requiredVsFpm < vnA.requiredVsFpm - 500.0,
              "exact crossing demands a steeper descent than an A-floor");
    }

    // A skipped "at or above" floor still limits the descent gradient: the
    // straight line to the next hard target would cross it below its minimum.
    {
        RouteSnapshot r;
        r.valid = true;
        r.sourceRoute = true;
        r.wptCount = 3;
        r.legCount = 2;
        r.activeWpt = 1;
        r.activeLeg = 0;
        r.nextLeg = 1;
        r.wpts.resize(3);
        r.wpts[0].lat = 0.0;  r.wpts[0].lon = 0.0;  r.wpts[0].id = "AAA";
        r.wpts[1].lat = 0.0;  r.wpts[1].lon = 1.0;  r.wpts[1].id = "BBB";
        r.wpts[2].lat = 0.0;  r.wpts[2].lon = 1.25; r.wpts[2].id = "CCC";
        for (auto& w : r.wpts) w.posValid = true;
        r.wpts[1].altFt = 15700;
        r.wpts[1].altValid = true;
        r.wpts[1].altConstraint = AltAtOrAbove;
        r.wpts[2].altFt = 7000;
        r.wpts[2].altValid = true;
        r.wpts[2].altConstraint = AltAt;
        r.legs.resize(2);
        r.legs[0] = RouteLeg{0, 0, 1, false};
        r.legs[1] = RouteLeg{0, 1, 2, false};

        AircraftState s = MakeState(0.0, 0.7, 90.0);  // 18 NM to BBB
        s.altFt = 18000.0;
        s.vsFpm = -400.0;   // keep the clamped error outside the FD deadband
        s.gsKts = 400.0;

        VnavResult vn = guidance.computeVnav(s, r, cfg);
        CHECK(vn.active && vn.pathBased, "floor clamp path active");
        CHECK_NEAR(vn.targetAltFt, 7000.0, 1.0,
                   "floor clamp keeps the hard target");
        // Line to CCC would need -2222 fpm and cross BBB at ~12000 ft.
        // The floor allows (18000-15700) ft over 18 NM at 400 kt = -852 fpm.
        CHECK_NEAR(vn.requiredVsFpm, -852.0, 30.0,
                   "descent limited by the satisfied at-or-above floor");
        CHECK(vn.note.find("at-or-above floor kept") != std::string::npos,
              "floor clamp is reported in the note");

        // Without the floor the same route goes to CCC directly and crosses
        // BBB at ~12000 ft, so the clamp is what keeps the floor intact.
        r.wpts[1].altValid = false;
        VnavResult vnNoFloor = guidance.computeVnav(s, r, cfg);
        CHECK_NEAR(vnNoFloor.targetAltFt, 7000.0, 1.0, "no-floor target CCC");
        CHECK_NEAR(vnNoFloor.requiredVsFpm, -2222.0, 60.0,
                   "no-floor line needs the steeper descent");
        CHECK(vnNoFloor.requiredVsFpm < vn.requiredVsFpm - 800.0,
              "floor clamp is what limits the descent");
    }

    // Pitch-authority hysteresis: once the plugin engages on a large VS error
    // it must keep pitch until the integrated path deviation is small, not
    // hand it back to the FF757 on the first small-error frame.
    {
        RouteSnapshot r = MakeCduRoute();
        r.wpts[0].altFt = 13500;
        r.wpts[0].altValid = true;
        AircraftState s = MakeState(0.0, -0.0833, 90.0);  // ~5 nm before AAA
        s.altFt = 15000.0;
        s.gsKts = 400.0;

        Guidance g;
        // 500 fpm short for 30 s: engages and accumulates ~250 ft.
        s.vsFpm = -1500.0;
        VnavResult a = g.computeVnav(s, r, cfg, 30.0);
        CHECK(a.pathBased && a.pathHeld, "large VS error engages pitch");

        // VS now matches the profile: the 250 ft deviation keeps it engaged.
        s.vsFpm = -2000.0;
        VnavResult b = g.computeVnav(s, r, cfg, 30.0);
        CHECK(b.pathBased && b.pathHeld,
              "authority held while 250 ft of deviation remains");

        // Descend faster than required to work the deviation off.
        s.vsFpm = -2600.0;
        g.computeVnav(s, r, cfg, 30.0);
        s.vsFpm = -2000.0;
        VnavResult c = g.computeVnav(s, r, cfg, 30.0);
        CHECK(c.passThrough && !c.pathHeld,
              "authority released once deviation and VS error are small");

        // A single small-error frame on a fresh instance must not engage.
        Guidance fresh;
        s.vsFpm = -1850.0;  // 150 fpm short
        VnavResult d = fresh.computeVnav(s, r, cfg, 1.0);
        CHECK(d.passThrough && !d.pathBased, "150 fpm does not engage");
    }

    // A target thousands of miles ahead comes from a mis-resolved waypoint.
    // Live report: FI07 at 2241.8 NM produced reqVS=-43 fpm.
    {
        RouteSnapshot r = MakeCduRoute();
        r.wpts[0].altValid = false;
        r.wpts[1].lat = 0.0;
        r.wpts[1].lon = 12.0;   // ~720 NM from AAA
        r.wpts[1].altFt = 11000;
        r.wpts[1].altValid = true;
        r.wpts[2].altValid = false;
        AircraftState s = MakeState(0.0, -0.0833, 90.0);
        s.altFt = 19200.0;
        s.gsKts = 356.0;
        s.vsFpm = -1783.0;
        VnavResult vn = guidance.computeVnav(s, r, cfg);
        CHECK(!vn.pathBased, "far target is rejected, no bogus reqVS");
        CHECK(vn.passThrough, "far target falls back to pass-through");
        CHECK(vn.note.find("no usable altitude target") != std::string::npos,
              "far target is reported as unusable");
    }

    // FAR 250/10000 must not arm at cruise for a low-altitude waypoint still
    // 100+ NM ahead (live log: 250 kt target printed at 106 NM).
    {
        Guidance g;
        RouteSnapshot r = MakeRoute();
        r.wpts[1].altFt = 15700;
        r.wpts[1].altValid = true;
        r.wpts[2].altFt = 6890;
        r.wpts[2].altValid = true;
        r.wpts[2].altConstraint = AltAtOrBelow;
        AircraftState s = MakeState(0.0, 0.75, 90.0);  // 15 NM to BBB, ~45 to CCC
        s.altFt = 33000.0;
        s.gsKts = 500.0;
        s.iasKts = 300.0;
        Config c;
        VnavResult vn = g.computeVnav(s, r, c);
        CHECK(vn.targetSpdKt < 0.5,
              "no 250 kt target while still 45 NM and 23000 ft above it");
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

    // The same "before TOD" geometry must NOT hold once the aircraft is
    // already descending: the FF757's VNAV path is live even if the 3-degree
    // estimate says the fix is farther away.  Live status showed PTH STBY /
    // ROUTE HOLD with vs=-1619 during a VNAV PTH descent.
    {
        RouteSnapshot r = MakeCduRoute();
        r.wpts[0].altFt = 15000;
        r.wpts[0].altValid = true;
        r.wpts[1].altFt = 11000;
        r.wpts[1].altValid = true;
        AircraftState s = MakeState(0.0, -0.1, 90.0);  // ~6 nm before AAA
        s.altFt = 15000.0;
        s.gsKts = 400.0;
        s.vsFpm = -1500.0;  // already descending
        VnavResult vn = guidance.computeVnav(s, r, cfg);
        CHECK(vn.active && vn.pathBased, "descending before-TOD estimate still flies the path");
        CHECK_NEAR(vn.targetAltFt, 11000.0, 1.0, "next constraint is the target");
        CHECK_NEAR(vn.requiredVsFpm, -741.0, 60.0, "required VS for the live descent");
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

    // MCP below the current altitude must not turn an above-us crossing
    // estimate into a dive: selecting 3000 at 8300 with a 8625 ft fix ahead
    // used to command -1800 fpm (live ZSNJ "3000 ft (MCP limited)").  Hold.
    {
        Guidance gMcpLow;
        RouteSnapshot r = MakeRoute();
        r.wpts[1].altFt = 8625;
        r.wpts[1].altValid = true;
        AircraftState s = MakeState(0.0, 0.9, 90.0);   // ~6 nm to BBB
        s.altFt = 8300.0;
        s.selectedAltFt = 3000.0;
        s.vsFpm = 0.0;
        VnavResult vn = gMcpLow.computeVnav(s, r, cfg);
        CHECK(vn.active, "MCP below and target above: mode active");
        CHECK(vn.passThrough, "MCP below and target above: hold, no dive");
        CHECK_NEAR(vn.requiredVsFpm, 0.0, 1.0, "no dive commanded");
        CHECK_NEAR(vn.targetAltFt, 8300.0, 1.0, "holds the current altitude");
    }

    // Boeing course capture (777): |xtk| beyond the blend distance commands a
    // constant 30 deg intercept; small offsets home direct to the fix, so the
    // angle is atan(xtk / distance-to-fix) and decays as the offset closes.
    // A long-range S-turn from a small offset must never happen.
    {
        Guidance g5;
        RouteSnapshot r = MakeRoute();
        CdiInput cdi;

        // Large offset: full constant intercept regardless of distance.
        AircraftState s = MakeState(0.1, 0.5, 90.0);      // ~6 nm left of course
        LnavResult ln = g5.computeLnav(s, r, cdi, cfg);
        CHECK_NEAR(ln.desiredCourseMagDeg, 120.0, 0.1,
                   "large xtk commands the full 30 deg intercept");

        // Small offset 20 nm out: pure direct-to-fix homing, ~3.6 deg.
        Guidance g5b;
        s = MakeState(0.02083, 0.6667, 90.0);             // ~1.25 nm off, 20 nm to go
        ln = g5b.computeLnav(s, r, cdi, cfg);
        const double mid = ln.desiredCourseMagDeg - 90.0;
        CHECK(mid > 1.0 && mid < 6.0,
              "1.25 nm offset 20 nm out homes to the fix, no forced intercept");

        // Small offset far away: even gentler.
        Guidance g5c;
        s = MakeState(0.0083, 0.0, 90.0);                 // ~0.5 nm off, 60 nm to go
        ln = g5c.computeLnav(s, r, cdi, cfg);
        const double farErr = ln.desiredCourseMagDeg - 90.0;
        CHECK(farErr > 0.2 && farErr < 1.2,
              "0.5 nm offset 60 nm out is a fraction of a degree");

        // Hand-over band: at 3 nm the constant capture angle is fully applied.
        Guidance g5d;
        s = MakeState(0.05, 0.6667, 90.0);                // 3 nm off, 20 nm to go
        ln = g5d.computeLnav(s, r, cdi, cfg);
        const double bandErr = ln.desiredCourseMagDeg - 90.0;
        CHECK(bandErr > 26.0 && bandErr < 31.0,
              "3 nm offset hands over to the 30 deg capture angle");
    }

    // Near-fix behaviour: homing aims at the fix, so the last mile can demand
    // a larger angle; the 45 deg cap still applies.
    {
        Guidance g6;
        RouteSnapshot r = MakeRoute();
        CdiInput cdi;
        AircraftState s = MakeState(0.0083, 0.983, 90.0);  // ~1 nm to BBB
        s.gsKts = 130.0;                                   // tangent < 1 nm, no arc
        LnavResult ln = g6.computeLnav(s, r, cdi, cfg);
        CHECK(ln.active, "near-fix intercept active");
        const double err = ln.desiredCourseMagDeg - 90.0;
        CHECK(err > 22.0 && err < 32.0,
              "near-fix homing aims at the fix (atan of offset over distance)");
    }

    // Roll-out lead: on the last few degrees of the arc the target is frozen
    // on the outbound course so the AP levels out on course; progress reads
    // 100% while frozen.  Before the freeze window the arc is still tracked.
    {
        Guidance g7;
        Config c7 = cfg;
        RouteSnapshot r = MakeRoute();          // left turn 090 -> 000 at BBB
        CdiInput cdi;

        // Points on the tangent arc for the configured turn bank (16 deg) and
        // the 4 deg rollout lead: T2 is 8.13 nm north of BBB, arc start radial
        // 180, end radial 090.
        // ~2 deg before the outbound tangent point T2 (inside the 4 deg lead).
        AircraftState s = MakeState(0.130668, 0.999913, 355.0);
        LnavResult ln = g7.computeLnav(s, r, cdi, c7);
        CHECK(ln.active, "rollout-lead state active");
        CHECK(std::abs(normalize180(ln.desiredCourseMagDeg)) < 0.5,
              "target frozen on the outbound course inside the lead window");
        CHECK(ln.turnProgress >= 0.999, "progress reads complete while frozen");

        AircraftState s2 = MakeState(0.111874, 0.997943, 15.0);   // ~80 deg turned
        ln = g7.computeLnav(s2, r, cdi, c7);
        CHECK(ln.active, "mid-arc state active");
        CHECK(std::abs(normalize180(ln.desiredCourseMagDeg) - 10.0) < 2.5,
              "mid-arc target still follows the tangent course");
    }

    // --- RNP / ANP monitoring (777 FMC logic) ---
    // Phase auto-selection: enroute RNP 2.0, terminal RNP 1.0 within 30 NM of
    // the destination, approach RNP 0.3 within 8 NM.  distToDest is the
    // remaining distance to the active fix plus the rest of the route.
    {
        Guidance g3;
        CdiInput cdi;

        RouteSnapshot far = MakeRoute();     // BBB->CCC = 30 nm
        AircraftState s = MakeState(0.0, 0.5, 90.0);   // 30 + 30 = 60 nm out
        LnavResult ln = g3.computeLnav(s, far, cdi, cfg);
        CHECK_NEAR(ln.rnpNm, 2.0, 1e-9, "enroute RNP 2.0 far from destination");

        RouteSnapshot nearR = MakeRoute();
        nearR.wpts[2].lat = 0.0;             // BBB->CCC = 6 nm
        nearR.wpts[2].lon = 1.1;
        s = MakeState(0.0, 0.9, 90.0);       // 6 + 6 = 12 nm out
        ln = g3.computeLnav(s, nearR, cdi, cfg);
        CHECK_NEAR(ln.rnpNm, 1.0, 1e-9, "terminal RNP 1.0 within 30 nm");
        s = MakeState(0.0, 0.98, 90.0);      // 1.2 + 6 = 7.2 nm out
        ln = g3.computeLnav(s, nearR, cdi, cfg);
        CHECK_NEAR(ln.rnpNm, 0.3, 1e-9, "approach RNP 0.3 within 8 nm");
    }

    // ANP is a 95% containment proxy (2 x filtered |xtk|).  Sustained ANP >
    // RNP raises the alert; clearing needs a clean margin (hysteresis), like
    // the FMC's NAV UNABLE RNP message.
    {
        Guidance g4;
        Config c4 = cfg;
        c4.rnpNm = 0.5;                      // force a tight requirement
        RouteSnapshot r = MakeRoute();
        CdiInput cdi;

        bool raised = false;
        for (int i = 0; i < 60; ++i) {
            AircraftState s = MakeState(0.05, 0.5, 90.0);   // ~3 nm off course
            LnavResult ln = g4.computeLnav(s, r, cdi, c4);
            raised = ln.rnpUnable;
        }
        CHECK(raised, "sustained ANP > RNP raises NAV UNABLE RNP");

        bool cleared = false;
        for (int i = 0; i < 260; ++i) {
            AircraftState s = MakeState(0.001, 0.5, 90.0);  // back on track
            LnavResult ln = g4.computeLnav(s, r, cdi, c4);
            cleared = !ln.rnpUnable;
        }
        CHECK(cleared, "ANP back inside 0.8*RNP clears the alert");
    }

    // --- RF (constant-radius-to-fix) arc tracking ---
    // Geometry from the real KSEA RNAV (RNP) Z RWY 16C approach: arc
    // HEGEM -> SEGAW, centre CFBRK, R = 3.150 NM, left turn, swept ~72 deg.
    auto MakeRfRoute = [](bool complexLeg) {
        RouteSnapshot r;
        r.valid = true;
        r.sourceRoute = true;
        r.wptCount = 4;
        r.legCount = 3;
        r.activeWpt = 2;
        r.activeLeg = 1;
        r.nextLeg = 2;

        r.wpts.resize(4);
        r.wpts[0].lat = 47.697239; r.wpts[0].lon = -122.483956; r.wpts[0].id = "FERIE";
        r.wpts[1].lat = 47.646133; r.wpts[1].lon = -122.459689; r.wpts[1].id = "HEGEM";
        r.wpts[2].lat = 47.609656; r.wpts[2].lon = -122.385864; r.wpts[2].id = "SEGAW";
        r.wpts[3].lat = 47.559656; r.wpts[3].lon = -122.385864; r.wpts[3].id = "NEXT";
        for (auto& w : r.wpts) w.posValid = true;

        r.legs.resize(3);
        r.legs[0] = RouteLeg{0, 0, 1, false};
        r.legs[2] = RouteLeg{0, 2, 3, false};

        RouteLeg& rf = r.legs[1];
        rf.type = 0;
        rf.firstWpt = 1;
        rf.lastWpt = 2;
        rf.complex = complexLeg;
        rf.rfArc = true;
        rf.rfRadiusNm = 3.150;
        rf.rfCenterLat = 47.662125;
        rf.rfCenterLon = -122.385728;
        rf.rfTurn = -1;                 // left
        rf.rfRadialStartDeg = 252.23;   // centre -> HEGEM
        rf.rfRadialEndDeg = 180.10;     // centre -> SEGAW
        rf.rfTotalSweptDeg = 72.13;
        rf.rfEntryWpt = 1;
        rf.rfEntryId = "HEGEM";
        rf.rfEntryLat = 47.646133;
        rf.rfEntryLon = -122.459689;
        rf.rfEntryValid = true;
        return r;
    };

    // On the circle at the entry fix the target is the inbound tangent
    // (252.23 - 90 = 162.23 true), matching the FERIE->HEGEM course of 162.
    {
        Guidance g;
        RouteSnapshot r = MakeRfRoute(true);
        CdiInput cdi;
        AircraftState s = MakeState(47.646133, -122.459689, 162.0);
        LnavResult ln = g.computeLnav(s, r, cdi, cfg);
        CHECK(ln.active && ln.source == "RF ARC", "RF arc branch active");
        CHECK_NEAR(ln.desiredCourseMagDeg, 162.23, 2.0,
                   "RF entry target is the tangent course");
        CHECK(ln.turnProgress < 0.05, "RF progress ~0 at entry");
    }

    // Flying the arc in small steps: desired course sweeps monotonically to
    // the left and progress never runs backwards.
    {
        Guidance g;
        RouteSnapshot r = MakeRfRoute(true);
        CdiInput cdi;
        double prevCourse = 999.0;
        double prevProg = -1.0;
        bool monotonicCourse = true;
        bool monotonicProg = true;
        for (double radial = 250.0; radial >= 183.0; radial -= 2.0) {
            double lat = 0.0, lon = 0.0;
            destinationPoint(47.662125, -122.385728, radial, 3.15, lat, lon);
            AircraftState s = MakeState(lat, lon, radial - 90.0);
            LnavResult ln = g.computeLnav(s, r, cdi, cfg);
            if (!ln.active || ln.source != "RF ARC") monotonicCourse = false;
            if (prevCourse < 900.0) {
                if (normalize180(ln.desiredCourseMagDeg - prevCourse) > 1.0)
                    monotonicCourse = false;
                if (ln.turnProgress < prevProg - 1e-6) monotonicProg = false;
            }
            prevCourse = ln.desiredCourseMagDeg;
            prevProg = ln.turnProgress;
        }
        CHECK(monotonicCourse, "RF desired course sweeps monotonically left");
        CHECK(monotonicProg, "RF progress monotonic non-decreasing");
        CHECK(prevProg > 0.95, "RF progress reaches the roll-out window");
    }

    // Inside the roll-out lead (3 deg) the target freezes on the outbound
    // tangent: radialEnd 180.10 - 90 = 90.10 true.
    {
        Guidance g;
        RouteSnapshot r = MakeRfRoute(true);
        CdiInput cdi;
        double lat = 0.0, lon = 0.0;
        destinationPoint(47.662125, -122.385728, 181.5, 3.15, lat, lon);
        AircraftState s = MakeState(lat, lon, 95.0);
        LnavResult ln = g.computeLnav(s, r, cdi, cfg);
        CHECK_NEAR(ln.desiredCourseMagDeg, 90.10, 1.5,
                   "RF target frozen on the outbound course in the lead window");
        CHECK(ln.turnProgress >= 0.999, "RF progress complete while frozen");
    }

    // Off-circle correction: outside the circle the target rotates inward
    // from the tangent; inside it rotates outward.  xtk reports the radial
    // error in the route convention (positive = right of the path).
    {
        Guidance g;
        RouteSnapshot r = MakeRfRoute(true);
        CdiInput cdi;
        double lat = 0.0, lon = 0.0;
        destinationPoint(47.662125, -122.385728, 216.0, 3.65, lat, lon);
        AircraftState s = MakeState(lat, lon, 126.0);
        LnavResult ln = g.computeLnav(s, r, cdi, cfg);
        CHECK(ln.desiredCourseMagDeg > 95.0 && ln.desiredCourseMagDeg < 105.0,
              "RF outside the circle steers inward of the tangent (126 deg)");
        CHECK(ln.xtkNm > 0.4, "RF outside the circle is right of path on a left arc");

        destinationPoint(47.662125, -122.385728, 216.0, 2.65, lat, lon);
        s = MakeState(lat, lon, 126.0);
        ln = g.computeLnav(s, r, cdi, cfg);
        CHECK(ln.desiredCourseMagDeg > 148.0 && ln.desiredCourseMagDeg < 157.0,
              "RF inside the circle steers outward of the tangent");
        CHECK(ln.xtkNm < -0.4, "RF inside the circle is left of path on a left arc");
    }

    // lnav_rf_enabled=0 restores the old behaviour: the complex leg passes
    // through to the FF757 untouched.
    {
        Guidance g;
        Config c = cfg;
        c.lnavRfEnabled = false;
        RouteSnapshot r = MakeRfRoute(true);
        CdiInput cdi;
        AircraftState s = MakeState(47.646133, -122.459689, 162.0);
        LnavResult ln = g.computeLnav(s, r, cdi, c);
        CHECK(!ln.active, "RF disabled: complex leg passes through");
    }

    // A CDU-synthesised route has straight legs only; an annotated RF leg
    // (complex=false) must still be tracked as an arc.
    {
        Guidance g;
        RouteSnapshot r = MakeRfRoute(false);
        CdiInput cdi;
        AircraftState s = MakeState(47.646133, -122.459689, 162.0);
        LnavResult ln = g.computeLnav(s, r, cdi, cfg);
        CHECK(ln.active && ln.source == "RF ARC",
              "RF annotation on a straight CDU leg still tracks the arc");
    }

    // RF capture gate: the leg becomes active many miles before the arc entry.
    // The FF757 cannot fly RF legs, so the plugin must fly the inbound course
    // to the entry fix itself (lead-in) instead of handing the leg back.
    {
        Guidance g;
        RouteSnapshot r = MakeRfRoute(true);
        CdiInput cdi;
        // 8 NM before HEGEM along the inbound course (162.23 true).
        double lat = 0.0, lon = 0.0;
        destinationPoint(r.wpts[1].lat, r.wpts[1].lon,
                         normalize360(162.23 + 180.0), 8.0, lat, lon);
        AircraftState s = MakeState(lat, lon, 162.0);
        LnavResult ln = g.computeLnav(s, r, cdi, cfg);
        CHECK(ln.active, "RF lead-in is flown by the plugin, not handed back");
        CHECK(ln.note.find("RF arc lead-in") != std::string::npos,
              "note reports the RF lead-in");
        CHECK_NEAR(ln.desiredCourseMagDeg, 162.2, 2.5,
                   "lead-in follows the inbound course to the entry fix");
        CHECK(std::abs(ln.distanceNm - 8.0) < 1.0,
              "lead-in distance counts to the arc entry fix");
    }

    // The FF757's own route is unreliable on RNP segments, so the arc must not
    // be gated by the FMS course: purely geometric capture.
    {
        Guidance g;
        RouteSnapshot r = MakeRfRoute(true);
        CdiInput cdi;
        cdi.valid = true;
        cdi.courseMagDeg = normalize360(162.23 - 60.0);   // FMS disagrees by 60 deg
        cdi.distanceNm = 3.15;
        cdi.xtkNm = 0.0;
        AircraftState s = MakeState(47.646133, -122.459689, 162.0);
        LnavResult ln = g.computeLnav(s, r, cdi, cfg);
        CHECK(ln.active && ln.source == "RF ARC",
              "RF arc ignores the FMS course and captures from geometry alone");
        CHECK(ln.note.find("(course from FMS)") == std::string::npos,
              "RF tracking does not claim an FMS course anchor");
    }

    // On the circle at the entry the arc still captures (regression guard).
    {
        Guidance g;
        RouteSnapshot r = MakeRfRoute(true);
        CdiInput cdi;
        cdi.valid = true;
        cdi.courseMagDeg = 162.23;
        cdi.distanceNm = 3.15;
        cdi.xtkNm = 0.0;
        AircraftState s = MakeState(47.646133, -122.459689, 162.0);
        LnavResult ln = g.computeLnav(s, r, cdi, cfg);
        CHECK(ln.active && ln.source == "RF ARC",
              "RF arc captures on the circle at the entry with an agreeing FMS course");
    }

    // CDU cache style route: the snapshot starts at the active waypoint, so an
    // RF leg can be active with its entry fix absent.  Guidance must still fly
    // the lead-in and capture the arc, using the entry coordinates from CIFP.
    {
        RouteSnapshot r;
        r.valid = true;
        r.sourceRoute = true;
        r.wptCount = 2;
        r.legCount = 1;
        r.activeWpt = 0;                 // LS-type cache: starts at the exit
        r.activeLeg = 0;
        r.nextLeg = -1;
        r.wpts.resize(2);
        r.wpts[0].lat = 47.609656; r.wpts[0].lon = -122.385864; r.wpts[0].id = "SEGAW";
        r.wpts[1].lat = 47.559656; r.wpts[1].lon = -122.385864; r.wpts[1].id = "NEXT";
        r.wpts[0].posValid = true; r.wpts[1].posValid = true;

        RouteLeg leg;
        leg.type = 0;
        leg.firstWpt = 0;
        leg.lastWpt = 0;
        leg.complex = true;
        leg.rfArc = true;
        leg.rfRadiusNm = 3.150;
        leg.rfCenterLat = 47.662125;
        leg.rfCenterLon = -122.385728;
        leg.rfTurn = -1;
        leg.rfRadialStartDeg = 252.23;
        leg.rfRadialEndDeg = 180.10;
        leg.rfTotalSweptDeg = 72.13;
        leg.rfEntryWpt = -1;             // entry fix not in the snapshot
        leg.rfEntryId = "HEGEM";
        leg.rfEntryLat = 47.646133;
        leg.rfEntryLon = -122.459689;
        leg.rfEntryValid = true;
        r.legs.push_back(leg);

        Guidance gLead;
        CdiInput cdi;
        double lat = 0.0, lon = 0.0;
        destinationPoint(leg.rfEntryLat, leg.rfEntryLon,
                         normalize360(162.23 + 180.0), 8.0, lat, lon);
        AircraftState s = MakeState(lat, lon, 162.0);
        LnavResult ln = gLead.computeLnav(s, r, cdi, cfg);
        CHECK(ln.active, "RF lead-in works without the entry fix in the route");
        CHECK(ln.note.find("RF arc lead-in to HEGEM") != std::string::npos,
              "lead-in note names the CIFP entry fix");
        CHECK_NEAR(ln.desiredCourseMagDeg, 162.2, 2.5,
                   "lead-in course comes from the arc entry tangent");
        CHECK(std::abs(ln.distanceNm - 8.0) < 1.0,
              "lead-in distance counts to the CIFP entry point");

        Guidance gArc;
        AircraftState onCircle = MakeState(leg.rfEntryLat, leg.rfEntryLon, 162.0);
        LnavResult ln2 = gArc.computeLnav(onCircle, r, cdi, cfg);
        CHECK(ln2.active && ln2.source == "RF ARC",
              "arc captures at the entry even without the entry waypoint");

        // Exit-only CIFP match borrowed from another airport: the arc tangent
        // disagrees with the FMS course, so it must not take over.  Live 1.2.23
        // case: LS987 tangent 145.7 vs FMS 091.9 produced a 12.8 NM phantom xtk.
        Guidance gReject;
        CdiInput fms;
        fms.valid = true;
        fms.courseMagDeg = 108.2;   // 54 deg off the 162.2 entry tangent
        fms.distanceNm = 8.0;
        fms.xtkNm = 0.0;
        LnavResult ln3 = gReject.computeLnav(s, r, fms, cfg);
        CHECK(!ln3.active, "disagreeing FMS course rejects the borrowed RF arc");
        CHECK(ln3.note.find("RF arc lead-in") == std::string::npos,
              "rejected RF arc is not flown as a lead-in");
    }

    // --- VNAV: speed-limit (DECEL) handling ---
    // A descent profile that also hits a DECEL point at the same altitude
    // should soften the fpm requirement by the deceleration lead (1 NM / 10 KT).
    {
        Guidance g;
        RouteSnapshot r;
        r.valid = true;
        r.sourceRoute = true;   // CDU-declared route (speed constraints only come from CDU)
        r.wptCount = 2;
        r.legCount = 1;
        r.activeWpt = 1;
        r.activeLeg = 0;
        r.nextLeg = -1;
        // DECEL at lat=0, AAA behind at lat=-1 (60 NM) - irrelevant for distance calc
        r.wpts.resize(2);
        r.wpts[0].lat = -1.0; r.wpts[0].lon = 0.0; r.wpts[0].id = "AAA"; r.wpts[0].posValid = true;
        r.wpts[1].lat = 0.0;   r.wpts[1].lon = 0.0;
        r.wpts[1].id = "DECEL"; r.wpts[1].posValid = true;
        r.wpts[1].altFt = 8000; r.wpts[1].altValid = true;
        r.wpts[1].constraintSpeedKt = 250;
        r.legs.resize(1);
        r.legs[0] = RouteLeg{0, 0, 1, false};

        AircraftState s;
        s.valid = true; s.onGround = false; s.apEngaged = true; s.vnavMode = true;
        // 4 NM from DECEL (4 minutes of lat = 4 * 1 NM), above minDistance=3 NM
        s.lat = 0.066667; s.lon = 0.0;
        s.altFt = 10000.0; s.vsFpm = -500.0; s.vsAccelFpmPerSec = 0.0;
        s.gsKts = 250.0; s.iasKts = 350.0;  // 100 KT slowdown needed
        s.selectedAltFt = 0.0;
        Config c;
        c.vnavRespectSelectedAlt = true;

        VnavResult vn = g.computeVnav(s, r, c);
        CHECK(vn.active && vn.pathBased, "DECEL leg path-based VNAV active");
        CHECK(vn.targetSpdKt > 0.5 && vn.targetSpdKt < 251.0,
              "DECEL point found: speed limit 250 kt");
        CHECK(vn.targetSpdLeadNm > 9.5 && vn.targetSpdLeadNm < 10.5,
              "decel lead = 10 NM for 100 KT slowdown (1 NM / 10 KT)");
        // Nominal descent from 10000 to 8000 at 4 NM → rawRequired = -2082 fpm.
        // DECEL lead = 10 NM → softened = -2082 * (4 / (4+10)) ≈ -595 fpm.
        CHECK(vn.requiredVsFpm < -200.0 && vn.requiredVsFpm > -800.0,
              "DECEL lead softens the required descent fpm well below the nominal");
    }

    // Hard 250 KT / 10000 ft auto-enforcement: waypoint below 10000 ft with
    // speed above 270 KT triggers the FAR limit even with no CDU constraint.
    {
        Guidance g;
        RouteSnapshot r;
        r.valid = true;
        r.sourceRoute = true;
        r.wptCount = 2;
        r.legCount = 1;
        r.activeWpt = 1;
        r.activeLeg = 0;
        r.nextLeg = -1;
        r.wpts.resize(2);
        r.wpts[0].lat = -1.0; r.wpts[0].lon = 0.0; r.wpts[0].id = "AAA"; r.wpts[0].posValid = true;
        r.wpts[1].lat = 0.0;   r.wpts[1].lon = 0.0;
        r.wpts[1].id = "APPR1"; r.wpts[1].posValid = true;
        r.wpts[1].altFt = 8000; r.wpts[1].altValid = true;   // below 10000 ft
        // no constraintSpeedKt set - FAR 250/10000 must kick in automatically
        r.legs.resize(1);
        r.legs[0] = RouteLeg{0, 0, 1, false};

        AircraftState s;
        s.valid = true; s.onGround = false; s.apEngaged = true; s.vnavMode = true;
        s.lat = 0.05; s.lon = 0.0;         // 3 NM from APPR1
        s.altFt = 11000.0; s.vsFpm = -800.0; s.vsAccelFpmPerSec = 0.0;
        s.gsKts = 300.0; s.iasKts = 310.0;
        Config c;

        VnavResult vn = g.computeVnav(s, r, c);
        CHECK(vn.targetSpdKt > 0.5 && vn.targetSpdKt < 251.0,
              "auto FAR 250/10000 detected on a low-altitude waypoint");
        CHECK(vn.targetSpdLeadNm > 5.5 && vn.targetSpdLeadNm < 6.5,
              "decel lead = 6 NM for 60 KT slowdown");
    }

    // Route WITHOUT speed constraint (cruise waypoint) must NOT invent a limit.
    {
        Guidance g;
        RouteSnapshot r = MakeRoute();
        CdiInput cdi;
        AircraftState s = MakeState(0.0, 0.5, 90.0);
        s.altFt = 35000.0; s.iasKts = 320.0; s.gsKts = 490.0; s.vsFpm = 0.0;
        Config c;
        VnavResult vn = g.computeVnav(s, r, c);
        CHECK(vn.targetSpdKt < 0.5, "no speed limit invented for cruise");
    }

    // --- Long leg, small offset, high speed: no phantom intercept ---
    // Reproduces the reported swing: UU402 -> BOKIR (60 NM), aircraft 11 NM
    // along and ~1 NM right of the leg, 305 KT IAS / 400 KT GS.  Our computed
    // course differs from the FMS leg course by 1.2 deg, which over 49 NM is
    // a ~1 NM phantom cross-track.  Old logic turned that into a 21 deg left
    // intercept; the aircraft must now stay within a few degrees.
    {
        Guidance g;
        RouteSnapshot r;
        r.valid = true;
        r.sourceRoute = true;
        r.wptCount = 2;
        r.legCount = 1;
        r.activeWpt = 1;
        r.activeLeg = 0;
        r.nextLeg = -1;
        r.wpts.resize(2);
        r.wpts[0].lat = 30.753416; r.wpts[0].lon = 104.135750; r.wpts[0].id = "UU402";
        r.wpts[1].lat = 31.767778; r.wpts[1].lon = 104.349724; r.wpts[1].id = "BOKIR";
        r.wpts[0].posValid = true; r.wpts[1].posValid = true;
        r.legs.resize(1);
        r.legs[0] = RouteLeg{0, 0, 1, false};

        // 11 NM along the leg, 1 NM right of it.
        const double legCrs = bearingTrue(r.wpts[0].lat, r.wpts[0].lon,
                                          r.wpts[1].lat, r.wpts[1].lon);
        double lat = 0.0, lon = 0.0;
        destinationPoint(r.wpts[0].lat, r.wpts[0].lon, legCrs, 11.0, lat, lon);
        double offLat = 0.0, offLon = 0.0;
        destinationPoint(lat, lon, normalize360(legCrs + 90.0), 1.0, offLat, offLon);

        AircraftState s = MakeState(offLat, offLon, legCrs);
        s.magVarDeg = 0.0;
        s.gsKts = 400.0;
        s.iasKts = 305.0;

        // FMS reports the same leg 1.2 deg off our computed course.
        CdiInput cdi;
        cdi.valid = true;
        cdi.courseMagDeg = normalize360(legCrs - 1.2);
        cdi.distanceNm = 60.3;
        cdi.xtkNm = 0.03;
        cdi.xtkValid = true;

        LnavResult ln = g.computeLnav(s, r, cdi, cfg);
        CHECK(ln.active && ln.source == "ROUTE", "long leg route LNAV active");
        CHECK(std::abs(ln.xtkNm) < 0.3,
              "long leg uses the FMS cross-track directly");
        CHECK(std::abs(ln.correctionDeg) < 5.0,
              "no phantom intercept: correction stays under 5 deg");
        CHECK(ln.note.find("course from FMS") != std::string::npos,
              "note reports the FMS course source");
    }

    // 130 NM leg: the FMS CDI course is the local great-circle track at the
    // aircraft while the route course is the leg's initial bearing.  Rebuilding
    // the line from the local course rotated the leg by its convergence and
    // produced a persistent -1.36 NM offset in ROUTE mode while the CDI stayed
    // at ~0.05 NM (live LH -> GYA).  The FMS cross-track must be used directly.
    {
        Guidance g;
        RouteSnapshot r;
        r.valid = true;
        r.sourceRoute = true;
        r.wptCount = 2;
        r.legCount = 1;
        r.activeWpt = 1;
        r.activeLeg = 0;
        r.nextLeg = -1;
        r.wpts.resize(2);
        r.wpts[0].lat = 21.135000; r.wpts[0].lon = 110.333333; r.wpts[0].id = "LH";
        r.wpts[1].lat = 23.070000; r.wpts[1].lon = 112.486667; r.wpts[1].id = "GYA";
        r.wpts[0].posValid = true; r.wpts[1].posValid = true;
        r.legs.resize(1);
        r.legs[0] = RouteLeg{0, 0, 1, false};

        const double legCrs = bearingTrue(r.wpts[0].lat, r.wpts[0].lon,
                                          r.wpts[1].lat, r.wpts[1].lon);
        double lat = 0.0, lon = 0.0;
        destinationPoint(r.wpts[0].lat, r.wpts[0].lon, legCrs, 0.5, lat, lon);
        AircraftState s = MakeState(lat, lon, legCrs);
        s.magVarDeg = 0.0;

        CdiInput cdi;
        cdi.valid = true;
        cdi.courseMagDeg = normalize360(legCrs - 0.5);   // local vs initial
        cdi.distanceNm = 128.9;
        cdi.xtkNm = 0.05;
        cdi.xtkValid = true;

        LnavResult ln = g.computeLnav(s, r, cdi, cfg);
        CHECK(ln.active && ln.source == "ROUTE", "130 NM leg route active");
        CHECK(std::abs(ln.xtkNm) < 0.1,
              "130 NM leg uses the FMS cross-track, not a rebuilt line");
        CHECK(std::abs(ln.correctionDeg) < 1.0,
              "130 NM leg phantom offset does not command a turn");
        CHECK(ln.note.find("course from FMS") != std::string::npos,
              "130 NM leg reports the FMS course source");
    }

    // Without a CDI course the geometry still must not swing: a 1.26 NM
    // offset 49 NM from the fix is a ~1.5 deg correction, not 21 deg.
    {
        Guidance g;
        RouteSnapshot r;
        r.valid = true;
        r.sourceRoute = true;
        r.wptCount = 2;
        r.legCount = 1;
        r.activeWpt = 1;
        r.activeLeg = 0;
        r.nextLeg = -1;
        r.wpts.resize(2);
        r.wpts[0].lat = 30.753416; r.wpts[0].lon = 104.135750; r.wpts[0].id = "UU402";
        r.wpts[1].lat = 31.767778; r.wpts[1].lon = 104.349724; r.wpts[1].id = "BOKIR";
        r.wpts[0].posValid = true; r.wpts[1].posValid = true;
        r.legs.resize(1);
        r.legs[0] = RouteLeg{0, 0, 1, false};

        const double legCrs = bearingTrue(r.wpts[0].lat, r.wpts[0].lon,
                                          r.wpts[1].lat, r.wpts[1].lon);
        double lat = 0.0, lon = 0.0;
        destinationPoint(r.wpts[0].lat, r.wpts[0].lon, legCrs, 11.0, lat, lon);
        double offLat = 0.0, offLon = 0.0;
        destinationPoint(lat, lon, normalize360(legCrs + 90.0), 1.26, offLat, offLon);

        AircraftState s = MakeState(offLat, offLon, legCrs);
        s.gsKts = 400.0;
        s.iasKts = 305.0;

        LnavResult ln = g.computeLnav(s, r, CdiInput{}, cfg);
        CHECK(std::abs(ln.correctionDeg) < 5.0,
              "geometry-only homing: 1.26 NM offset at 49 NM commands < 5 deg");
        CHECK(std::abs(ln.xtkNm - 1.26) < 0.15,
              "cross-track still reports the true 1.26 NM offset");
    }

    // A genuine large offset still gets the Boeing capture: 4 NM off with
    // 10 NM to run must command close to the 30 deg limit.
    {
        Guidance g;
        RouteSnapshot r;
        r.valid = true;
        r.sourceRoute = true;
        r.wptCount = 2;
        r.legCount = 1;
        r.activeWpt = 1;
        r.activeLeg = 0;
        r.nextLeg = -1;
        r.wpts.resize(2);
        r.wpts[0].lat = 0.0; r.wpts[0].lon = 0.0; r.wpts[0].id = "AAA";
        r.wpts[1].lat = 0.166667; r.wpts[1].lon = 0.0; r.wpts[1].id = "BBB";
        r.wpts[0].posValid = true; r.wpts[1].posValid = true;
        r.legs.resize(1);
        r.legs[0] = RouteLeg{0, 0, 1, false};

        // Place the aircraft 10 NM from the fix, 4 NM right of the line.
        double lat = 0.0, lon = 0.0;
        destinationPoint(r.wpts[1].lat, r.wpts[1].lon, 180.0, 10.0, lat, lon);
        double oLat = 0.0, oLon = 0.0;
        destinationPoint(lat, lon, 90.0, 4.0, oLat, oLon);
        AircraftState s = MakeState(oLat, oLon, 0.0);
        s.gsKts = 250.0;

        LnavResult ln = g.computeLnav(s, r, CdiInput{}, cfg);
        CHECK(ln.xtkNm > 3.8 && ln.xtkNm < 4.2, "4 NM offset measured");
        CHECK(std::abs(ln.correctionDeg) > 20.0,
              "large offset still commands the Boeing capture angle");
    }

    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
