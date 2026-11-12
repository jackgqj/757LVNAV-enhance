// Reproduce the HC306 fly-by state from the live log.
#include "config.h"
#include "geo.h"
#include "guidance.h"

#include <cmath>
#include <cstdio>

using namespace lvnv;

static RouteSnapshot MakeRoute() {
    RouteSnapshot r;
    r.valid = true;
    r.sourceRoute = true;
    r.wptCount = 5;
    r.legCount = 5;
    r.wpts.resize(5);
    const double lat[5] = {29.996111, 30.125834, 30.089722, 30.023056, 30.088612};
    const double lon[5] = {120.275002, 120.566391, 120.669724, 120.858055, 120.960281};
    const char* ids[5] = {"HC307", "HC306", "HC606", "SUPAR", "OKTUG"};
    for (int i = 0; i < 5; ++i) {
        r.wpts[i].lat = lat[i];
        r.wpts[i].lon = lon[i];
        r.wpts[i].id = ids[i];
        r.wpts[i].posValid = true;
    }
    r.legs.resize(5);
    r.legs[0] = RouteLeg{0, 0, 0, false};
    for (int i = 1; i < 5; ++i) r.legs[i] = RouteLeg{0, i - 1, i, false};
    r.activeWpt = 1;
    r.activeLeg = 1;
    r.nextLeg = 2;
    r.activeId = "HC306";
    r.nextId = "HC606";
    return r;
}

int main() {
    Config cfg;
    Guidance guidance;
    const RouteSnapshot route = MakeRoute();
    const double inbound = bearingTrue(route.wpts[0].lat, route.wpts[0].lon,
                                       route.wpts[1].lat, route.wpts[1].lon);
    for (double remain : {3.2, 2.6, 2.0, 1.4, 0.8, 0.3}) {
        AircraftState s;
        s.valid = true;
        s.onGround = false;
        s.apEngaged = true;
        s.lnavModeDetected = true;
        s.gsKts = 315.0;
        s.altFt = 20000.0;
        s.magVarDeg = 0.0;
        s.trackTrueDeg = 69.6;
        destinationPoint(route.wpts[1].lat, route.wpts[1].lon,
                         normalize360(inbound + 180.0), remain, s.lat, s.lon);
        CdiInput cdi;
        LnavResult ln = guidance.computeLnav(s, route, cdi, cfg);
        std::printf("remain=%.1f desired=%.1f turn=%.0f%% turnBank=%d %.1f remainDBG=%.2f tangent=%.2f arc=%.1f zone=%d note=%s\n",
                    remain, ln.desiredCourseMagDeg, ln.turnProgress * 100.0,
                    ln.turnBankValid ? 1 : 0, ln.turnBankDeg,
                    ln.turnRemainNm, ln.turnTangentNm, ln.turnArcDeg,
                    ln.turnZone, ln.note.c_str());
    }
    return 0;
}
