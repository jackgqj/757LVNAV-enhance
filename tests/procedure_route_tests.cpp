#include "procedure_route.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace lvnv;

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond, msg) do { \
    ++g_checks; \
    if (!(cond)) { \
        ++g_failures; \
        std::printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); \
    } \
} while (0)

#define CHECK_NEAR(a, b, eps, msg) CHECK(std::abs((a) - (b)) <= (eps), msg)

static Procedure MakeStraightProcedure() {
    Procedure p;
    p.airport = "TEST";
    p.kind = "SID";
    p.ident = "TEST1";
    p.transition = "10L";
    ProcedureLeg a;
    a.type = "IF";
    a.fix = "AAA";
    a.sequence = 10;
    a.lat = 0.0;
    a.lon = 0.0;
    a.posValid = true;
    ProcedureLeg b;
    b.type = "TF";
    b.fix = "BBB";
    b.sequence = 20;
    b.lat = 0.0;
    b.lon = 1.0;
    b.posValid = true;
    ProcedureLeg c;
    c.type = "TF";
    c.fix = "CCC";
    c.sequence = 30;
    c.lat = 0.5;
    c.lon = 1.0;
    c.posValid = true;
    p.legs = {a, b, c};
    return p;
}

static RouteSnapshot MakeFmsRoute() {
    RouteSnapshot r;
    r.valid = true;
    r.sourceRoute = true;
    r.wptCount = 4;
    r.legCount = 4;
    r.wpts.resize(4);
    r.wpts[0].id = "BBB";
    r.wpts[0].lat = 0.0;
    r.wpts[0].lon = 1.0;
    r.wpts[1].id = "CCC";
    r.wpts[1].lat = 0.5;
    r.wpts[1].lon = 1.0;
    r.wpts[2].id = "DDD";
    r.wpts[2].lat = 1.0;
    r.wpts[2].lon = 1.0;
    r.wpts[3].id = "EEE";
    r.wpts[3].lat = 1.5;
    r.wpts[3].lon = 1.0;
    for (auto& w : r.wpts) w.posValid = true;
    r.legs.resize(4);
    for (int i = 0; i < 4; ++i) {
        r.legs[static_cast<size_t>(i)] = RouteLeg{0, i == 0 ? 0 : i - 1, i == 0 ? 0 : i, false};
    }
    r.activeWpt = 2;   // DDD in the FMS route
    r.activeLeg = 2;
    r.nextLeg = 3;
    r.activeId = "DDD";
    r.nextId = "EEE";
    return r;
}

int main() {
    const Procedure straight = MakeStraightProcedure();
    RouteSnapshot procRoute;
    CHECK(ProcedureRouteBuilder::BuildRoute(straight, procRoute),
          "straight procedure converts to a route");
    CHECK(procRoute.valid && procRoute.wpts.size() == 3,
          "procedure route has three waypoints");
    CHECK(procRoute.legs.size() == 3 && procRoute.legs[0].lastWpt == 0,
          "CDU-style leg 0 is one point");
    CHECK(procRoute.legs[2].firstWpt == 1 && procRoute.legs[2].lastWpt == 2,
          "procedure legs connect consecutive fixes");
    CHECK(procRoute.procedureAssisted && procRoute.procedureId == "TEST1 10L",
          "procedure source is recorded");

    RouteSnapshot fms = MakeFmsRoute();
    RouteSnapshot merged;
    CHECK(ProcedureRouteBuilder::merge(fms, procRoute, merged),
          "procedure prefix merges with the FMS route");
    CHECK(merged.wpts.size() == 4,
          "merged route starts at the first common fix (BBB,CCC,DDD,EEE)");
    CHECK(merged.wpts.front().id == "BBB" && merged.wpts.back().id == "EEE",
          "merged route spans first common fix to FMS end");
    CHECK(merged.activeWpt == 2 && merged.activeId == "DDD",
          "merged active fix maps back to DDD");
    CHECK(merged.procedureAssisted,
          "merged route is marked procedure-assisted");

    // RF arc geometry: a 90 deg right arc from (1,1) to (0,2) around (0,1),
    // radius one degree of latitude (60 NM).
    Procedure arc;
    arc.airport = "TEST";
    arc.kind = "SID";
    arc.ident = "ARC1";
    arc.transition = "10L";
    ProcedureLeg entry;
    entry.type = "TF";
    entry.fix = "ENTRY";
    entry.sequence = 10;
    entry.lat = 1.0;
    entry.lon = 1.0;
    entry.posValid = true;
    ProcedureLeg exit;
    exit.type = "RF";
    exit.fix = "EXIT";
    exit.sequence = 20;
    exit.lat = 0.0;
    exit.lon = 2.0;
    exit.posValid = true;
    exit.rf = true;
    exit.rfCenter = "CTR";
    exit.rfCenterLat = 0.0;
    exit.rfCenterLon = 1.0;
    exit.rfCenterValid = true;
    exit.rfRadiusNm = 60.0;
    exit.rfTurn = 1;
    arc.legs = {entry, exit};
    RouteSnapshot arcRoute;
    CHECK(ProcedureRouteBuilder::BuildRoute(arc, arcRoute),
          "RF procedure converts to a route");
    CHECK(arcRoute.legs.size() == 2 && arcRoute.legs[1].rfArc,
          "RF leg is annotated");
    CHECK_NEAR(arcRoute.legs[1].rfTotalSweptDeg, 90.0, 2.0,
               "RF arc sweeps 90 degrees");
    CHECK_NEAR(arcRoute.legs[1].rfRadiusNm, 60.0, 1e-9,
               "RF radius preserved");
    CHECK(arcRoute.legs[1].rfEntryValid && arcRoute.legs[1].rfEntryWpt == 0,
          "RF entry fix is the previous waypoint");

    // No common fix: do not invent a join.
    RouteSnapshot unrelated;
    unrelated.valid = true;
    unrelated.sourceRoute = true;
    unrelated.wptCount = 2;
    unrelated.legCount = 2;
    unrelated.wpts.resize(2);
    unrelated.wpts[0].id = "XXX";
    unrelated.wpts[0].lat = 10.0;
    unrelated.wpts[0].lon = 10.0;
    unrelated.wpts[1].id = "YYY";
    unrelated.wpts[1].lat = 11.0;
    unrelated.wpts[1].lon = 11.0;
    unrelated.wpts[0].posValid = true;
    unrelated.wpts[1].posValid = true;
    unrelated.legs.resize(2);
    unrelated.legs[0] = RouteLeg{0, 0, 0, false};
    unrelated.legs[1] = RouteLeg{0, 0, 1, false};
    RouteSnapshot noJoin;
    CHECK(!ProcedureRouteBuilder::merge(unrelated, procRoute, noJoin),
          "unrelated routes are not merged");

    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
