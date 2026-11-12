// Exercise CIFP detection + merge against a real X-Plane install.  The FMS
// route is synthesized from the procedure's own fixes so the test focuses on
// the detector and joiner, not on a live CDU.
#include "procedure_route.h"

#include <algorithm>
#include <cstdio>
#include <string>

using namespace lvnv;

int main(int argc, char** argv) {
    if (argc < 5) {
        std::printf("usage: %s <xplane-root> <ICAO> <SID> <TRANSITION>\n", argv[0]);
        return 2;
    }
    CifpProcedureDatabase db;
    db.setRootPath(argv[1]);
    if (!db.loadAirport(argv[2])) {
        std::printf("airport load failed\n");
        return 1;
    }
    ProcedureRouteBuilder builder(&db);
    ProcedureSelection sel;
    ProcedureRouteOptions opts;
    if (!builder.detect(RouteSnapshot{}, 0, 0, sel, opts)) {
        std::printf("detect needs a route; using explicit selection\n");
        sel.valid = true;
        sel.airport = argv[2];
        sel.ident = argv[3];
        sel.transition = argv[4];
    }
    Procedure proc;
    ResolveStats stats;
    if (!db.buildProcedure(argv[2], argv[3], argv[4], proc, &stats)) {
        std::printf("build failed\n");
        return 1;
    }
    RouteSnapshot procedureRoute;
    if (!ProcedureRouteBuilder::BuildRoute(proc, procedureRoute)) {
        std::printf("route build failed\n");
        return 1;
    }
    std::printf("proc=%s %s legs=%zu resolved=%d rf=%d centres=%d\n",
                proc.ident.c_str(), proc.transition.c_str(), proc.legs.size(),
                stats.resolved, stats.rf, stats.rfCenters);

    // Synthesize an FMS route that starts at the second procedure fix and
    // keeps a few fixes after it.
    RouteSnapshot fms;
    fms.valid = true;
    fms.sourceRoute = true;
    const int start = 1;
    const int count = std::min(25, static_cast<int>(procedureRoute.wpts.size()) - start);
    fms.wpts.assign(procedureRoute.wpts.begin() + start,
                    procedureRoute.wpts.begin() + start + count);
    fms.wptCount = count;
    fms.legCount = count;
    fms.legs.resize(count);
    for (int i = 0; i < count; ++i) {
        fms.legs[static_cast<size_t>(i)] =
            RouteLeg{0, i == 0 ? 0 : i - 1, i == 0 ? 0 : i, false};
    }
    fms.activeWpt = 1;
    fms.activeLeg = 1;
    fms.nextLeg = 2;
    fms.activeId = fms.wpts[1].id;
    fms.nextId = fms.wpts[2].id;

    ProcedureSelection detected;
    const double anchorLat = proc.legs.size() > 1 ? proc.legs[1].lat : 0.0;
    const double anchorLon = proc.legs.size() > 1 ? proc.legs[1].lon : 0.0;
    if (builder.detect(fms, anchorLat, anchorLon, detected, opts)) {
        std::printf("detected=%s %s score=%d airport=%s\n",
                    detected.ident.c_str(), detected.transition.c_str(),
                    detected.score, detected.airport.c_str());
    } else {
        std::printf("detection failed\n");
    }
    Procedure detectedProc;
    if (!db.buildProcedure(argv[2], argv[3], argv[4], detectedProc, nullptr)) {
        return 1;
    }
    RouteSnapshot detectedRoute;
    ProcedureRouteBuilder::BuildRoute(detectedProc, detectedRoute);
    RouteSnapshot merged;
    if (ProcedureRouteBuilder::merge(fms, detectedRoute, merged)) {
        std::printf("merged wpts=%zu active=%d id=%s proc=%d\n",
                    merged.wpts.size(), merged.activeWpt,
                    merged.activeId.c_str(), merged.procedureAssisted ? 1 : 0);
    } else {
        std::printf("merge failed\n");
    }
    return 0;
}
