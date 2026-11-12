// Diagnose SID detection for an arbitrary route ident list.
// usage: procedure_detect_probe <xplane-root> <lat> <lon> <ID> [ID ...]
#include "procedure_route.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace lvnv;

int main(int argc, char** argv) {
    if (argc < 5) {
        std::printf("usage: %s <root> <lat> <lon> <ID> [ID...]\n", argv[0]);
        return 2;
    }
    RouteSnapshot r;
    r.valid = true;
    r.sourceRoute = true;
    for (int i = 4; i < argc; ++i) {
        RouteWaypoint w;
        w.id = argv[i];
        w.lat = 0.0;
        w.lon = 0.0;
        w.posValid = true;
        r.wpts.push_back(w);
    }
    r.wptCount = static_cast<int>(r.wpts.size());
    r.legCount = r.wptCount;
    r.legs.resize(r.wpts.size());
    for (int i = 0; i < r.wptCount; ++i) {
        r.legs[static_cast<size_t>(i)] =
            RouteLeg{0, i == 0 ? 0 : i - 1, i == 0 ? 0 : i, false};
    }

    CifpProcedureDatabase db;
    db.setRootPath(argv[1]);
    ProcedureRouteBuilder builder(&db);
    std::string icao;
    double aptLat = 0.0;
    double aptLon = 0.0;
    if (db.nearestAirport(std::atof(argv[2]), std::atof(argv[3]), 60.0, icao,
                          aptLat, aptLon)) {
        std::printf("nearest=%s (%.4f,%.4f)\n", icao.c_str(), aptLat, aptLon);
    } else {
        std::printf("no nearest airport\n");
    }

    ProcedureSelection sel;
    if (builder.detect(r, std::atof(argv[2]), std::atof(argv[3]), sel)) {
        std::printf("detected=%s %s score=%d airport=%s\n", sel.ident.c_str(),
                    sel.transition.c_str(), sel.score, sel.airport.c_str());
        Procedure proc;
        ResolveStats stats;
        if (db.buildProcedure(sel.airport, sel.ident, sel.transition, proc,
                              &stats)) {
            std::printf("build legs=%zu resolved=%d rf=%d centres=%d\n",
                        proc.legs.size(), stats.resolved, stats.rf,
                        stats.rfCenters);
            RouteSnapshot procRoute;
            if (ProcedureRouteBuilder::BuildRoute(proc, procRoute)) {
                RouteSnapshot merged;
                if (ProcedureRouteBuilder::merge(r, procRoute, merged)) {
                    std::printf("merged wpts=%zu active=%d id=%s first=%s last=%s\n",
                                merged.wpts.size(), merged.activeWpt,
                                merged.activeId.c_str(), merged.wpts.front().id.c_str(),
                                merged.wpts.back().id.c_str());
                } else {
                    std::printf("merge failed\n");
                }
            }
        }
    } else {
        std::printf("detect failed\n");
    }
    return 0;
}
