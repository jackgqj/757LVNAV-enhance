// Standalone probe for the CIFP-only departure parser.  It is not part of the
// plugin; run it against a real X-Plane install:
//   procedure_probe.exe "E:\...\X-Plane 12" ZULS [SID [TRANSITION]]
#include "procedure_parser.h"

#include <cstdio>
#include <string>

using namespace lvnv;

int main(int argc, char** argv) {
    if (argc < 3) {
        std::printf("usage: %s <xplane-root> <ICAO> [SID [TRANSITION]]\n", argv[0]);
        return 2;
    }
    const std::string root = argv[1];
    const std::string icao = argv[2];
    CifpProcedureDatabase db;
    db.setRootPath(root);
    const bool loaded = db.loadAirport(icao);
    std::printf("%s: loaded=%d cifp=%zu\n", icao.c_str(), loaded ? 1 : 0,
                db.procedures(icao).size());
    if (!loaded) return 1;

    if (argc >= 4) {
        const std::string sid = argv[3];
        const std::string transition = (argc >= 5) ? argv[4] : "";
        Procedure proc;
        ResolveStats stats;
        if (!db.buildProcedure(icao, sid, transition, proc, &stats)) {
            std::printf("procedure %s %s not found\n", sid.c_str(),
                        transition.c_str());
            return 1;
        }
        std::printf("resolved %s %s: legs=%zu resolved=%d rf=%d centres=%d\n",
                    proc.ident.c_str(), proc.transition.c_str(),
                    proc.legs.size(), stats.resolved, stats.rf,
                    stats.rfCenters);
        for (const ProcedureLeg& leg : proc.legs) {
            std::printf("  %-2d %-3s %-6s lat=%10.6f lon=%11.6f pos=%d",
                        leg.sequence, leg.type.c_str(), leg.fix.c_str(), leg.lat,
                        leg.lon, leg.posValid ? 1 : 0);
            if (leg.rf) {
                std::printf(" centre=%s C=(%.6f,%.6f) R=%.1f turn=%+d",
                            leg.rfCenter.c_str(), leg.rfCenterLat,
                            leg.rfCenterLon, leg.rfRadiusNm, leg.rfTurn);
            }
            std::printf("\n");
        }
        return 0;
    }

    for (const Procedure& p : db.procedures(icao)) {
        std::printf("%-8s %-8s legs=%zu\n", p.ident.c_str(),
                    p.transition.c_str(), p.legs.size());
    }
    return 0;
}
