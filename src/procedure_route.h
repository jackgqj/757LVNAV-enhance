#pragma once

#include "procedure_parser.h"
#include "route_model.h"

#include <string>

namespace lvnv {

struct ProcedureSelection {
    bool valid = false;
    std::string airport;
    std::string ident;
    std::string transition;
    int score = 0;
    double airportLat = 0.0;
    double airportLon = 0.0;
};

struct ProcedureRouteOptions {
    double maxAirportNm = 60.0;
    int lookahead = 40;   // route idents compared against the SID sequence
    int minMatches = 2;   // a 2-fix suffix is enough when the route starts late
};

// Turns a resolved CIFP departure into a RouteSnapshot and merges it with the
// FMS/CDU route.  The FMS route stays authoritative after the last common
// departure fix; the procedure supplies the earlier fixes plus RF geometry.
class ProcedureRouteBuilder {
public:
    explicit ProcedureRouteBuilder(CifpProcedureDatabase* db = nullptr)
        : db_(db) {}
    void setDatabase(CifpProcedureDatabase* db) { db_ = db; }

    // Detect the active SID by matching the route's first idents against every
    // CIFP SID transition at the nearest airport.
    bool detect(const RouteSnapshot& fms, double lat, double lon,
                ProcedureSelection& out,
                const ProcedureRouteOptions& options = ProcedureRouteOptions());

    bool build(const ProcedureSelection& selection, RouteSnapshot& out,
               ResolveStats* stats = nullptr);

    // Merge the procedure prefix with the FMS route.  Returns false when the
    // two routes share no fix (do not invent a join).
    static bool merge(const RouteSnapshot& fms, const RouteSnapshot& procedure,
                      RouteSnapshot& out);

    // Convert a resolved procedure to a route.  Exposed for unit tests.
    static bool BuildRoute(const Procedure& procedure, RouteSnapshot& out);

private:
    CifpProcedureDatabase* db_ = nullptr;
};

} // namespace lvnv
