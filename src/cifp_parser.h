#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

namespace lvnv {

class NavDatabase;
struct RouteSnapshot;

// One RF (constant radius to fix) leg row from an X-Plane CIFP airport file.
// X-Plane stores ARINC 424 procedure legs as comma-separated rows in
//   Resources/default data/CIFP/<ICAO>.dat   (or Custom Data/CIFP overrides)
// Column layout (0-based, field 0 is "SECTION:seq"):
//   [2]  procedure id (e.g. R16CZ)      [3] transition entry fix
//   [4]  this row's fix (arc exit)      [9] turn direction L/R
//   [11] path terminator ("RF")         [17] arc radius in 0.001 NM
//   [30] arc centre fix ident
// The arc entry fix is the previous row's fix inside the same transition
// block (a block = same section + procedure + transition entry).
struct RfArcRow {
    std::string entryIdent;
    std::string exitIdent;
    std::string centerIdent;
    double radiusNm = 0.0;
    int turn = 0;   // +1 = right (clockwise), -1 = left
};

// Lazily loads per-airport CIFP files and answers "is there an RF arc from
// fix A to fix B near this position?".  The airport is found by proximity to
// the anchor (normally the exit fix) using the FF757 navdata airport list.
class CifpDatabase {
public:
    void setRootPath(const std::string& xplaneRoot) { root_ = xplaneRoot; }

    bool findRfArc(const std::string& entryIdent, const std::string& exitIdent,
                   double anchorLat, double anchorLon, RfArcRow& out);
    // Exit-only lookup: used when the entry fix is not in the route snapshot
    // (the CDU LEGS cache starts at the active waypoint).  Returns the first
    // arc that terminates at `exitIdent`.
    bool findRfArcByExit(const std::string& exitIdent, double anchorLat,
                         double anchorLon, RfArcRow& out);

    // Pure parser, exposed for unit tests.
    static std::vector<RfArcRow> ParseCifpLines(const std::vector<std::string>& lines);
    // Parser for the FF757's own procedure database
    // (Custom Data/STSFF/navdata/Proc/<ICAO>.txt).  Its RF rows carry the
    // direction the FMS itself uses (1 = left / 2 = right, i.e. the CDU's
    // ARC LEFT / ARC RIGHT), plus centre fix, course and radius, so it is
    // preferred over CIFP when both describe the same arc.
    static std::vector<RfArcRow> ParseFfProcLines(const std::vector<std::string>& lines);

private:
    struct AirportFix {
        std::string ident;
        double lat = 0.0;
        double lon = 0.0;
    };

    void loadAirports();
    const std::vector<RfArcRow>& arcsForAirport(const std::string& icao);
    const std::vector<RfArcRow>& ffArcsForAirport(const std::string& icao);
    std::vector<std::pair<double, size_t>> nearestAirports(double anchorLat,
                                                           double anchorLon);

    std::string root_;
    bool airportsLoaded_ = false;
    std::vector<AirportFix> airports_;
    std::map<std::string, std::vector<RfArcRow>> cache_;  // ICAO -> CIFP arcs
    std::map<std::string, std::vector<RfArcRow>> ffCache_;  // ICAO -> FF757 arcs
    std::set<std::string> negative_;                     // "ENTRY|EXIT" pairs with no RF arc
};

// Matches CIFP RF rows against every route leg and fills the leg's rfArc
// geometry.  Candidate entry fixes are the leg's first waypoint or up to two
// waypoints before it, because rteData complex legs may start with unnamed
// arc points.  The centre fix is resolved via the nav database (earth_fix.dat
// fallback included); when that fails the centre is reconstructed from the
// entry point, the inbound course and the radius (tangent continuity).
void AnnotateRfArcs(RouteSnapshot& route, CifpDatabase& cifp, NavDatabase& nav);

} // namespace lvnv
