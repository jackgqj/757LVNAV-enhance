#pragma once

#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "nav_database.h"

namespace lvnv {

// One leg of a CIFP departure procedure.  ARINC 424 owns the sequence and
// path terminator; coordinates and RF centre coordinates are resolved from
// the nav database because CIFP rows do not carry lat/lon.
struct ProcedureLeg {
    std::string type;        // IF, TF, RF, DF, ...
    std::string fix;         // fix / exit ident
    int sequence = 0;        // SID:nnn order
    double lat = 0.0;
    double lon = 0.0;
    bool posValid = false;

    bool rf = false;
    std::string rfCenter;    // CIFP centre ident
    double rfCenterLat = 0.0;
    double rfCenterLon = 0.0;
    bool rfCenterValid = false;
    double rfRadiusNm = 0.0;
    int rfTurn = 0;          // +1 right (clockwise), -1 left
};

struct Procedure {
    std::string airport;
    std::string kind;        // SID
    std::string ident;
    std::string transition;  // normalized: "10L" / "DUMIX"
    std::vector<ProcedureLeg> legs;
};

struct ResolveStats {
    int legs = 0;
    int resolved = 0;
    int rf = 0;
    int rfCenters = 0;
};

// Loads one airport's SID sequence from X-Plane's ARINC 424 CIFP file and
// resolves every fix through the nav database (FF757 navdata + earth_fix.dat).
// This is the CIFP-only alternative to the mixed GNS430 experiment.
class CifpProcedureDatabase {
public:
    void setRootPath(const std::string& xplaneRoot) {
        root_ = xplaneRoot;
        if (!root_.empty() && root_.back() != '/' && root_.back() != '\\') {
            root_ += '/';
        }
        nav_.setRootPath(root_);
    }

    bool loadAirport(const std::string& icao);
    bool buildProcedure(const std::string& icao, const std::string& ident,
                        const std::string& transition, Procedure& out,
                        ResolveStats* stats = nullptr);
    const std::vector<Procedure>& procedures(const std::string& icao) const;

    // Nearest airport within maxNm, used to pick the departure airport from
    // the aircraft position.  Reads Airports.txt once.
    bool nearestAirport(double lat, double lon, double maxNm, std::string& icao,
                        double& aptLat, double& aptLon);

    // CIFP writes runway transitions as "RW10L"; the FMS/route code uses
    // "10L".  Normalize both to the latter.
    static std::string NormalizeTransition(const std::string& transition);

    // Pure parser, exposed for unit tests.
    static std::vector<Procedure> ParseCifpSidLines(
        const std::vector<std::string>& lines, const std::string& airport);

    // Resolve coordinates with an injectable fix resolver.  The plugin wraps
    // NavDatabase; unit tests use a small map.
    using FixResolver = std::function<bool(const std::string&, double, double,
                                           double&, double&)>;
    static void ResolveProcedure(Procedure& procedure,
                                 const FixResolver& resolver,
                                 double airportLat, double airportLon,
                                 ResolveStats* stats = nullptr);

private:
    bool airportLatLon(const std::string& icao, double& lat, double& lon) const;

    std::string root_;
    NavDatabase nav_;
    std::map<std::string, std::vector<Procedure>> procedures_;
    std::set<std::string> loaded_;
    struct AirportRef {
        std::string icao;
        double lat = 0.0;
        double lon = 0.0;
    };
    std::vector<AirportRef> airports_;
    bool airportsLoaded_ = false;
};

} // namespace lvnv
