#include "cifp_parser.h"

#include "geo.h"
#include "nav_database.h"
#include "route_model.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <utility>

namespace lvnv {

namespace {

std::string Trim(const std::string& value) {
    size_t b = 0;
    size_t e = value.size();
    while (b < e && std::isspace(static_cast<unsigned char>(value[b]))) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(value[e - 1]))) --e;
    return value.substr(b, e - b);
}

std::string Upper(const std::string& value) {
    std::string out = value;
    for (char& c : out) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return out;
}

std::vector<std::string> SplitComma(const std::string& line) {
    std::vector<std::string> parts;
    size_t start = 0;
    while (true) {
        const size_t comma = line.find(',', start);
        if (comma == std::string::npos) {
            parts.push_back(line.substr(start));
            break;
        }
        parts.push_back(line.substr(start, comma - start));
        start = comma + 1;
    }
    return parts;
}

} // namespace

std::vector<RfArcRow> CifpDatabase::ParseCifpLines(const std::vector<std::string>& lines) {
    std::vector<RfArcRow> arcs;
    std::string blockKey;
    std::string prevExit;

    for (const std::string& line : lines) {
        const std::vector<std::string> parts = SplitComma(line);
        if (parts.size() < 12) {
            blockKey.clear();
            prevExit.clear();
            continue;
        }

        const std::string section = Upper(Trim(parts[0].substr(0, parts[0].find(':'))));
        if (section != "SID" && section != "STAR" && section != "APPCH") {
            blockKey.clear();
            prevExit.clear();
            continue;
        }

        // A transition block is a run of rows sharing the section, procedure
        // id and transition entry fix.  The RF entry fix is the previous
        // row's fix inside that block.
        const std::string key =
            section + "|" + Upper(Trim(parts[2])) + "|" + Upper(Trim(parts[3]));
        if (key != blockKey) {
            blockKey = key;
            prevExit.clear();
        }

        if (Trim(parts[11]) == "RF" && parts.size() > 30 && !prevExit.empty()) {
            RfArcRow arc;
            arc.entryIdent = Upper(Trim(prevExit));
            arc.exitIdent = Upper(Trim(parts[4]));
            const std::string dir = Upper(Trim(parts[9]));
            arc.turn = (dir == "L") ? -1 : (dir == "R") ? 1 : 0;
            arc.radiusNm = std::atoi(Trim(parts[17]).c_str()) / 1000.0;
            arc.centerIdent = Upper(Trim(parts[30]));

            if (arc.turn != 0 && arc.radiusNm >= 0.3 && !arc.entryIdent.empty() &&
                !arc.exitIdent.empty() && !arc.centerIdent.empty()) {
                // The same arc appears once per procedure that uses it; the
                // geometry is identical, so keep the first copy.
                bool dup = false;
                for (const RfArcRow& existing : arcs) {
                    if (existing.entryIdent == arc.entryIdent &&
                        existing.exitIdent == arc.exitIdent) {
                        dup = true;
                        break;
                    }
                }
                if (!dup) arcs.push_back(arc);
            }
        }
        prevExit = Trim(parts[4]);
    }
    return arcs;
}

void CifpDatabase::loadAirports() {
    if (airportsLoaded_) return;
    airportsLoaded_ = true;

    std::ifstream file(root_ + "Custom Data/STSFF/navdata/Airports.txt");
    if (!file.is_open()) return;
    std::string line;
    while (std::getline(file, line)) {
        const std::vector<std::string> parts = SplitComma(line);
        if (parts.size() < 5) continue;
        if (Trim(parts[0]) != "A") continue;
        AirportFix fix;
        fix.ident = Upper(Trim(parts[1]));
        fix.lat = std::strtod(Trim(parts[3]).c_str(), nullptr);
        fix.lon = std::strtod(Trim(parts[4]).c_str(), nullptr);
        if (fix.ident.empty()) continue;
        if (!(fix.lat >= -90.0 && fix.lat <= 90.0 && fix.lon >= -180.0 && fix.lon <= 180.0))
            continue;
        airports_.push_back(fix);
    }
}

const std::vector<RfArcRow>& CifpDatabase::arcsForAirport(const std::string& icao) {
    auto it = cache_.find(icao);
    if (it != cache_.end()) return it->second;

    std::vector<RfArcRow> arcs;
    // A user-supplied CIFP file overrides the default database.
    std::ifstream file(root_ + "Custom Data/CIFP/" + icao + ".dat");
    if (!file.is_open()) {
        file.open(root_ + "Resources/default data/CIFP/" + icao + ".dat");
    }
    if (file.is_open()) {
        std::vector<std::string> lines;
        std::string line;
        while (std::getline(file, line)) lines.push_back(line);
        arcs = ParseCifpLines(lines);
    }
    auto res = cache_.emplace(icao, std::move(arcs));
    return res.first->second;
}

bool CifpDatabase::findRfArc(const std::string& entryIdent,
                             const std::string& exitIdent, double anchorLat,
                             double anchorLon, RfArcRow& out) {
    if (entryIdent.empty() || exitIdent.empty()) return false;
    if (!(anchorLat >= -90.0 && anchorLat <= 90.0 && anchorLon >= -180.0 &&
          anchorLon <= 180.0)) {
        return false;
    }
    loadAirports();
    if (airports_.empty()) return false;

    const std::string entry = Upper(entryIdent);
    const std::string exit = Upper(exitIdent);

    // One lookup per fix pair per session.  Without this the same "no RF arc
    // here" question is re-asked for every leg on every route rebuild.
    const std::string pairKey = entry + "|" + exit;
    auto negative = negative_.find(pairKey);
    if (negative != negative_.end()) return false;

    // Nearest airports first; an RF pair belongs to the procedure database of
    // the airport it terminates at, which is not always the closest one.
    // Select the six nearest in a single pass instead of sorting the whole
    // airport list on every call.
    std::vector<std::pair<double, size_t>> near;
    near.reserve(6);
    for (size_t i = 0; i < airports_.size(); ++i) {
        const double d =
            distanceNm(anchorLat, anchorLon, airports_[i].lat, airports_[i].lon);
        if (d > 120.0) continue;
        if (near.size() == 6 && d >= near.back().first) continue;
        near.emplace_back(d, i);
        std::sort(near.begin(), near.end());
        if (near.size() > 6) near.pop_back();
    }

    for (const auto& candidate : near) {
        const std::vector<RfArcRow>& arcs =
            arcsForAirport(airports_[candidate.second].ident);
        for (const RfArcRow& arc : arcs) {
            if (arc.entryIdent == entry && arc.exitIdent == exit) {
                out = arc;
                return true;
            }
        }
    }
    if (negative_.size() < 4096) negative_.insert(pairKey);
    return false;
}

void AnnotateRfArcs(RouteSnapshot& route, CifpDatabase& cifp, NavDatabase& nav) {
    for (size_t li = 0; li < route.legs.size(); ++li) {
        RouteLeg& leg = route.legs[li];
        if (leg.rfArc) continue;

        const RouteWaypoint* exitW = route.waypoint(leg.lastWpt);
        if (!exitW || !exitW->posValid || exitW->id.empty()) continue;

        RfArcRow arc;
        int entryIdx = -1;
        bool found = false;
        for (int ci = leg.firstWpt; ci >= leg.firstWpt - 2 && ci >= 0; --ci) {
            const RouteWaypoint* entryW = route.waypoint(ci);
            if (!entryW || !entryW->posValid || entryW->id.empty() ||
                entryW->id == exitW->id) {
                continue;
            }
            if (cifp.findRfArc(entryW->id, exitW->id, exitW->lat, exitW->lon, arc)) {
                entryIdx = ci;
                found = true;
                break;
            }
        }
        if (!found) continue;

        const RouteWaypoint* entryW = route.waypoint(entryIdx);

        double centerLat = 0.0;
        double centerLon = 0.0;
        bool haveCenter = false;

        std::vector<NavFix> fix;
        if (nav.resolve(std::vector<std::string>{arc.centerIdent}, exitW->lat,
                        exitW->lon, fix) &&
            !fix.empty() && fix[0].valid) {
            // Sanity: the centre must sit one radius away from both ends.
            // This rejects a same-named fix from a different airport.
            const double dExit =
                distanceNm(fix[0].lat, fix[0].lon, exitW->lat, exitW->lon);
            const double dEntry =
                distanceNm(fix[0].lat, fix[0].lon, entryW->lat, entryW->lon);
            if (dExit > 0.5 * arc.radiusNm && dExit < 1.5 * arc.radiusNm &&
                dEntry > 0.5 * arc.radiusNm && dEntry < 1.5 * arc.radiusNm) {
                centerLat = fix[0].lat;
                centerLon = fix[0].lon;
                haveCenter = true;
            }
        }

        if (!haveCenter) {
            // Tangent continuity: the centre is R to the side of the inbound
            // course at the entry fix.
            const RouteWaypoint* prevW = route.waypoint(entryIdx - 1);
            if (!prevW || !prevW->posValid) continue;
            const double inbound =
                bearingTrue(prevW->lat, prevW->lon, entryW->lat, entryW->lon);
            destinationPoint(entryW->lat, entryW->lon,
                             normalize360(inbound + arc.turn * 90.0),
                             arc.radiusNm, centerLat, centerLon);
        }

        const double radialStart =
            bearingTrue(centerLat, centerLon, entryW->lat, entryW->lon);
        const double radialEnd =
            bearingTrue(centerLat, centerLon, exitW->lat, exitW->lon);
        double total = normalize180(arc.turn * (radialEnd - radialStart));
        if (total < 0.0) total += 360.0;
        if (total < 5.0 || total > 355.0) continue;

        // Geometric consistency: the arc cannot be shorter than the chord.
        const double chord =
            distanceNm(entryW->lat, entryW->lon, exitW->lat, exitW->lon);
        if (arc.radiusNm * deg2rad(total) < chord * 0.98) continue;

        leg.rfArc = true;
        leg.rfRadiusNm = arc.radiusNm;
        leg.rfCenterLat = centerLat;
        leg.rfCenterLon = centerLon;
        leg.rfTurn = arc.turn;
        leg.rfRadialStartDeg = radialStart;
        leg.rfRadialEndDeg = radialEnd;
        leg.rfTotalSweptDeg = total;
        leg.rfEntryWpt = entryIdx;
    }
}

} // namespace lvnv
