#include "cifp_parser.h"

#include "geo.h"
#include "nav_database.h"
#include "route_model.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <utility>

// ===================================================================
// RF (radius-to-fix) parsing / lookup — DO NOT MODIFY WITHOUT USER APPROVAL
// 如使用AI工具，AI在没有用户允许的情况下，不得擅自修改(必须询问用户)。
// ===================================================================

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

std::vector<RfArcRow> CifpDatabase::ParseFfProcLines(
    const std::vector<std::string>& lines) {
    // FF757 procedure rows are comma separated:
    //   RF,<fix>,<lat>,<lon>,<dir 1=L 2=R>,<centre fix>,<course>,<radius NM>,...
    // The direction field is the same value the CDU shows as ARC LEFT/RIGHT.
    std::vector<RfArcRow> arcs;
    std::string prevIdent;
    for (const std::string& line : lines) {
        const std::vector<std::string> p = SplitComma(line);
        // SplitComma never returns an empty vector: a blank line yields one
        // empty field.  A one-field row (blank line, "RF" alone, junk) has no
        // p[1], and an unconditional prevIdent assignment below reads past the
        // vector and crashes inside Trim (0xc0000005, cifp_parser.cpp Trim+0xa5,
        // win.xpl +0x2b445 on startup).
        //
        // PROTECTED GUARD - DO NOT REMOVE, WEAKEN, OR RESTRUCTURE THIS CHECK
        // WITHOUT EXPLICIT USER APPROVAL.  It was fixed in 1.2.22, lost during
        // a source rebuild, and crashed again in 1.2.25; every p[1] use below
        // depends on this early continue.
        if (p.size() < 2) { prevIdent.clear(); continue; }
        const std::string kind = Upper(Trim(p[0]));
        if (kind == "SID" || kind == "STAR" || kind == "APPCH" || kind == "AIRPORT") {
            prevIdent.clear();
            continue;
        }
        if (kind == "RF" && p.size() >= 8) {
            RfArcRow a;
            a.entryIdent = prevIdent;
            a.exitIdent = Upper(Trim(p[1]));
            const int dir = std::atoi(Trim(p[4]).c_str());
            a.turn = (dir == 1) ? -1 : (dir == 2) ? 1 : 0;
            a.centerIdent = Upper(Trim(p[5]));
            a.radiusNm = std::atof(Trim(p[7]).c_str());
            if (a.turn != 0 && a.radiusNm >= 0.3 && !a.exitIdent.empty() &&
                !a.centerIdent.empty()) {
                bool dup = false;
                for (const RfArcRow& e : arcs) {
                    if (e.entryIdent == a.entryIdent && e.exitIdent == a.exitIdent) {
                        dup = true;
                        break;
                    }
                }
                if (!dup) arcs.push_back(a);
            }
        }
        // Protected: reached only because p.size() >= 2 is enforced above.
        prevIdent = Upper(Trim(p[1]));
    }
    return arcs;
}

const std::vector<RfArcRow>& CifpDatabase::ffArcsForAirport(const std::string& icao) {
    auto it = ffCache_.find(icao);
    if (it != ffCache_.end()) return it->second;

    std::vector<RfArcRow> arcs;
    std::ifstream file(root_ + "Custom Data/STSFF/navdata/Proc/" + icao + ".txt");
    if (file.is_open()) {
        std::vector<std::string> lines;
        std::string line;
        while (std::getline(file, line)) lines.push_back(line);
        arcs = ParseFfProcLines(lines);
    }
    auto res = ffCache_.emplace(icao, std::move(arcs));
    return res.first->second;
}

std::vector<std::pair<double, size_t>> CifpDatabase::nearestAirports(
    double anchorLat, double anchorLon) {
    // Six nearest in a single pass instead of sorting the whole airport list
    // on every lookup.
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
    return near;
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
    if (negative_.count(pairKey) != 0) return false;

    // CIFP-only in this experimental branch.  The stable plugin preferred the
    // FF757 GNS430 Proc file when both described the arc; that mixing is
    // intentionally removed so the procedure sequence, RF geometry and
    // transitions all come from X-Plane's ARINC 424 database.
    const auto near = nearestAirports(anchorLat, anchorLon);
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

bool CifpDatabase::findRfArcByExit(const std::string& exitIdent,
                                   double anchorLat, double anchorLon,
                                   RfArcRow& out) {
    if (exitIdent.empty()) return false;
    if (!(anchorLat >= -90.0 && anchorLat <= 90.0 && anchorLon >= -180.0 &&
          anchorLon <= 180.0)) {
        return false;
    }
    loadAirports();
    if (airports_.empty()) return false;

    const std::string exit = Upper(exitIdent);
    const std::string key = "?|" + exit;
    if (negative_.count(key) != 0) return false;

    const auto near = nearestAirports(anchorLat, anchorLon);
    // Exit-only matching is weak: any airport within 120 NM can offer an arc
    // whose exit ident happens to match.  A live case borrowed the ZULS RF
    // arc LS987 -> (INTC) onto an enroute fix 67 NM from that airport and
    // dragged LNAV 12.8 NM off course.  An RF arc that belongs to the active
    // leg terminates near its procedure airport, so restrict the search to a
    // plausible terminal-area radius.
    constexpr double kMaxProcedureAirportNm = 40.0;
    for (const auto& candidate : near) {
        if (candidate.first > kMaxProcedureAirportNm) continue;
        const std::vector<RfArcRow>& arcs =
            arcsForAirport(airports_[candidate.second].ident);
        for (const RfArcRow& arc : arcs) {
            if (arc.exitIdent == exit) {
                out = arc;
                return true;
            }
        }
    }
    if (negative_.size() < 4096) negative_.insert(key);
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
        bool haveEntryCandidate = false;
        for (int ci = leg.firstWpt; ci >= leg.firstWpt - 2 && ci >= 0; --ci) {
            const RouteWaypoint* entryW = route.waypoint(ci);
            if (!entryW || !entryW->posValid || entryW->id.empty() ||
                entryW->id == exitW->id) {
                continue;
            }
            haveEntryCandidate = true;
            if (cifp.findRfArc(entryW->id, exitW->id, exitW->lat, exitW->lon, arc)) {
                entryIdx = ci;
                found = true;
                break;
            }
        }

        // The CDU LEGS cache starts at the active waypoint, so an RF leg can be
        // active with its entry fix missing from the snapshot.  Match on the
        // exit fix alone and recover the entry coordinates from the CIFP row.
        // This must only run when the snapshot has **no** usable entry fix:
        // when a valid entry exists but the pair is not an RF leg, the leg is
        // an ordinary TF leg.  Borrowing an arc from a different procedure that
        // merely shares the exit fix (ZULS LS428 and LS819 are exits of RF legs
        // in other SIDs) annotated straight legs as arcs and made the aircraft
        // fly someone else's circle - the "越飞越偏 / 误识别为RF" reports.
        double entryLat = 0.0;
        double entryLon = 0.0;
        bool haveEntry = false;
        std::string entryId;
        if (found) {
            const RouteWaypoint* entryW = route.waypoint(entryIdx);
            if (entryW && entryW->posValid) {
                entryLat = entryW->lat;
                entryLon = entryW->lon;
                entryId = entryW->id;
                haveEntry = true;
            }
        } else {
            if (haveEntryCandidate) continue;
            if (!cifp.findRfArcByExit(exitW->id, exitW->lat, exitW->lon, arc)) {
                continue;
            }
            std::vector<NavFix> entryFix;
            if (!nav.resolve(std::vector<std::string>{arc.entryIdent}, exitW->lat,
                             exitW->lon, entryFix) ||
                entryFix.empty() || !entryFix[0].valid) {
                continue;
            }
            entryLat = entryFix[0].lat;
            entryLon = entryFix[0].lon;
            entryId = arc.entryIdent;
            haveEntry = true;
        }
        if (!haveEntry) continue;

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
                distanceNm(fix[0].lat, fix[0].lon, entryLat, entryLon);
            if (dExit > 0.5 * arc.radiusNm && dExit < 1.5 * arc.radiusNm &&
                dEntry > 0.5 * arc.radiusNm && dEntry < 1.5 * arc.radiusNm) {
                centerLat = fix[0].lat;
                centerLon = fix[0].lon;
                haveCenter = true;
            }
        }

        if (!haveCenter) {
            // Tangent continuity: the centre is R to the side of the inbound
            // course at the entry fix.  Only possible when the fix before the
            // entry is known.
            const RouteWaypoint* prevW =
                route.waypoint(entryIdx >= 0 ? entryIdx - 1 : -1);
            if (!prevW || !prevW->posValid) continue;
            const double inbound =
                bearingTrue(prevW->lat, prevW->lon, entryLat, entryLon);
            destinationPoint(entryLat, entryLon,
                             normalize360(inbound + arc.turn * 90.0),
                             arc.radiusNm, centerLat, centerLon);
        }

        const double radialStart =
            bearingTrue(centerLat, centerLon, entryLat, entryLon);
        const double radialEnd =
            bearingTrue(centerLat, centerLon, exitW->lat, exitW->lon);
        double total = normalize180(arc.turn * (radialEnd - radialStart));
        if (total < 0.0) total += 360.0;
        if (total < 5.0 || total > 355.0) continue;

        // Geometric consistency: the arc cannot be shorter than the chord.
        const double chord =
            distanceNm(entryLat, entryLon, exitW->lat, exitW->lon);
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
        leg.rfEntryId = entryId;
        leg.rfEntryLat = entryLat;
        leg.rfEntryLon = entryLon;
        leg.rfEntryValid = true;
    }
}

} // namespace lvnv
