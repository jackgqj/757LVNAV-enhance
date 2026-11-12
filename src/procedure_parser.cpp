#include "procedure_parser.h"

#include "geo.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <string>

// ===================================================================
// RF (radius-to-fix) geometry/resolution — DO NOT MODIFY WITHOUT USER APPROVAL
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

bool IsLegType(const std::string& type) {
    static const char* kTypes[] = {"IF", "TF", "RF", "DF", "CF", "CA", "FA",
                                   "FC", "FD", "VA", "VM", "VI", "VR", "CI"};
    for (const char* t : kTypes) {
        if (type == t) return true;
    }
    return false;
}

int ToInt(const std::string& value) {
    const std::string s = Trim(value);
    if (s.empty()) return 0;
    return std::atoi(s.c_str());
}

double ToDouble(const std::string& value) {
    const std::string s = Trim(value);
    if (s.empty()) return 0.0;
    return std::atof(s.c_str());
}

const Procedure* FindProcedure(const std::vector<Procedure>& list,
                               const std::string& ident,
                               const std::string& transition) {
    const std::string wantIdent = Upper(Trim(ident));
    const std::string wantTrans = CifpProcedureDatabase::NormalizeTransition(transition);
    for (const Procedure& p : list) {
        if (Upper(p.ident) == wantIdent && p.transition == wantTrans) return &p;
    }
    return nullptr;
}

bool ValidLatLon(double lat, double lon) {
    return std::isfinite(lat) && std::isfinite(lon) && std::abs(lat) <= 90.0 &&
           std::abs(lon) <= 180.0;
}

} // namespace

std::string CifpProcedureDatabase::NormalizeTransition(
    const std::string& transition) {
    std::string t = Upper(Trim(transition));
    if (t.rfind("RW", 0) == 0) t.erase(0, 2);
    if (t.rfind("RWY", 0) == 0) t.erase(0, 3);
    return t;
}

std::vector<Procedure> CifpProcedureDatabase::ParseCifpSidLines(
    const std::vector<std::string>& lines, const std::string& airport) {
    std::vector<Procedure> out;
    std::map<std::string, int> groups;
    for (const std::string& line : lines) {
        const std::string trimmed = Trim(line);
        if (trimmed.empty()) continue;
        const std::vector<std::string> parts = SplitComma(trimmed);
        if (parts.size() < 12) continue;

        std::string section = Upper(parts[0]);
        const size_t colon = section.find(':');
        if (colon == std::string::npos) continue;
        const std::string seqText = section.substr(colon + 1);
        section = section.substr(0, colon);
        if (section != "SID") continue;

        const std::string ident = Upper(Trim(parts[2]));
        const std::string transition = NormalizeTransition(parts[3]);
        const std::string fix = Upper(Trim(parts[4]));
        const std::string type = Upper(Trim(parts[11]));
        if (ident.empty() || fix.empty() || !IsLegType(type)) continue;

        const std::string key = ident + "|" + transition;
        auto it = groups.find(key);
        if (it == groups.end()) {
            Procedure p;
            p.airport = airport;
            p.kind = "SID";
            p.ident = ident;
            p.transition = transition;
            out.push_back(std::move(p));
            it = groups.emplace(key, static_cast<int>(out.size()) - 1).first;
        }
        Procedure& proc = out[static_cast<size_t>(it->second)];
        ProcedureLeg leg;
        leg.type = type;
        leg.fix = fix;
        leg.sequence = ToInt(seqText);
        if (type == "RF" && parts.size() > 30) {
            const std::string dir = Upper(Trim(parts[9]));
            leg.rf = true;
            leg.rfTurn = (dir == "L") ? -1 : (dir == "R") ? 1 : 0;
            leg.rfRadiusNm = ToDouble(parts[17]) / 1000.0;
            leg.rfCenter = Upper(Trim(parts[30]));
        }
        proc.legs.push_back(std::move(leg));
    }

    for (Procedure& p : out) {
        std::stable_sort(p.legs.begin(), p.legs.end(),
                         [](const ProcedureLeg& a, const ProcedureLeg& b) {
                             return a.sequence < b.sequence;
                         });
    }
    return out;
}

void CifpProcedureDatabase::ResolveProcedure(Procedure& procedure,
                                             const FixResolver& resolver,
                                             double airportLat, double airportLon,
                                             ResolveStats* stats) {
    if (stats) *stats = ResolveStats{};
    double anchorLat = airportLat;
    double anchorLon = airportLon;

    for (size_t i = 0; i < procedure.legs.size(); ++i) {
        ProcedureLeg& leg = procedure.legs[i];
        if (stats) ++stats->legs;

        double lat = 0.0;
        double lon = 0.0;
        if (resolver && resolver(leg.fix, anchorLat, anchorLon, lat, lon) &&
            ValidLatLon(lat, lon)) {
            leg.lat = lat;
            leg.lon = lon;
            leg.posValid = true;
            anchorLat = lat;
            anchorLon = lon;
            if (stats) ++stats->resolved;
        }

        if (!leg.rf) continue;
        if (stats) ++stats->rf;

        double cLat = 0.0;
        double cLon = 0.0;
        if (resolver && !leg.rfCenter.empty() &&
            resolver(leg.rfCenter, anchorLat, anchorLon, cLat, cLon) &&
            ValidLatLon(cLat, cLon)) {
            leg.rfCenterLat = cLat;
            leg.rfCenterLon = cLon;
            leg.rfCenterValid = true;
            if (stats) ++stats->rfCenters;
            continue;
        }

        // Centre not in the nav database: reconstruct it from the entry fix,
        // the inbound course and the radius (same tangent-continuity rule the
        // plugin already uses for FF757 Proc arcs).
        if (!leg.posValid || leg.rfRadiusNm < 0.3 || leg.rfTurn == 0) continue;
        const int entryIdx = static_cast<int>(i) - 1;
        if (entryIdx < 0 || !procedure.legs[static_cast<size_t>(entryIdx)].posValid) {
            continue;
        }
        const ProcedureLeg& entry = procedure.legs[static_cast<size_t>(entryIdx)];
        double inbound = bearingTrue(entry.lat, entry.lon, leg.lat, leg.lon);
        if (entryIdx > 0) {
            const ProcedureLeg& prev = procedure.legs[static_cast<size_t>(entryIdx - 1)];
            if (prev.posValid) {
                inbound = bearingTrue(prev.lat, prev.lon, entry.lat, entry.lon);
            }
        }
        double outLat = 0.0;
        double outLon = 0.0;
        destinationPoint(entry.lat, entry.lon,
                         normalize360(inbound + leg.rfTurn * 90.0),
                         leg.rfRadiusNm, outLat, outLon);
        if (ValidLatLon(outLat, outLon)) {
            leg.rfCenterLat = outLat;
            leg.rfCenterLon = outLon;
            leg.rfCenterValid = true;
            if (stats) ++stats->rfCenters;
        }
    }
}

bool CifpProcedureDatabase::nearestAirport(double lat, double lon, double maxNm,
                                           std::string& icao, double& aptLat,
                                           double& aptLon) {
    if (!airportsLoaded_) {
        airportsLoaded_ = true;
        std::string root = root_;
        if (!root.empty() && root.back() != '/' && root.back() != '\\') {
            root += '/';
        }
        std::ifstream in(root + "Custom Data/STSFF/navdata/Airports.txt");
        std::string line;
        while (std::getline(in, line)) {
            const std::vector<std::string> parts = SplitComma(line);
            if (parts.size() < 5) continue;
            if (Upper(Trim(parts[0])) != "A") continue;
            AirportRef a;
            a.icao = Upper(Trim(parts[1]));
            a.lat = ToDouble(parts[3]);
            a.lon = ToDouble(parts[4]);
            if (!a.icao.empty() && ValidLatLon(a.lat, a.lon)) {
                airports_.push_back(a);
            }
        }
    }

    bool found = false;
    double best = maxNm;
    for (const AirportRef& a : airports_) {
        const double d = distanceNm(lat, lon, a.lat, a.lon);
        if (d <= best) {
            best = d;
            icao = a.icao;
            aptLat = a.lat;
            aptLon = a.lon;
            found = true;
        }
    }
    return found;
}

bool CifpProcedureDatabase::airportLatLon(const std::string& icao, double& lat,
                                          double& lon) const {
    std::string root = root_;
    if (!root.empty() && root.back() != '/' && root.back() != '\\') root += '/';
    std::ifstream in(root + "Custom Data/STSFF/navdata/Airports.txt");
    if (!in.is_open()) return false;
    const std::string want = Upper(Trim(icao));
    std::string line;
    while (std::getline(in, line)) {
        const std::vector<std::string> parts = SplitComma(line);
        if (parts.size() < 5) continue;
        if (Upper(Trim(parts[0])) != "A") continue;
        if (Upper(Trim(parts[1])) != want) continue;
        lat = ToDouble(parts[3]);
        lon = ToDouble(parts[4]);
        return ValidLatLon(lat, lon);
    }
    return false;
}

bool CifpProcedureDatabase::loadAirport(const std::string& icao) {
    const std::string key = Upper(Trim(icao));
    if (key.empty()) return false;
    if (loaded_.count(key) != 0) return true;

    std::string root = root_;
    if (!root.empty() && root.back() != '/' && root.back() != '\\') root += '/';

    std::vector<std::string> lines;
    std::string path = root + "Custom Data/CIFP/" + key + ".dat";
    std::ifstream in(path);
    if (!in.is_open()) {
        path = root + "Resources/default data/CIFP/" + key + ".dat";
        in.open(path);
    }
    if (in.is_open()) {
        std::string line;
        while (std::getline(in, line)) lines.push_back(line);
    }
    procedures_[key] = ParseCifpSidLines(lines, key);
    loaded_.insert(key);
    return !procedures_[key].empty();
}

const std::vector<Procedure>& CifpProcedureDatabase::procedures(
    const std::string& icao) const {
    static const std::vector<Procedure> empty;
    const auto it = procedures_.find(Upper(Trim(icao)));
    return (it == procedures_.end()) ? empty : it->second;
}

bool CifpProcedureDatabase::buildProcedure(const std::string& icao,
                                           const std::string& ident,
                                           const std::string& transition,
                                           Procedure& out,
                                           ResolveStats* stats) {
    const std::string key = Upper(Trim(icao));
    const Procedure* proc = FindProcedure(procedures(key), ident, transition);
    if (!proc) return false;
    out = *proc;

    double aptLat = 0.0;
    double aptLon = 0.0;
    const bool haveAirport = airportLatLon(key, aptLat, aptLon);
    const FixResolver resolver = [this](const std::string& fix, double anchorLat,
                                        double anchorLon, double& lat,
                                        double& lon) {
        std::vector<NavFix> fixes;
        if (!nav_.resolve(std::vector<std::string>{fix}, anchorLat, anchorLon,
                          fixes) ||
            fixes.empty() || !fixes[0].valid) {
            return false;
        }
        lat = fixes[0].lat;
        lon = fixes[0].lon;
        return true;
    };
    ResolveProcedure(out, resolver, haveAirport ? aptLat : 0.0,
                     haveAirport ? aptLon : 0.0, stats);
    return true;
}

} // namespace lvnv
