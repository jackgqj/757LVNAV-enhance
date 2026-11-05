#include "nav_database.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <set>

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

bool ParseCoord(const std::string& text, double& out) {
    const std::string value = Trim(text);
    if (value.empty()) return false;
    char* end = nullptr;
    const double parsed = std::strtod(value.c_str(), &end);
    if (end == value.c_str()) return false;
    out = parsed;
    return true;
}

void AddCandidate(std::map<std::string, std::vector<NavFix>>& cache,
                  const std::string& ident, double lat, double lon) {
    if (!(lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0)) return;
    if (std::abs(lat) < 0.00001 && std::abs(lon) < 0.00001) return;
    std::vector<NavFix>& list = cache[ident];
    for (const NavFix& existing : list) {
        if (std::abs(existing.lat - lat) < 1e-6 && std::abs(existing.lon - lon) < 1e-6) return;
    }
    if (list.size() >= 8) return;
    NavFix fix;
    fix.lat = lat;
    fix.lon = lon;
    fix.valid = true;
    list.push_back(fix);
}

double RoughDistanceSq(double lat1, double lon1, double lat2, double lon2) {
    const double dlat = lat1 - lat2;
    double dlon = lon1 - lon2;
    if (dlon > 180.0) dlon -= 360.0;
    if (dlon < -180.0) dlon += 360.0;
    const double cosLat = std::cos((lat1 + lat2) * 0.5 * 0.0174532925199433);
    return dlat * dlat + (dlon * cosLat) * (dlon * cosLat);
}

} // namespace

void NavDatabase::scanWaypoints(const std::vector<std::string>& wanted) {
    std::ifstream file(root_ + "Custom Data/STSFF/navdata/Waypoints.txt");
    if (!file.is_open()) return;
    std::string line;
    while (std::getline(file, line)) {
        const std::vector<std::string> parts = SplitComma(line);
        if (parts.size() < 3) continue;
        const std::string ident = Upper(Trim(parts[0]));
        if (ident.empty()) continue;
        bool needed = false;
        for (const std::string& w : wanted) {
            if (w == ident) { needed = true; break; }
        }
        if (!needed) continue;
        double lat = 0.0;
        double lon = 0.0;
        if (!ParseCoord(parts[1], lat) || !ParseCoord(parts[2], lon)) continue;
        AddCandidate(cache_, ident, lat, lon);
    }
}

void NavDatabase::scanNavaids(const std::vector<std::string>& wanted) {
    std::ifstream file(root_ + "Custom Data/STSFF/navdata/Navaids.txt");
    if (!file.is_open()) return;
    std::string line;
    while (std::getline(file, line)) {
        const std::vector<std::string> parts = SplitComma(line);
        if (parts.size() < 8) continue;
        const std::string ident = Upper(Trim(parts[0]));
        if (ident.empty()) continue;
        bool needed = false;
        for (const std::string& w : wanted) {
            if (w == ident) { needed = true; break; }
        }
        if (!needed) continue;
        double lat = 0.0;
        double lon = 0.0;
        if (!ParseCoord(parts[6], lat) || !ParseCoord(parts[7], lon)) continue;
        AddCandidate(cache_, ident, lat, lon);
    }
}

void NavDatabase::scanAirports(const std::vector<std::string>& wanted) {
    std::ifstream file(root_ + "Custom Data/STSFF/navdata/Airports.txt");
    if (!file.is_open()) return;
    std::string line;
    while (std::getline(file, line)) {
        const std::vector<std::string> parts = SplitComma(line);
        if (parts.size() < 5) continue;
        if (Trim(parts[0]) != "A") continue;
        const std::string ident = Upper(Trim(parts[1]));
        if (ident.empty()) continue;
        bool needed = false;
        for (const std::string& w : wanted) {
            if (w == ident) { needed = true; break; }
        }
        if (!needed) continue;
        double lat = 0.0;
        double lon = 0.0;
        if (!ParseCoord(parts[3], lat) || !ParseCoord(parts[4], lon)) continue;
        AddCandidate(cache_, ident, lat, lon);
    }
}

void NavDatabase::loadMissing(const std::vector<std::string>& missing) {
    if (missing.empty()) return;
    scanWaypoints(missing);
    scanNavaids(missing);
    scanAirports(missing);
}

bool NavDatabase::resolve(const std::vector<std::string>& idents, double anchorLat,
                          double anchorLon, std::vector<NavFix>& out) {
    out.assign(idents.size(), NavFix{});
    if (idents.empty()) return false;

    std::vector<std::string> missing;
    for (size_t i = 0; i < idents.size(); ++i) {
        const std::string key = Upper(Trim(idents[i]));
        if (key.empty()) continue;
        if (cache_.find(key) == cache_.end()) missing.push_back(key);
    }
    if (!missing.empty()) loadMissing(missing);
    if (missing.size() == idents.size()) {
        // Nothing was cached; report the likely reason once.
        std::ifstream probe(root_ + "Custom Data/STSFF/navdata/Waypoints.txt");
        if (!probe.is_open()) {
            lastError_ = "FF757 navdata Waypoints.txt not found";
            return false;
        }
        lastError_ = "waypoint identifiers not found in navdata";
    }

    double prevLat = anchorLat;
    double prevLon = anchorLon;
    bool any = false;
    for (size_t i = 0; i < idents.size(); ++i) {
        const std::string key = Upper(Trim(idents[i]));
        auto it = cache_.find(key);
        if (it == cache_.end() || it->second.empty()) continue;
        const NavFix* best = nullptr;
        double bestDist = 1e30;
        for (const NavFix& fix : it->second) {
            const double d = RoughDistanceSq(prevLat, prevLon, fix.lat, fix.lon);
            if (d < bestDist) {
                bestDist = d;
                best = &fix;
            }
        }
        if (!best) continue;
        out[i] = *best;
        prevLat = best->lat;
        prevLon = best->lon;
        any = true;
    }
    return any;
}

} // namespace lvnv
