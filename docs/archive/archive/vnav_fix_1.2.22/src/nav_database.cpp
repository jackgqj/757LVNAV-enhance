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

namespace {

// Copies field number `index` of a comma-separated line into `out` without
// building the whole field vector; the index builder runs over ~380,000 rows.
bool CsvField(const std::string& line, int index, std::string& out) {
    int field = 0;
    size_t start = 0;
    while (true) {
        const size_t comma = line.find(',', start);
        const size_t end = (comma == std::string::npos) ? line.size() : comma;
        if (field == index) {
            out.assign(line, start, end - start);
            return true;
        }
        if (comma == std::string::npos) return false;
        start = comma + 1;
        ++field;
    }
}

bool ParseFloatCoord(const std::string& text, float& out) {
    const std::string value = Trim(text);
    if (value.empty()) return false;
    char* end = nullptr;
    const double parsed = std::strtod(value.c_str(), &end);
    if (end == value.c_str()) return false;
    out = static_cast<float>(parsed);
    return true;
}

} // namespace

void NavDatabase::addIndexRows(const std::string& path, int identField,
                               int latField, int lonField, bool airportRowsOnly) {
    std::ifstream file(path);
    if (!file.is_open()) return;
    std::string line;
    std::string ident;
    std::string latText;
    std::string lonText;
    while (std::getline(file, line)) {
        // Airports.txt holds several row types; keep only the airport record.
        if (airportRowsOnly && !(line.size() > 2 && line[0] == 'A' && line[1] == ',')) {
            continue;
        }
        if (!CsvField(line, identField, ident)) continue;
        if (!CsvField(line, latField, latText)) continue;
        if (!CsvField(line, lonField, lonText)) continue;
        const std::string key = Upper(Trim(ident));
        if (key.empty() || key.size() > 5) continue;
        float lat = 0.0f;
        float lon = 0.0f;
        if (!ParseFloatCoord(latText, lat) || !ParseFloatCoord(lonText, lon)) continue;
        if (!(lat >= -90.0f && lat <= 90.0f && lon >= -180.0f && lon <= 180.0f)) continue;
        if (std::abs(lat) < 1e-5f && std::abs(lon) < 1e-5f) continue;

        FixEntry entry;
        const size_t n = std::min<size_t>(key.size(), 5);
        std::memcpy(entry.id, key.data(), n);
        entry.id[n] = '\0';
        entry.lat = lat;
        entry.lon = lon;
        index_.push_back(entry);
        if (index_.size() >= 600000) break;   // hard memory cap (~9.6 MB)
    }
}

void NavDatabase::ensureIndex() {
    if (indexBuilt_) return;
    indexBuilt_ = true;

    // One sequential read per file, then every lookup is a binary search.
    // Pre-sized to the observed row count so building does not reallocate.
    index_.reserve(420000);
    addIndexRows(root_ + "Custom Data/STSFF/navdata/Waypoints.txt", 0, 1, 2, false);
    addIndexRows(root_ + "Custom Data/STSFF/navdata/Navaids.txt", 0, 6, 7, false);
    addIndexRows(root_ + "Custom Data/STSFF/navdata/Airports.txt", 1, 3, 4, true);

    std::sort(index_.begin(), index_.end(),
              [](const FixEntry& a, const FixEntry& b) {
                  return std::strcmp(a.id, b.id) < 0;
              });
}

void NavDatabase::indexLookup(const std::string& ident) {
    if (index_.empty()) return;
    FixEntry probe;
    std::snprintf(probe.id, sizeof(probe.id), "%s",
                  Upper(Trim(ident)).c_str());

    auto it = std::lower_bound(index_.begin(), index_.end(), probe,
                               [](const FixEntry& a, const FixEntry& b) {
                                   return std::strcmp(a.id, b.id) < 0;
                               });
    // AddCandidate keeps at most 8 candidates per ident and de-duplicates, so
    // a repeated ident in several sources or regions stays bounded.
    for (; it != index_.end() && std::strcmp(it->id, probe.id) == 0; ++it) {
        AddCandidate(cache_, probe.id, it->lat, it->lon);
    }
}

bool NavDatabase::scanEarthFix(const std::vector<std::string>& wanted) {
    std::ifstream file(root_ + "Resources/default data/earth_fix.dat");
    if (!file.is_open()) return false;
    std::string line;
    while (std::getline(file, line)) {
        // Row layout: "<lat> <lon> <IDENT> <region> ..." (whitespace
        // separated); header/footer lines do not start with a number.
        std::vector<std::string> parts;
        size_t pos = 0;
        while (pos < line.size()) {
            while (pos < line.size() &&
                   std::isspace(static_cast<unsigned char>(line[pos]))) ++pos;
            size_t end = pos;
            while (end < line.size() &&
                   !std::isspace(static_cast<unsigned char>(line[end]))) ++end;
            if (end > pos) parts.push_back(line.substr(pos, end - pos));
            pos = end;
        }
        if (parts.size() < 3) continue;
        double lat = 0.0;
        double lon = 0.0;
        if (!ParseCoord(parts[0], lat) || !ParseCoord(parts[1], lon)) continue;
        const std::string ident = Upper(Trim(parts[2]));
        if (ident.empty()) continue;
        bool needed = false;
        for (const std::string& w : wanted) {
            if (w == ident) { needed = true; break; }
        }
        if (!needed) continue;
        AddCandidate(cache_, ident, lat, lon);
    }
    return true;
}

bool NavDatabase::resolve(const std::vector<std::string>& idents, double anchorLat,
                          double anchorLon, std::vector<NavFix>& out) {
    out.assign(idents.size(), NavFix{});
    if (idents.empty()) return false;

    std::vector<std::string> missing;
    for (size_t i = 0; i < idents.size(); ++i) {
        const std::string key = Upper(Trim(idents[i]));
        if (key.empty()) continue;
        if (cache_.find(key) != cache_.end()) continue;
        // Already searched and not found: do not touch the filesystem again.
        if (attempted_.count(key) != 0) continue;
        missing.push_back(key);
    }
    if (!missing.empty()) {
        // The index covers the whole FF757 navdata set in one read, so a new
        // ident costs a binary search instead of a 9 MB rescan.
        ensureIndex();
        for (const std::string& key : missing) indexLookup(key);

        // Anything the FF757 navdata cannot resolve falls back to X-Plane's
        // own fix file (computed procedure fixes).  14 MB, scanned lazily.
        std::vector<std::string> stillMissing;
        for (const std::string& key : missing) {
            auto it = cache_.find(key);
            if (it == cache_.end() || it->second.empty()) stillMissing.push_back(key);
        }
        if (!stillMissing.empty()) scanEarthFix(stillMissing);
        // Remember every searched ident so a route rebuild never repeats it,
        // and so the frame loop stays off the filesystem entirely.
        for (const std::string& key : missing) attempted_.insert(key);
    }
    if (!missing.empty() && !probeDone_) {
        probeDone_ = true;
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
        // A same-ident fix far from the running route is a navdata collision
        // (e.g. an approach fix that only exists elsewhere in the world), not
        // this route's fix.  Reject it so the CDU reader synthesises the leg
        // from the printed course/distance instead of building a 2000 NM leg.
        const double bestNm = std::sqrt(bestDist) * 60.0;
        if (bestNm > 500.0) {
            lastError_ = "fix too far from route; ignored";
            continue;
        }
        out[i] = *best;
        prevLat = best->lat;
        prevLon = best->lon;
        any = true;
    }
    return any;
}

} // namespace lvnv
