#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

namespace lvnv {

struct NavFix {
    double lat = 0.0;
    double lon = 0.0;
    bool valid = false;
};

// Resolves waypoint identifiers from the FF757 AIRAC text files:
//   Custom Data/STSFF/navdata/Waypoints.txt
//   Custom Data/STSFF/navdata/Navaids.txt
//   Custom Data/STSFF/navdata/Airports.txt
class NavDatabase {
public:
    NavDatabase() = default;

    void setRootPath(const std::string& xplaneRoot) { root_ = xplaneRoot; }
    const std::string& lastError() const { return lastError_; }

    bool resolve(const std::vector<std::string>& idents, double anchorLat,
                 double anchorLon, std::vector<NavFix>& out);

private:
    // One records-table entry of the in-memory navdata index.  Kept compact
    // (16 bytes) because the FF757 navdata holds ~380,000 fixes; the full
    // index is about 6 MB, well below the memory budget.
    struct FixEntry {
        char id[6];   // upper-case ident, max 5 chars + terminator
        float lat;
        float lon;
    };

    // Builds the in-memory index once per session: one sequential read of
    // Waypoints.txt / Navaids.txt / Airports.txt instead of a full scan per
    // unresolved ident.
    void ensureIndex();
    void addIndexRows(const std::string& path, int identField, int latField,
                      int lonField, bool airportRowsOnly);
    void indexLookup(const std::string& ident);

    // X-Plane's own fix database; contains computed procedure fixes (e.g. RF
    // arc centres such as CFBRK) that the FF757 navdata does not carry.  Kept
    // lazy (not indexed) because it is 14 MB and only needed for fixes that
    // the FF757 navdata cannot resolve.
    bool scanEarthFix(const std::vector<std::string>& wanted);

    std::string root_;
    std::string lastError_;
    std::map<std::string, std::vector<NavFix>> cache_;
    std::vector<FixEntry> index_;
    bool indexBuilt_ = false;
    // Idents already looked up, whether or not they were found, so a route
    // rebuild never touches the same unresolved ident again.
    std::set<std::string> attempted_;
    bool probeDone_ = false;
};

} // namespace lvnv
