#pragma once

#include <map>
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
    void loadMissing(const std::vector<std::string>& missing);
    void scanWaypoints(const std::vector<std::string>& wanted);
    void scanNavaids(const std::vector<std::string>& wanted);
    void scanAirports(const std::vector<std::string>& wanted);

    std::string root_;
    std::string lastError_;
    std::map<std::string, std::vector<NavFix>> cache_;
};

} // namespace lvnv
