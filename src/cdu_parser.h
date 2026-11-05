#pragma once

#include <string>
#include <vector>

namespace lvnv {

struct CduLeg {
    std::string id;
    int altFt = 0;              // estimated or constraint altitude
    int constraintAltFt = 0;    // hard altitude constraint, 0 if none
    int constraintSpeedKt = 0;  // hard speed constraint, 0 if none
    double legDistanceNm = 0.0; // distance printed on the leg's top line
    int page = 0;               // 1-based CDU page
    bool hold = false;
    bool discontinuity = false;
};

struct CduPage {
    bool valid = false;         // title is a LEGS page
    bool activeRoute = false;   // ACT RTE / RTE, not MOD RTE
    std::string title;
    std::string routeName;      // e.g. "ACT RTE 1"
    int page = 0;
    int pageCount = 0;
    std::vector<CduLeg> legs;
};

struct CduRoute {
    bool valid = false;
    bool activeRoute = false;
    std::string routeName;
    int pageCount = 0;
    std::vector<CduLeg> legs;
};

std::string TrimCdu(const std::string& value);
bool IsCduLegsPage(const std::vector<std::string>& rows);
int ParseCduAltitude(const std::string& text);
int ParseCduSpeed(const std::string& text);
CduPage ParseCduLegsPage(const std::vector<std::string>& rows, int pageHint = 0);

// Appends the legs of one page into a route, skipping duplicate page joins.
void AppendCduPage(CduRoute& route, const CduPage& page);

} // namespace lvnv
