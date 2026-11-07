#pragma once

#include <climits>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace lvnv {

struct RouteWaypoint {
    double lat = 0.0;
    double lon = 0.0;
    bool posValid = false;        // false when the ident could not be resolved
    int altFt = INT_MIN;          // estimated / constraint altitude, if available
    bool altValid = false;
    int constraintSpeedKt = 0;    // hard speed constraint at this waypoint ("250"), 0 = none
    int estSpeedKt = 0;           // published estimate ("310"), when present
    bool speedValid = false;      // estSpeedKt is present on the CDU line
    std::string id;
};

struct RouteLeg {
    int type = 0;                 // raw FF757 leg type (undocumented)
    int firstWpt = -1;            // inclusive index into RouteSnapshot::wpts
    int lastWpt = -1;             // inclusive
    bool complex = false;         // multi-point leg (arc, hold, procedure turn)

    // RF (constant-radius-to-fix) arc geometry, resolved from the X-Plane
    // CIFP database for the airport nearest the leg.  rfArc is true only when
    // every field below is usable; guidance then tracks the arc itself instead
    // of passing the leg through to the FF757's polygonal approximation.
    bool rfArc = false;
    double rfRadiusNm = 0.0;
    double rfCenterLat = 0.0;
    double rfCenterLon = 0.0;
    int rfTurn = 0;               // +1 = right (clockwise), -1 = left
    double rfRadialStartDeg = 0.0;  // bearing centre -> entry fix
    double rfRadialEndDeg = 0.0;    // bearing centre -> exit fix
    double rfTotalSweptDeg = 0.0;   // signed turn direction applied, 5..355
    int rfEntryWpt = -1;          // index of the arc entry waypoint, -1 if absent
    // The arc entry fix resolved from CIFP.  The CDU LEGS cache starts at the
    // active waypoint, so an RF leg can be active without its entry fix in the
    // snapshot; the geometry is still usable from these coordinates.
    std::string rfEntryId;
    double rfEntryLat = 0.0;
    double rfEntryLon = 0.0;
    bool rfEntryValid = false;
};

struct RouteSnapshot {
    bool valid = false;
    bool sourceRoute = false;     // true when the FF757 route datarefs were read
    std::string validationError;

    int legCount = 0;
    int wptCount = 0;
    int activeWpt = -1;
    int activeLeg = -1;
    int nextLeg = -1;
    int rawRteCount = 0;

    double vtk = 0.0;             // raw FF757 vertical track value
    std::vector<RouteWaypoint> wpts;
    std::vector<RouteLeg> legs;

    std::string activeId;
    std::string nextId;

    void clear() {
        valid = false;
        sourceRoute = false;
        validationError.clear();
        legCount = 0;
        wptCount = 0;
        activeWpt = -1;
        activeLeg = -1;
        nextLeg = -1;
        rawRteCount = 0;
        vtk = 0.0;
        wpts.clear();
        legs.clear();
        activeId.clear();
        nextId.clear();
    }

    const RouteWaypoint* waypoint(int idx) const {
        if (idx < 0 || idx >= static_cast<int>(wpts.size())) return nullptr;
        return &wpts[static_cast<size_t>(idx)];
    }

    const RouteLeg* leg(int idx) const {
        if (idx < 0 || idx >= static_cast<int>(legs.size())) return nullptr;
        return &legs[static_cast<size_t>(idx)];
    }

    bool activeWaypointAlive() const {
        const RouteWaypoint* w = waypoint(activeWpt);
        return w && w->posValid;
    }
};

inline int findLegContaining(const RouteSnapshot& r, int wptIdx) {
    for (int i = 0; i < static_cast<int>(r.legs.size()); ++i) {
        const RouteLeg& l = r.legs[static_cast<size_t>(i)];
        if (wptIdx >= l.firstWpt && wptIdx <= l.lastWpt) return i;
    }
    return -1;
}

inline int findPreviousLegEnd(const RouteSnapshot& r, int legIdx) {
    if (legIdx <= 0) return -1;
    const RouteLeg* prev = r.leg(legIdx - 1);
    return prev ? prev->lastWpt : -1;
}

} // namespace lvnv
