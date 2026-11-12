#include "procedure_route.h"

#include "geo.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>
#include <vector>

namespace lvnv {

namespace {

std::string Upper(const std::string& value) {
    std::string out = value;
    for (char& c : out) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return out;
}

bool IsRunwayIdent(const std::string& ident) {
    return ident.size() >= 2 && ident[0] == 'R' && ident[1] == 'W';
}

std::vector<std::string> ProcedureIdents(const Procedure& procedure) {
    std::vector<std::string> out;
    for (const ProcedureLeg& leg : procedure.legs) {
        const std::string id = Upper(leg.fix);
        if (id.empty() || IsRunwayIdent(id)) continue;
        out.push_back(id);
    }
    return out;
}

std::vector<std::string> RouteIdents(const RouteSnapshot& route, int maxCount) {
    std::vector<std::string> out;
    const int n = std::min(static_cast<int>(route.wpts.size()), maxCount);
    for (int i = 0; i < n; ++i) {
        const std::string id = Upper(route.wpts[static_cast<size_t>(i)].id);
        if (!id.empty()) out.push_back(id);
    }
    return out;
}

// Longest contiguous run shared by two ident sequences.
int MatchRun(const std::vector<std::string>& a,
             const std::vector<std::string>& b, int* aStart, int* bStart) {
    int best = 0;
    int bestA = -1;
    int bestB = -1;
    for (int i = 0; i < static_cast<int>(a.size()); ++i) {
        for (int j = 0; j < static_cast<int>(b.size()); ++j) {
            int n = 0;
            while (i + n < static_cast<int>(a.size()) &&
                   j + n < static_cast<int>(b.size()) &&
                   a[static_cast<size_t>(i + n)] == b[static_cast<size_t>(j + n)]) {
                ++n;
            }
            if (n > best) {
                best = n;
                bestA = i;
                bestB = j;
            }
        }
    }
    if (aStart) *aStart = bestA;
    if (bStart) *bStart = bestB;
    return best;
}

bool SetRfGeometry(RouteLeg& leg, const ProcedureLeg& exitLeg,
                   const RouteWaypoint& entry, const RouteWaypoint& exit) {
    if (!exitLeg.rf || !exitLeg.rfCenterValid || exitLeg.rfRadiusNm < 0.3 ||
        exitLeg.rfTurn == 0) {
        return false;
    }
    const double radialStart =
        bearingTrue(exitLeg.rfCenterLat, exitLeg.rfCenterLon, entry.lat, entry.lon);
    const double radialEnd =
        bearingTrue(exitLeg.rfCenterLat, exitLeg.rfCenterLon, exit.lat, exit.lon);
    double total = normalize180(exitLeg.rfTurn * (radialEnd - radialStart));
    if (total < 0.0) total += 360.0;
    if (total < 5.0 || total > 355.0) return false;

    const double chord = distanceNm(entry.lat, entry.lon, exit.lat, exit.lon);
    if (exitLeg.rfRadiusNm * deg2rad(total) < chord * 0.98) return false;

    leg.rfArc = true;
    leg.rfRadiusNm = exitLeg.rfRadiusNm;
    leg.rfCenterLat = exitLeg.rfCenterLat;
    leg.rfCenterLon = exitLeg.rfCenterLon;
    leg.rfTurn = exitLeg.rfTurn;
    leg.rfRadialStartDeg = radialStart;
    leg.rfRadialEndDeg = radialEnd;
    leg.rfTotalSweptDeg = total;
    leg.rfEntryId = entry.id;
    leg.rfEntryLat = entry.lat;
    leg.rfEntryLon = entry.lon;
    leg.rfEntryValid = true;
    return true;
}

void CopyLegMetadata(RouteLeg& dst, const RouteLeg& src) {
    dst.type = src.type;
    dst.complex = src.complex;
    dst.hold = src.hold;
    dst.rfArc = src.rfArc;
    dst.rfRadiusNm = src.rfRadiusNm;
    dst.rfCenterLat = src.rfCenterLat;
    dst.rfCenterLon = src.rfCenterLon;
    dst.rfTurn = src.rfTurn;
    dst.rfRadialStartDeg = src.rfRadialStartDeg;
    dst.rfRadialEndDeg = src.rfRadialEndDeg;
    dst.rfTotalSweptDeg = src.rfTotalSweptDeg;
    dst.rfEntryWpt = src.rfEntryWpt;
    dst.rfEntryId = src.rfEntryId;
    dst.rfEntryLat = src.rfEntryLat;
    dst.rfEntryLon = src.rfEntryLon;
    dst.rfEntryValid = src.rfEntryValid;
}

} // namespace

bool ProcedureRouteBuilder::detect(const RouteSnapshot& fms, double lat,
                                   double lon, ProcedureSelection& out,
                                   const ProcedureRouteOptions& options) {
    out = ProcedureSelection{};
    if (!db_ || !fms.valid) return false;

    std::string icao;
    double aptLat = 0.0;
    double aptLon = 0.0;
    if (!db_->nearestAirport(lat, lon, options.maxAirportNm, icao, aptLat,
                             aptLon)) {
        return false;
    }
    if (!db_->loadAirport(icao)) return false;

    const std::vector<std::string> routeIds =
        RouteIdents(fms, std::max(2, options.lookahead));
    if (routeIds.size() < 2) return false;

    int bestScore = 0;
    double bestCoverage = 0.0;
    const Procedure* best = nullptr;
    for (const Procedure& p : db_->procedures(icao)) {
        const std::vector<std::string> procIds = ProcedureIdents(p);
        if (procIds.size() < 2) continue;
        int procStart = -1;
        int routeStart = -1;
        const int score = MatchRun(procIds, routeIds, &procStart, &routeStart);
        if (score < options.minMatches) continue;
        // The CDU LEGS cache often starts at the active fix, so only the last
        // one or two SID fixes may still be visible.  A two-fix match is
        // accepted when it is a suffix of the procedure (the route then
        // continues with the enroute fixes); longer matches are unambiguous.
        const bool suffix = procStart >= 0 && procStart + score ==
                                                  static_cast<int>(procIds.size());
        if (score < 3 && !suffix) continue;
        const double coverage =
            static_cast<double>(score) /
            static_cast<double>(std::min(procIds.size(), routeIds.size()));
        if (score > bestScore ||
            (score == bestScore && coverage > bestCoverage)) {
            bestScore = score;
            bestCoverage = coverage;
            best = &p;
        }
    }
    if (!best) return false;

    out.valid = true;
    out.airport = icao;
    out.ident = best->ident;
    out.transition = best->transition;
    out.score = bestScore;
    out.airportLat = aptLat;
    out.airportLon = aptLon;
    return true;
}

bool ProcedureRouteBuilder::BuildRoute(const Procedure& procedure,
                                       RouteSnapshot& out) {
    std::vector<const ProcedureLeg*> resolved;
    resolved.reserve(procedure.legs.size());
    for (const ProcedureLeg& leg : procedure.legs) {
        if (leg.posValid) resolved.push_back(&leg);
    }
    if (resolved.size() < 2) return false;

    out.clear();
    out.valid = true;
    out.sourceRoute = true;
    out.procedureAssisted = true;
    out.procedureId = procedure.ident + " " + procedure.transition;
    out.wptCount = static_cast<int>(resolved.size());
    out.wpts.resize(resolved.size());
    for (size_t i = 0; i < resolved.size(); ++i) {
        RouteWaypoint& w = out.wpts[i];
        w.id = resolved[i]->fix;
        w.lat = resolved[i]->lat;
        w.lon = resolved[i]->lon;
        w.posValid = true;
    }

    // CDU-style layout: leg 0 is a one-point leg, then leg i connects
    // waypoint i-1 to waypoint i.  The RF row for an arc is the exit leg, so
    // resolved[i] supplies the arc geometry for leg i.
    out.legs.resize(resolved.size());
    out.legs[0] = RouteLeg{0, 0, 0, false};
    for (int i = 1; i < static_cast<int>(resolved.size()); ++i) {
        RouteLeg leg;
        leg.firstWpt = i - 1;
        leg.lastWpt = i;
        leg.type = 0;
        leg.complex = false;
        if (resolved[static_cast<size_t>(i)]->rf) {
            SetRfGeometry(leg, *resolved[static_cast<size_t>(i)],
                          out.wpts[static_cast<size_t>(i - 1)],
                          out.wpts[static_cast<size_t>(i)]);
            if (leg.rfArc) leg.rfEntryWpt = i - 1;
        }
        out.legs[static_cast<size_t>(i)] = leg;
    }

    out.legCount = static_cast<int>(out.legs.size());
    out.activeWpt = 0;
    out.activeLeg = 0;
    out.nextLeg = out.legCount > 1 ? 1 : -1;
    out.activeId = out.wpts[0].id;
    if (out.nextLeg >= 0) {
        out.nextId = out.wpts[static_cast<size_t>(out.legs[1].lastWpt)].id;
    }
    return true;
}

bool ProcedureRouteBuilder::build(const ProcedureSelection& selection,
                                  RouteSnapshot& out, ResolveStats* stats) {
    if (!db_ || !selection.valid) return false;
    Procedure procedure;
    if (!db_->buildProcedure(selection.airport, selection.ident,
                             selection.transition, procedure, stats)) {
        return false;
    }
    return BuildRoute(procedure, out);
}

bool ProcedureRouteBuilder::merge(const RouteSnapshot& fms,
                                  const RouteSnapshot& procedure,
                                  RouteSnapshot& out) {
    if (!procedure.valid || procedure.wpts.size() < 2) {
        out = fms;
        return false;
    }
    if (!fms.valid || fms.wpts.size() < 2) {
        out = procedure;
        return true;
    }

    // Join on the latest procedure fix that also appears in the FMS route,
    // but start the merged prefix at the FIRST common fix: the CDU LEGS cache
    // often starts at the active fix, so procedure fixes before the route's
    // first ident are not part of what the aircraft is flying and may belong
    // to a different runway variant.
    int procFirst = -1;
    int fmsFirst = -1;
    int procJoin = -1;
    int fmsJoin = -1;
    const int fmsLimit = std::min(12, static_cast<int>(fms.wpts.size()));
    for (int p = 0; p < static_cast<int>(procedure.wpts.size()); ++p) {
        const std::string& pid = procedure.wpts[static_cast<size_t>(p)].id;
        if (pid.empty()) continue;
        for (int f = 0; f < fmsLimit; ++f) {
            if (Upper(fms.wpts[static_cast<size_t>(f)].id) != Upper(pid)) continue;
            if (procFirst < 0) {
                procFirst = p;
                fmsFirst = f;
            }
            procJoin = p;
            fmsJoin = f;
        }
    }
    if (procFirst < 0 || fmsFirst < 0 || procJoin < 0 || fmsJoin < 0) return false;
    if (fmsJoin < fmsFirst) return false;

    out.clear();
    out.valid = true;
    out.sourceRoute = true;
    out.procedureAssisted = true;
    out.procedureId = procedure.procedureId;

    const int prefixCount = procJoin - procFirst + 1;
    out.wpts.reserve(static_cast<size_t>(prefixCount) +
                     fms.wpts.size() - static_cast<size_t>(fmsJoin + 1));
    for (int i = procFirst; i <= procJoin; ++i) {
        out.wpts.push_back(procedure.wpts[static_cast<size_t>(i)]);
    }
    for (int i = fmsJoin + 1; i < static_cast<int>(fms.wpts.size()); ++i) {
        out.wpts.push_back(fms.wpts[static_cast<size_t>(i)]);
    }
    if (out.wpts.size() < 2) return false;

    out.legs.resize(out.wpts.size());
    out.legs[0] = RouteLeg{0, 0, 0, false};
    for (int i = 1; i < static_cast<int>(out.wpts.size()); ++i) {
        RouteLeg leg;
        leg.firstWpt = i - 1;
        leg.lastWpt = i;
        if (i < prefixCount) {
            const int procLegIdx = procFirst + i;
            if (procLegIdx < static_cast<int>(procedure.legs.size())) {
                leg = procedure.legs[static_cast<size_t>(procLegIdx)];
                leg.firstWpt = i - 1;
                leg.lastWpt = i;
                if (leg.rfArc) leg.rfEntryWpt = i - 1;
            }
        } else {
            const int fmsLegIdx = fmsJoin + (i - (prefixCount - 1));
            if (fmsLegIdx >= 0 && fmsLegIdx < static_cast<int>(fms.legs.size())) {
                CopyLegMetadata(leg, fms.legs[static_cast<size_t>(fmsLegIdx)]);
            }
        }
        out.legs[static_cast<size_t>(i)] = leg;
    }

    out.wptCount = static_cast<int>(out.wpts.size());
    out.legCount = static_cast<int>(out.legs.size());

    int active = 0;
    const int activeBase = prefixCount - 1;   // index of the join in out
    if (fms.activeWpt >= 0 && fms.activeWpt < static_cast<int>(fms.wpts.size())) {
        if (fms.activeWpt <= fmsJoin) {
            const std::string& want = fms.wpts[static_cast<size_t>(fms.activeWpt)].id;
            for (int i = 0; i < prefixCount; ++i) {
                if (Upper(out.wpts[static_cast<size_t>(i)].id) == Upper(want)) {
                    active = i;
                    break;
                }
            }
        } else {
            active = activeBase + (fms.activeWpt - fmsJoin);
        }
    }
    if (active < 0) active = 0;
    if (active >= static_cast<int>(out.wpts.size())) {
        active = static_cast<int>(out.wpts.size()) - 1;
    }
    out.activeWpt = active;
    out.activeLeg = active;
    out.nextLeg = (active + 1 < out.legCount) ? active + 1 : -1;
    out.activeId = out.wpts[static_cast<size_t>(active)].id;
    if (out.nextLeg >= 0) {
        out.nextId = out.wpts[static_cast<size_t>(out.legs[static_cast<size_t>(out.nextLeg)].lastWpt)].id;
    }
    return true;
}

} // namespace lvnv
