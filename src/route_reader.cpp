#include "route_reader.h"

#include "log.h"

#include "XPLMUtilities.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

namespace lvnv {

namespace {

const char* kPrefixes[] = {
    "1-sim/for-internal-usage-only/fms/",
    "757Avionics/fms/",
    "1-sim/fms/",
};

const char* kKnownSuffixes[] = {
    "rteData/rteCount",
    "rteData/rteOffsets",
    "rteData/rteTypes",
    "rteData/rteSizes",
    "rteData/wptsLat",
    "rteData/wptsLon",
    "rteData/wptsEstimatedAlt",
    "rteData/wptsEstimatedAltStart",
    "rteData/wptsEstimatedAltLen",
    "rteData/wptsId",
    "rteData/wptsIdStart",
    "rteData/wptsIdLen",
    "rteData/actWpt/idx",
    "rteData/actWpt/distToGo",
    "rteData/vtk",
};

std::string trim(const std::string& s) {
    size_t a = 0;
    while (a < s.size() && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r' ||
                            s[a] == '\n' || s[a] == '\0')) {
        ++a;
    }
    size_t b = s.size();
    while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t' || s[b - 1] == '\r' ||
                     s[b - 1] == '\n' || s[b - 1] == '\0')) {
        --b;
    }
    return s.substr(a, b - a);
}

std::string sliceString(const std::vector<char>& buffer, int start, int len) {
    if (start < 0 || start >= static_cast<int>(buffer.size())) return {};
    int n = len;
    if (n <= 0) return {};
    if (start + n > static_cast<int>(buffer.size())) {
        n = static_cast<int>(buffer.size()) - start;
    }
    std::string s(buffer.data() + start, static_cast<size_t>(n));
    return trim(s);
}

// The private route alt string carries the same constraint qualifier as the
// CDU LEGS page: "FL157A" = at or above, "6890B" = at or below.
AltConstraint parseAltConstraint(const std::string& input) {
    std::string s = trim(input);
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    if (s.find("ABOVE") != std::string::npos) return AltAtOrAbove;
    if (s.find("BELOW") != std::string::npos) return AltAtOrBelow;

    size_t lastDigit = std::string::npos;
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] >= '0' && s[i] <= '9') lastDigit = i;
    }
    if (lastDigit == std::string::npos) return AltAt;
    size_t j = lastDigit + 1;
    while (j < s.size() && (s[j] == ' ' || s[j] == '\t')) ++j;
    if (j < s.size()) {
        if (s[j] == 'A') return AltAtOrAbove;
        if (s[j] == 'B') return AltAtOrBelow;
    }
    return AltAt;
}

int parseAltitudeString(const std::string& input, AltConstraint& altType) {
    altType = AltAt;
    std::string s = trim(input);
    if (s.empty()) return INT_MIN;

    bool hasDigits = false;
    for (char c : s) {
        if (c >= '0' && c <= '9') {
            hasDigits = true;
            break;
        }
    }
    if (!hasDigits) return INT_MIN;

    bool flightLevel = false;
    size_t fl = s.find("FL");
    if (fl == std::string::npos) fl = s.find("fl");
    if (fl != std::string::npos) flightLevel = true;

    long value = 0;
    bool started = false;
    for (char c : s) {
        if (c >= '0' && c <= '9') {
            value = value * 10 + (c - '0');
            started = true;
            if (value > 200000) break;
        } else if (started) {
            break;
        }
    }
    if (!started) return INT_MIN;
    if (flightLevel && value < 1000) value *= 100;
    if (value <= 0 || value > 100000) return INT_MIN;
    altType = parseAltConstraint(input);
    return static_cast<int>(value);
}

} // namespace

RouteReader::RouteReader() : procBuilder_(&cifpProc_) {}

XPLMDataRef RouteReader::find(const std::string& suffix) const {
    auto it = refs_.find(suffix);
    if (it != refs_.end()) return it->second;

    XPLMDataRef result = nullptr;
    for (const char* prefix : kPrefixes) {
        std::string full = std::string(prefix) + suffix;
        XPLMDataRef r = XPLMFindDataRef(full.c_str());
        if (r) {
            result = r;
            break;
        }
    }
    refs_[suffix] = result;
    return result;
}

void RouteReader::bind() {
    refs_.clear();
    report_.clear();
    bound_ = true;

    if (!cifpProcRootSet_) {
        char root[512] = {0};
        XPLMGetSystemPath(root);
        if (root[0]) {
            cifpProc_.setRootPath(root);
            cifpProcRootSet_ = true;
        }
    }

    for (const char* suffix : kKnownSuffixes) {
        XPLMDataRef r = find(suffix);
        if (r) {
            if (!report_.empty()) report_ += " ";
            report_ += suffix;
        }
    }
    if (report_.empty()) {
        report_ = "none";
    }
}

void RouteReader::reset() {
    route_.clear();
    baseRoute_.clear();
    mergedRoute_.clear();
    baseValid_ = false;
    hasMergedRoute_ = false;
    cifpAssisted_ = false;
    cifpApplied_ = false;
    cifpChecked_ = false;
    cifpRouteSignature_.clear();
    cifpNote_.clear();
    lastCifpAttempt_ = -1000.0;
    lastFullRefresh_ = -1000.0;
    lastBindAttempt_ = -1000.0;
    lastActiveIdx_ = -2;
    lastError_.clear();
}

bool RouteReader::hasRouteDatarefs() const {
    return find("rteData/wptsLat") && find("rteData/wptsLon");
}

bool RouteReader::readInt(const std::string& suffix, int& out) const {
    XPLMDataRef r = find(suffix);
    if (!r) return false;
    int types = XPLMGetDataRefTypes(r);
    if (types & xplmType_Int) {
        out = XPLMGetDatai(r);
        return true;
    }
    if (types & xplmType_Float) {
        out = static_cast<int>(std::llround(XPLMGetDataf(r)));
        return true;
    }
    if (types & xplmType_IntArray) {
        int v[1] = {0};
        int n = XPLMGetDatavi(r, v, 0, 1);
        if (n > 0) {
            out = v[0];
            return true;
        }
    }
    if (types & xplmType_FloatArray) {
        float v[1] = {0};
        int n = XPLMGetDatavf(r, v, 0, 1);
        if (n > 0) {
            out = static_cast<int>(std::llround(v[0]));
            return true;
        }
    }
    return false;
}

bool RouteReader::readFloat(const std::string& suffix, float& out) const {
    XPLMDataRef r = find(suffix);
    if (!r) return false;
    int types = XPLMGetDataRefTypes(r);
    if (types & xplmType_Float) {
        out = XPLMGetDataf(r);
        return true;
    }
    if (types & xplmType_Int) {
        out = static_cast<float>(XPLMGetDatai(r));
        return true;
    }
    if (types & xplmType_FloatArray) {
        float v[1] = {0};
        int n = XPLMGetDatavf(r, v, 0, 1);
        if (n > 0) {
            out = v[0];
            return true;
        }
    }
    if (types & xplmType_IntArray) {
        int v[1] = {0};
        int n = XPLMGetDatavi(r, v, 0, 1);
        if (n > 0) {
            out = static_cast<float>(v[0]);
            return true;
        }
    }
    return false;
}

bool RouteReader::readIntArray(const std::string& suffix,
                               std::vector<int>& out,
                               int maxCount) const {
    out.clear();
    XPLMDataRef r = find(suffix);
    if (!r) return false;
    int types = XPLMGetDataRefTypes(r);
    if (types & xplmType_Int) {
        out.push_back(XPLMGetDatai(r));
        return true;
    }
    if (types & xplmType_Float) {
        out.push_back(static_cast<int>(std::llround(XPLMGetDataf(r))));
        return true;
    }
    if (types & xplmType_IntArray) {
        out.resize(static_cast<size_t>(maxCount));
        int n = XPLMGetDatavi(r, out.data(), 0, maxCount);
        if (n < 0) n = 0;
        out.resize(static_cast<size_t>(n));
        return true;
    }
    if (types & xplmType_FloatArray) {
        std::vector<float> tmp(static_cast<size_t>(maxCount));
        int n = XPLMGetDatavf(r, tmp.data(), 0, maxCount);
        if (n < 0) n = 0;
        out.reserve(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) {
            out.push_back(static_cast<int>(std::llround(tmp[static_cast<size_t>(i)])));
        }
        return true;
    }
    return false;
}

bool RouteReader::readFloatArray(const std::string& suffix,
                                 std::vector<float>& out,
                                 int maxCount) const {
    out.clear();
    XPLMDataRef r = find(suffix);
    if (!r) return false;
    int types = XPLMGetDataRefTypes(r);
    if (types & xplmType_Float) {
        out.push_back(XPLMGetDataf(r));
        return true;
    }
    if (types & xplmType_Int) {
        out.push_back(static_cast<float>(XPLMGetDatai(r)));
        return true;
    }
    if (types & xplmType_FloatArray) {
        out.resize(static_cast<size_t>(maxCount));
        int n = XPLMGetDatavf(r, out.data(), 0, maxCount);
        if (n < 0) n = 0;
        out.resize(static_cast<size_t>(n));
        return true;
    }
    if (types & xplmType_IntArray) {
        std::vector<int> tmp(static_cast<size_t>(maxCount));
        int n = XPLMGetDatavi(r, tmp.data(), 0, maxCount);
        if (n < 0) n = 0;
        out.reserve(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) {
            out.push_back(static_cast<float>(tmp[static_cast<size_t>(i)]));
        }
        return true;
    }
    return false;
}

bool RouteReader::readBytes(const std::string& suffix,
                            std::vector<char>& out,
                            int maxCount) const {
    out.clear();
    XPLMDataRef r = find(suffix);
    if (!r) return false;
    int types = XPLMGetDataRefTypes(r);
    if (!(types & xplmType_Data)) return false;
    out.resize(static_cast<size_t>(maxCount));
    int n = XPLMGetDatab(r, out.data(), 0, maxCount);
    if (n < 0) n = 0;
    out.resize(static_cast<size_t>(n));
    return true;
}

int RouteReader::activeWptIndex() const {
    int idx = -1;
    if (!readInt("rteData/actWpt/idx", idx)) return -1;
    return idx;
}

void RouteReader::refresh(double elapsedSeconds, bool force) {
    if (!bound_) bind();

    // The FF757 may register its private route tree lazily.  Re-probe the names
    // occasionally instead of caching a permanent "not found".
    if (!hasRouteDatarefs() && elapsedSeconds - lastBindAttempt_ > 5.0) {
        bind();
        lastBindAttempt_ = elapsedSeconds;
        force = true;
    }

    // Drive the CDU reader only when the private route tree is unavailable.
    // That avoids pressing CDU page keys on builds that expose rteData.  Keep
    // driving it while a scan/restore is in progress so the CDU is left on its
    // original page and no page key is held down.
    if (!hasRouteDatarefs() || cdu_.scanning()) {
        cdu_.update(elapsedSeconds);
    }

    if (!hasRouteDatarefs()) {
        const RouteSnapshot& cduRoute = cdu_.route();
        if (cduRoute.valid) {
            route_ = cduRoute;
            report_ = "CDU LEGS";
            lastError_ = cdu_.lastError();
            captureBaseRoute();
        } else if (!route_.valid || !route_.sourceRoute) {
            route_.clear();
            lastError_ = cdu_.lastError();
        }
        return;
    }

    int activeNow = activeWptIndex();
    if (activeNow != lastActiveIdx_) {
        force = true;
    }

    if (!force && elapsedSeconds - lastFullRefresh_ < 1.0) {
        return;
    }

    lastFullRefresh_ = elapsedSeconds;
    lastActiveIdx_ = activeNow;
    rebuild();
    captureBaseRoute();
}

void RouteReader::captureBaseRoute() {
    if (!route_.valid) {
        baseRoute_.clear();
        baseValid_ = false;
        hasMergedRoute_ = false;
        cifpAssisted_ = false;
        cifpApplied_ = false;
        cifpChecked_ = false;
        cifpRouteSignature_.clear();
        cifpNote_.clear();
        return;
    }
    baseRoute_ = route_;
    baseValid_ = true;
    const std::string sig = routeSignature(baseRoute_);
    if (cifpChecked_ && sig == cifpRouteSignature_ && hasMergedRoute_) {
        route_ = mergedRoute_;
        cifpAssisted_ = true;
        cifpApplied_ = true;
        return;
    }
    hasMergedRoute_ = false;
    cifpAssisted_ = false;
    cifpApplied_ = false;
    cifpChecked_ = false;
    cifpRouteSignature_.clear();
    cifpNote_.clear();
}

void RouteReader::restoreBaseRoute() {
    if (baseValid_) route_ = baseRoute_;
    cifpAssisted_ = false;
    cifpApplied_ = false;
    report_ = hasRouteDatarefs() ? "rteData" : "CDU LEGS";
}

std::string RouteReader::routeSignature(const RouteSnapshot& route) const {
    std::string s;
    s.reserve(128);
    s += std::to_string(route.wptCount);
    s += '|';
    s += std::to_string(route.activeWpt);
    s += '|';
    s += std::to_string(route.legCount);
    const int n = std::min(12, static_cast<int>(route.wpts.size()));
    for (int i = 0; i < n; ++i) {
        s += '|';
        s += route.wpts[static_cast<size_t>(i)].id;
    }
    s += '|';
    s += route.activeId;
    return s;
}

void RouteReader::applyCifpProcedure(double lat, double lon, bool enabled,
                                     double elapsedSeconds) {
    if (!enabled) {
        if (cifpApplied_) restoreBaseRoute();
        cifpProcedureEnabled_ = false;
        return;
    }
    if (!cifpProcedureEnabled_) {
        cifpProcedureEnabled_ = true;
        cifpChecked_ = false;
    }
    if (!baseValid_ || !baseRoute_.valid) return;
    // Only assist the departure segment.  A route that has already sequenced
    // several fixes should not be re-detected against a SID at the nearby
    // airport.
    if (baseRoute_.activeWpt > 6) return;

    const std::string sig = routeSignature(baseRoute_);
    if (cifpChecked_ && sig == cifpRouteSignature_ &&
        elapsedSeconds - lastCifpAttempt_ < 5.0) {
        return;
    }
    cifpRouteSignature_ = sig;
    cifpChecked_ = true;
    lastCifpAttempt_ = elapsedSeconds;

    if (!cifpProcRootSet_) return;

    ProcedureSelection selection;
    const ProcedureRouteOptions options;
    if (!procBuilder_.detect(baseRoute_, lat, lon, selection, options)) {
        if (cifpApplied_) restoreBaseRoute();
        hasMergedRoute_ = false;
        cifpNote_ = "no CIFP departure match";
        if (lastCifpLog_ != cifpNote_) {
            Logf("CIFP procedure: %s", cifpNote_.c_str());
            lastCifpLog_ = cifpNote_;
        }
        return;
    }

    RouteSnapshot procRoute;
    ResolveStats stats;
    if (!procBuilder_.build(selection, procRoute, &stats)) {
        if (cifpApplied_) restoreBaseRoute();
        hasMergedRoute_ = false;
        cifpNote_ = "CIFP procedure build failed";
        if (lastCifpLog_ != cifpNote_) {
            Logf("CIFP procedure: %s", cifpNote_.c_str());
            lastCifpLog_ = cifpNote_;
        }
        return;
    }

    RouteSnapshot merged;
    if (!ProcedureRouteBuilder::merge(baseRoute_, procRoute, merged)) {
        if (cifpApplied_) restoreBaseRoute();
        hasMergedRoute_ = false;
        cifpNote_ = "CIFP route has no common fix with the FMS route";
        if (lastCifpLog_ != cifpNote_) {
            Logf("CIFP procedure: %s", cifpNote_.c_str());
            lastCifpLog_ = cifpNote_;
        }
        return;
    }

    route_ = merged;
    mergedRoute_ = merged;
    hasMergedRoute_ = true;
    cifpAssisted_ = true;
    cifpApplied_ = true;
    report_ = (hasRouteDatarefs() ? "rteData" : "CDU LEGS") + std::string(" + CIFP");
    char note[160];
    std::snprintf(note, sizeof(note), "%s %s, %d/%zu fixes, %d RF",
                  selection.ident.c_str(), selection.transition.c_str(),
                  stats.resolved, procRoute.wpts.size(), stats.rfCenters);
    cifpNote_ = note;
    if (lastCifpLog_ != cifpNote_) {
        Logf("CIFP procedure: %s", cifpNote_.c_str());
        lastCifpLog_ = cifpNote_;
    }
}

void RouteReader::rebuild() {
    route_.clear();
    lastError_.clear();

    int rteCount = 0;
    readInt("rteData/rteCount", rteCount);
    route_.rawRteCount = rteCount;

    std::vector<int> offsets;
    std::vector<int> types;
    std::vector<int> sizes;
    readIntArray("rteData/rteOffsets", offsets, 256);
    readIntArray("rteData/rteTypes", types, 256);
    readIntArray("rteData/rteSizes", sizes, 256);

    std::vector<float> lats;
    std::vector<float> lons;
    readFloatArray("rteData/wptsLat", lats, 2048);
    readFloatArray("rteData/wptsLon", lons, 2048);

    int wptCount = static_cast<int>(std::min(lats.size(), lons.size()));
    if (wptCount <= 0) {
        lastError_ = "route waypoint arrays are empty";
        return;
    }

    route_.wpts.resize(static_cast<size_t>(wptCount));
    for (int i = 0; i < wptCount; ++i) {
        RouteWaypoint& w = route_.wpts[static_cast<size_t>(i)];
        w.lat = lats[static_cast<size_t>(i)];
        w.lon = lons[static_cast<size_t>(i)];
        w.posValid = std::isfinite(w.lat) && std::isfinite(w.lon) &&
                     std::abs(w.lat) <= 90.0 && std::abs(w.lon) <= 180.0;
    }

    std::vector<int> idStarts;
    std::vector<int> idLens;
    std::vector<char> idBuffer;
    readIntArray("rteData/wptsIdStart", idStarts, 2048);
    readIntArray("rteData/wptsIdLen", idLens, 2048);
    readBytes("rteData/wptsId", idBuffer, 8192);
    parseIds(idStarts, idLens, idBuffer);

    // Estimated altitude can be a float array or a string buffer.
    std::vector<float> altFloats;
    bool altAsFloats = false;
    XPLMDataRef altRef = find("rteData/wptsEstimatedAlt");
    if (altRef) {
        int altTypes = XPLMGetDataRefTypes(altRef);
        if (altTypes & (xplmType_Float | xplmType_FloatArray |
                        xplmType_Int | xplmType_IntArray)) {
            altAsFloats = readFloatArray("rteData/wptsEstimatedAlt", altFloats, 2048);
        }
    }

    std::vector<int> altStarts;
    std::vector<int> altLens;
    std::vector<char> altBuffer;
    readIntArray("rteData/wptsEstimatedAltStart", altStarts, 2048);
    readIntArray("rteData/wptsEstimatedAltLen", altLens, 2048);
    readBytes("rteData/wptsEstimatedAlt", altBuffer, 8192);

    if (altAsFloats) {
        int n = static_cast<int>(std::min<size_t>(altFloats.size(), route_.wpts.size()));
        for (int i = 0; i < n; ++i) {
            int alt = static_cast<int>(std::llround(altFloats[static_cast<size_t>(i)]));
            if (alt > 0 && alt <= 100000) {
                route_.wpts[static_cast<size_t>(i)].altFt = alt;
                route_.wpts[static_cast<size_t>(i)].altValid = true;
                route_.wpts[static_cast<size_t>(i)].altConstraint = AltAt;
            }
        }
    } else {
        parseAltitudes(altStarts, altLens, altBuffer, false, altFloats);
    }

    int activeIdx = -1;
    const bool haveActiveIdx = readInt("rteData/actWpt/idx", activeIdx);
    if (!haveActiveIdx || activeIdx < 0 || activeIdx >= wptCount) {
        route_.valid = false;
        route_.sourceRoute = false;
        route_.activeWpt = -1;
        lastError_ = "active waypoint index missing or out of range";
        return;
    }
    route_.activeWpt = activeIdx;

    int legCount = 0;
    if (rteCount > 0) {
        legCount = std::min(rteCount, static_cast<int>(types.size()));
        legCount = std::min(legCount, static_cast<int>(offsets.size()));
    } else {
        legCount = std::min(static_cast<int>(types.size()),
                            static_cast<int>(offsets.size()));
    }

    if (legCount <= 0 || wptCount < 2) {
        buildFallbackFromWaypoints();
    } else {
        route_.legs.reserve(static_cast<size_t>(legCount));
        for (int i = 0; i < legCount; ++i) {
            int off = offsets[static_cast<size_t>(i)];
            int size = 0;
            if (i < static_cast<int>(sizes.size())) {
                size = sizes[static_cast<size_t>(i)];
            } else if (i + 1 < static_cast<int>(offsets.size())) {
                size = offsets[static_cast<size_t>(i + 1)] - off;
            } else {
                size = wptCount - off;
            }

            if (size <= 0) continue;

            int first = off;
            int last = off + size - 1;
            if (first < 0 || first >= wptCount) {
                // Some builds store a leg index rather than a waypoint index.
                if (off == i && size == 1) {
                    first = i;
                    last = i;
                } else {
                    continue;
                }
            }
            if (last >= wptCount) last = wptCount - 1;
            if (last < first) continue;

            RouteLeg leg;
            leg.type = (i < static_cast<int>(types.size())) ? types[static_cast<size_t>(i)] : 0;
            leg.firstWpt = first;
            leg.lastWpt = last;
            leg.complex = (last > first);
            route_.legs.push_back(leg);
        }
        if (route_.legs.empty()) buildFallbackFromWaypoints();
    }

    route_.legCount = static_cast<int>(route_.legs.size());
    route_.wptCount = wptCount;

    if (route_.legs.empty()) {
        route_.valid = false;
        lastError_ = "no route legs";
        return;
    }

    int legIdx = findLegContaining(route_, activeIdx);
    if (legIdx < 0 && activeIdx >= 0 && activeIdx < route_.legCount) {
        legIdx = activeIdx;
        route_.activeWpt = route_.legs[static_cast<size_t>(legIdx)].lastWpt;
    }
    if (legIdx < 0) {
        for (int i = 0; i < route_.legCount; ++i) {
            if (route_.legs[static_cast<size_t>(i)].lastWpt <= activeIdx) {
                legIdx = i;
            }
        }
    }
    if (legIdx < 0) legIdx = 0;

    route_.activeLeg = legIdx;
    route_.nextLeg = (legIdx + 1 < route_.legCount) ? legIdx + 1 : -1;

    // The active waypoint is the end of the active leg.
    route_.activeWpt = route_.legs[static_cast<size_t>(legIdx)].lastWpt;

    std::string activeId;
    readBytes("rteData/actWpt/id", idBuffer, 128);
    if (!idBuffer.empty()) {
        activeId = trim(std::string(idBuffer.data(), idBuffer.size()));
    }
    if (activeId.empty()) {
        const RouteWaypoint* w = route_.waypoint(route_.activeWpt);
        if (w) activeId = w->id;
    }
    route_.activeId = activeId;

    if (route_.nextLeg >= 0) {
        const RouteWaypoint* w = route_.waypoint(
            route_.legs[static_cast<size_t>(route_.nextLeg)].lastWpt);
        if (w) route_.nextId = w->id;
    }

    float vtk = 0.0f;
    if (readFloat("rteData/vtk", vtk)) route_.vtk = vtk;

    bool coordsOk = true;
    for (const RouteWaypoint& w : route_.wpts) {
        if (!w.posValid) {
            coordsOk = false;
            break;
        }
    }

    if (!coordsOk) {
        route_.valid = false;
        lastError_ = "route contains invalid lat/lon";
        return;
    }

    route_.valid = true;
    route_.sourceRoute = true;

    // RF arcs come from the CIFP procedure database, not from rteData; match
    // and annotate them so guidance can fly a true constant-radius path.
    cdu_.annotateRfArcs(route_);
}

void RouteReader::buildFallbackFromWaypoints() {
    route_.legs.clear();
    for (int i = 0; i < static_cast<int>(route_.wpts.size()); ++i) {
        RouteLeg leg;
        leg.type = 0;
        leg.firstWpt = i;
        leg.lastWpt = i;
        leg.complex = false;
        route_.legs.push_back(leg);
    }
}

void RouteReader::parseIds(std::vector<int>& starts, std::vector<int>& lens,
                           const std::vector<char>& buffer) {
    if (buffer.empty()) return;

    const size_t n = std::min(route_.wpts.size(), starts.size());
    size_t m = std::min(n, lens.size());
    for (size_t i = 0; i < m; ++i) {
        std::string id = sliceString(buffer, starts[i], lens[i]);
        if (!id.empty()) route_.wpts[i].id = id;
    }

    if (m == 0) {
        // No start/len table: treat the buffer as newline-separated IDs.
        std::string all(buffer.data(), buffer.size());
        size_t pos = 0;
        int idx = 0;
        while (pos < all.size() && idx < static_cast<int>(route_.wpts.size())) {
            size_t nl = all.find_first_of("\r\n", pos);
            std::string line = trim(all.substr(pos, nl == std::string::npos
                                                       ? std::string::npos
                                                       : nl - pos));
            if (!line.empty()) route_.wpts[static_cast<size_t>(idx++)].id = line;
            if (nl == std::string::npos) break;
            pos = nl + 1;
        }
    }
}

bool RouteReader::parseAltitudes(std::vector<int>& starts,
                                 std::vector<int>& lens,
                                 const std::vector<char>& buffer,
                                 bool altAsFloatArray,
                                 const std::vector<float>& altFloats) {
    if (altAsFloatArray) {
        int n = static_cast<int>(std::min<size_t>(altFloats.size(), route_.wpts.size()));
        for (int i = 0; i < n; ++i) {
            int alt = static_cast<int>(std::llround(altFloats[static_cast<size_t>(i)]));
            if (alt > 0 && alt <= 100000) {
                route_.wpts[static_cast<size_t>(i)].altFt = alt;
                route_.wpts[static_cast<size_t>(i)].altValid = true;
                route_.wpts[static_cast<size_t>(i)].altConstraint = AltAt;
            }
        }
        return n > 0;
    }

    if (buffer.empty() || starts.empty() || lens.empty()) return false;

    const size_t n = std::min(route_.wpts.size(),
                              std::min(starts.size(), lens.size()));
    bool any = false;
    for (size_t i = 0; i < n; ++i) {
        std::string text = sliceString(buffer, starts[i], lens[i]);
        AltConstraint altType = AltAt;
        int alt = parseAltitudeString(text, altType);
        if (alt != INT_MIN) {
            route_.wpts[i].altFt = alt;
            route_.wpts[i].altValid = true;
            route_.wpts[i].altConstraint = altType;
            any = true;
        }
    }
    return any;
}

} // namespace lvnv
