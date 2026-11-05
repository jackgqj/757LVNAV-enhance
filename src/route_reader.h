#pragma once

#include "cdu_reader.h"
#include "route_model.h"

#include <map>
#include <string>
#include <vector>

#include "XPLMDataAccess.h"

namespace lvnv {

class RouteReader {
public:
    RouteReader();

    void bind();
    void reset();

    // Reads the FF757 route datarefs.  The full route is refreshed at a low
    // rate; activeWptIndex() can be polled every frame.
    void refresh(double elapsedSeconds, bool force = false);

    int activeWptIndex() const;
    const RouteSnapshot& route() const { return route_; }
    bool hasRouteDatarefs() const;
    const std::string& report() const { return report_; }
    const std::string& lastError() const { return lastError_; }

    // Queue a CDU LEGS-page rescan (menu action).
    void requestScan() { cdu_.requestScan(); }
    bool cduScanning() const { return cdu_.scanning(); }
    bool cduBased() const { return cdu_.hasCache() && !hasRouteDatarefs(); }
    std::string cduScanState() const { return cdu_.scanStateName(); }
    int cduCacheLegs() const { return cdu_.cachedLegCount(); }
    int cduCachePages() const { return cdu_.cachedPageCount(); }
    const std::string& cduError() const { return cdu_.lastError(); }
    void setRescanInterval(double seconds) { cdu_.setRescanInterval(seconds); }
    void setScanOnExecOnly(bool enabled) { cdu_.setScanOnExecOnly(enabled); }

private:
    XPLMDataRef find(const std::string& suffix) const;
    bool readInt(const std::string& suffix, int& out) const;
    bool readFloat(const std::string& suffix, float& out) const;
    bool readIntArray(const std::string& suffix, std::vector<int>& out, int maxCount) const;
    bool readFloatArray(const std::string& suffix, std::vector<float>& out, int maxCount) const;
    bool readBytes(const std::string& suffix, std::vector<char>& out, int maxCount) const;

    void rebuild();
    void buildFallbackFromWaypoints();
    void parseIds(std::vector<int>& starts, std::vector<int>& lens,
                  const std::vector<char>& buffer);
    bool parseAltitudes(std::vector<int>& starts, std::vector<int>& lens,
                        const std::vector<char>& buffer, bool altAsFloatArray,
                        const std::vector<float>& altFloats);

    mutable std::map<std::string, XPLMDataRef> refs_;
    CduRouteReader cdu_;
    RouteSnapshot route_;
    std::string report_;
    std::string lastError_;
    double lastFullRefresh_ = -1000.0;
    double lastBindAttempt_ = -1000.0;
    int lastActiveIdx_ = -2;
    bool bound_ = false;
};

} // namespace lvnv
