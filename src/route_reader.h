#pragma once

#include "cdu_reader.h"
#include "procedure_parser.h"
#include "procedure_route.h"
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

    // Overlay the detected CIFP departure (procedure prefix + RF geometry) on
    // the FMS/CDU route.  Called after the aircraft state is read; the base
    // route is kept so a disabled/failed overlay can restore it.
    void applyCifpProcedure(double lat, double lon, bool enabled,
                            double elapsedSeconds);
    void setCifpProcedureEnabled(bool enabled) { cifpProcedureEnabled_ = enabled; }
    bool cifpProcedureAssisted() const { return cifpAssisted_; }
    const std::string& cifpProcedureNote() const { return cifpNote_; }

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
    void captureBaseRoute();
    void restoreBaseRoute();
    std::string routeSignature(const RouteSnapshot& route) const;

    mutable std::map<std::string, XPLMDataRef> refs_;
    CduRouteReader cdu_;
    RouteSnapshot route_;
    RouteSnapshot baseRoute_;
    RouteSnapshot mergedRoute_;
    bool baseValid_ = false;
    bool hasMergedRoute_ = false;
    CifpProcedureDatabase cifpProc_;
    ProcedureRouteBuilder procBuilder_{&cifpProc_};
    bool cifpProcRootSet_ = false;
    bool cifpProcedureEnabled_ = true;
    bool cifpAssisted_ = false;
    bool cifpApplied_ = false;
    bool cifpChecked_ = false;
    std::string cifpRouteSignature_;
    std::string cifpNote_;
    std::string lastCifpLog_;
    // Last CDU route signature copied into route_/baseRoute_.  Avoids copying
    // the whole snapshot every frame when the CDU content has not changed.
    std::string lastCduRouteSignature_;
    double lastCifpAttempt_ = -1000.0;
    std::string report_;
    std::string lastError_;
    double lastFullRefresh_ = -1000.0;
    double lastBindAttempt_ = -1000.0;
    int lastActiveIdx_ = -2;
    bool bound_ = false;
};

} // namespace lvnv
