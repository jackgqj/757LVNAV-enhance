#pragma once

#include "cdu_parser.h"
#include "cifp_parser.h"
#include "nav_database.h"
#include "route_model.h"

#include <string>
#include <vector>

#include "XPLMDataAccess.h"

namespace lvnv {

// Reads the FF757 CDU LEGS page and turns it into a RouteSnapshot.
// The private 1-sim/.../rteData tree is not exposed in the release build, so
// the official CDU display datarefs are used instead.
class CduRouteReader {
public:
    CduRouteReader();

    // Called every frame.  Drives the scan/restore state machine and refreshes
    // the cached route at a low rate.
    void update(double elapsedSeconds);

    // Request a rescan at the next safe opportunity (menu command).
    void requestScan() { scanRequested_ = true; }

    // Periodic full-page rescan interval.  Content changes visible on the
    // current LEGS page are detected every poll and trigger a rescan earlier.
    void setRescanInterval(double seconds);

    // When true, only scan a newly executed (ACT) route: an initial ACT page,
    // a MOD -> ACT transition (EXEC), a changed active route name, or a manual
    // request.  Route edits while the page shows MOD never trigger a scan.
    void setScanOnExecOnly(bool enabled) { scanOnExecOnly_ = enabled; }

    // Fills rfArc geometry on legs that match a CIFP RF row.  Used internally
    // by buildSnapshot(); also exposed so RouteReader can annotate its
    // rteData-based snapshot with the same nav/cifp databases.
    void annotateRfArcs(RouteSnapshot& route) { AnnotateRfArcs(route, cifp_, nav_); }

    const RouteSnapshot& route() const { return route_; }
    const std::string& lastError() const { return lastError_; }
    bool scanning() const { return state_ != ScanState::Idle; }
    bool hasCache() const { return cacheValid_; }
    int cachedLegCount() const { return cacheValid_ ? static_cast<int>(cache_.legs.size()) : 0; }
    int cachedPageCount() const { return cacheValid_ ? cache_.pageCount : 0; }
    const char* scanStateName() const;

private:
    struct CduRefs {
        XPLMDataRef lines = nullptr;
        XPLMDataRef width = nullptr;
        XPLMDataRef symbols = nullptr;
        XPLMDataRef prevPage = nullptr;
        XPLMDataRef nextPage = nullptr;
        bool complete() const { return lines && width && symbols; }
        bool canPage() const { return prevPage && nextPage; }
    };

    enum class ScanState { Idle, SeekFirst, Collect, Restore };

    void bindRefs();
    void releaseButton(double elapsed);
    void press(XPLMDataRef ref, double elapsed);
    bool readScreen(CduRefs& refs, std::vector<std::string>& rows);
    void pollIdle(double elapsed);
    void updateScan(double elapsed);
    void startScan(const CduPage& page, CduRefs& refs, double elapsed);
    void finishScan(double elapsed);
    void abortScan(double elapsed);
    void buildSnapshot();
    void advanceActiveIfNeeded(double elapsed);
    void updateActiveFromScreen(const CduPage& page, double elapsed);
    bool screenActiveFresh() const;
    bool pageMatchesCache(const CduPage& page) const;
    bool syncActiveFromAirnav(double airnavDistanceNm, double bearingDeg);
    CduRefs* chooseSide(std::vector<std::string>& rows, CduPage& page);
    void readAircraftPosition(double& lat, double& lon);
    void readAirnav(double& distanceNm, int& fromTo);

    CduRefs left_;
    CduRefs right_;
    XPLMDataRef vtk_ = nullptr;
    XPLMDataRef airnavDis_ = nullptr;
    XPLMDataRef airnavFromTo_ = nullptr;
    XPLMDataRef airnavDtk_ = nullptr;
    XPLMDataRef airnavBrg_ = nullptr;
    XPLMDataRef yAgl_ = nullptr;
    XPLMDataRef latRef_ = nullptr;
    XPLMDataRef lonRef_ = nullptr;
    XPLMDataRef magVar_ = nullptr;

    NavDatabase nav_;
    CifpDatabase cifp_;
    bool navRootSet_ = false;

    ScanState state_ = ScanState::Idle;
    CduRefs* scanSide_ = nullptr;
    CduPage scanPage_;
    std::vector<CduPage> scannedPages_;
    int originalPage_ = 1;
    int scanPageCount_ = 1;
    int lastCollectedPage_ = 0;
    double scanStarted_ = 0.0;
    double restoreStarted_ = 0.0;
    double lastPoll_ = 0.0;
    double lastBind_ = -1000.0;
    double lastActivePoll_ = 0.0;
    // Last active index pushed into the snapshot by the geometric sequencing
    // fallback, so a silent advance does not rebuild every poll.
    int lastActiveBuilt_ = -1;
    double lastAction_ = 0.0;
    double lastPress_ = -100.0;
    double releaseAt_ = 0.0;
    double rescanIntervalSec_ = 0.0;
    int collectStall_ = 0;
    int seekStall_ = 0;
    XPLMDataRef pressedRef_ = nullptr;
    bool scanRequested_ = false;
    bool pendingExec_ = false;   // a MOD route is waiting for EXEC
    bool scanOnExecOnly_ = true;
    std::string lastPageSignature_;  // content seen at the last scan request

    CduRoute cache_;
    RouteSnapshot route_;
    bool cacheValid_ = false;
    int activeIndex_ = 0;
    // CDU LEGS page 1 line 1 is the waypoint the FMS is actually flying to.
    // While that page is being read it is the highest-priority active-index
    // source; the geometric sequencing fallback and the distance heuristic in
    // buildSnapshot() must not run ahead of it, otherwise they advance a few
    // legs, page 1 pulls the index back, and the two repeat (live 1.2.24 log:
    // "CDU active from page 1: 2 (DM) -> 0 (LXA)" every few seconds).
    int screenActiveIndex_ = -1;
    double screenActiveAt_ = -1000.0;
    double lastScan_ = -1000.0;
    double lastAirnavDis_ = 0.0;
    bool haveAirnavDis_ = false;
    int lastReadBytes_ = -1;   // bytes returned by the last CDU display read
    int lastReadTotal_ = 0;    // expected bytes for that display
    std::string lastError_;
};

} // namespace lvnv
