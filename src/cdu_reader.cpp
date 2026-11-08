#include "cdu_reader.h"

#include "geo.h"
#include "log.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "XPLMUtilities.h"
#include "XPLMProcessing.h"

namespace lvnv {

namespace {

constexpr int kMaxScanPages = 32;
constexpr double kActionIntervalSec = 0.35;
constexpr double kScanTimeoutSec = 25.0;
constexpr double kRestoreTimeoutSec = 8.0;
constexpr int kMaxStalledPresses = 8;

int RdI(XPLMDataRef ref) {
    if (!ref) return 0;
    const int types = XPLMGetDataRefTypes(ref);
    if (types & xplmType_Int) return XPLMGetDatai(ref);
    if (types & xplmType_Float) return static_cast<int>(XPLMGetDataf(ref));
    if (types & xplmType_Double) return static_cast<int>(XPLMGetDatad(ref));
    return 0;
}

float RdF(XPLMDataRef ref) {
    if (!ref) return 0.0f;
    const int types = XPLMGetDataRefTypes(ref);
    if (types & xplmType_Float) return XPLMGetDataf(ref);
    if (types & xplmType_Double) return static_cast<float>(XPLMGetDatad(ref));
    if (types & xplmType_Int) return static_cast<float>(XPLMGetDatai(ref));
    return 0.0f;
}

double RdD(XPLMDataRef ref) {
    if (!ref) return 0.0;
    const int types = XPLMGetDataRefTypes(ref);
    if (types & xplmType_Double) return XPLMGetDatad(ref);
    if (types & xplmType_Float) return static_cast<double>(XPLMGetDataf(ref));
    if (types & xplmType_Int) return static_cast<double>(XPLMGetDatai(ref));
    return 0.0;
}

bool ValidLatLon(double lat, double lon) {
    return std::isfinite(lat) && std::isfinite(lon) && std::abs(lat) <= 90.0 &&
           std::abs(lon) <= 180.0 && !(std::abs(lat) < 0.00001 && std::abs(lon) < 0.00001);
}

// Content signature of a displayed LEGS page.  Used to scan once per visible
// page change and to avoid rescanning the same MOD page that was already
// scanned and rejected.
std::string PageSignature(const CduPage& page) {
    std::string s;
    s.reserve(192);
    s += page.title;
    s += '|';
    s += page.routeName;
    s += '|';
    s += std::to_string(page.page);
    s += '/';
    s += std::to_string(page.pageCount);
    for (const CduLeg& leg : page.legs) {
        s += '|';
        s += leg.id;
        s += ',';
        s += std::to_string(leg.altFt);
        s += static_cast<char>('0' + static_cast<int>(leg.altConstraint));
        s += ',';
        s += std::to_string(static_cast<int>(leg.legDistanceNm * 10.0));
        if (leg.hold) s += 'H';
    }
    return s;
}

// True when the aircraft has already passed a waypoint along its outbound
// leg.  Used to tell a genuinely stale CDU page from a page that is merely
// lagging behind the geometric sequencing.
bool WaypointBehind(const RouteSnapshot& route, int idx, double lat, double lon) {
    const RouteWaypoint* cur = route.waypoint(idx);
    const RouteWaypoint* nxt = route.waypoint(idx + 1);
    if (!cur || !nxt || !cur->posValid || !nxt->posValid) return false;
    const double crs = bearingTrue(cur->lat, cur->lon, nxt->lat, nxt->lon);
    const double midLatRad = deg2rad((lat + cur->lat) * 0.5);
    const double northNm = (lat - cur->lat) * 60.0;
    const double eastNm = (lon - cur->lon) * 60.0 * std::cos(midLatRad);
    const double alongNm =
        northNm * std::cos(deg2rad(crs)) + eastNm * std::sin(deg2rad(crs));
    return std::isfinite(alongNm) && alongNm > 0.3;
}

} // namespace

CduRouteReader::CduRouteReader() {
    // No XPLM calls here.  This constructor runs as a static initializer
    // (global object), i.e. inside DllMain during LoadLibrary.  Calling XPLM
    // from DllMain is undefined and was failing with "DLL initialization
    // routine failed" (error 1114) on some machines.  Refs are bound lazily
    // from update() on the first flight-loop tick.
}

void CduRouteReader::bindRefs() {
    left_.lines = XPLMFindDataRef("1-sim/cduL/display/numOfLines");
    left_.width = XPLMFindDataRef("1-sim/cduL/display/numOfSymbolsInLine");
    left_.symbols = XPLMFindDataRef("1-sim/cduL/display/symbols");
    left_.prevPage = XPLMFindDataRef("757Avionics/CDU/prev_page");
    left_.nextPage = XPLMFindDataRef("757Avionics/CDU/next_page");

    right_.lines = XPLMFindDataRef("1-sim/cduR/display/numOfLines");
    right_.width = XPLMFindDataRef("1-sim/cduR/display/numOfSymbolsInLine");
    right_.symbols = XPLMFindDataRef("1-sim/cduR/display/symbols");
    right_.prevPage = XPLMFindDataRef("757Avionics/CDU2/prev_page");
    right_.nextPage = XPLMFindDataRef("757Avionics/CDU2/next_page");

    vtk_ = XPLMFindDataRef("757Avionics/fms/vtk");
    airnavDis_ = XPLMFindDataRef("757Avionics/fms/AirnavDis");
    airnavFromTo_ = XPLMFindDataRef("757Avionics/fms/AirnavFromTo");
    airnavDtk_ = XPLMFindDataRef("757Avionics/fms/AirnavDtk");
    airnavBrg_ = XPLMFindDataRef("757Avionics/fms/AirnavBrg");
    yAgl_ = XPLMFindDataRef("sim/flightmodel/position/y_agl");
    latRef_ = XPLMFindDataRef("sim/flightmodel/position/latitude");
    lonRef_ = XPLMFindDataRef("sim/flightmodel/position/longitude");
    magVar_ = XPLMFindDataRef("sim/flightmodel/position/magnetic_variation");

    if (!navRootSet_) {
        char root[512] = {0};
        XPLMGetSystemPath(root);
        if (root[0]) {
            nav_.setRootPath(root);
            cifp_.setRootPath(root);
            navRootSet_ = true;
        }
    }
}

void CduRouteReader::setRescanInterval(double seconds) {
    if (seconds >= 0.0 && seconds <= 600.0) rescanIntervalSec_ = seconds;
}

const char* CduRouteReader::scanStateName() const {
    switch (state_) {
    case ScanState::Idle: return "idle";
    case ScanState::SeekFirst: return "seek-first";
    case ScanState::Collect: return "collect";
    case ScanState::Restore: return "restore";
    }
    return "unknown";
}

void CduRouteReader::releaseButton(double elapsed) {
    if (pressedRef_ && elapsed >= releaseAt_) {
        XPLMSetDatai(pressedRef_, 0);
        pressedRef_ = nullptr;
    }
}

void CduRouteReader::press(XPLMDataRef ref, double elapsed) {
    if (!ref) return;
    if (elapsed - lastPress_ < 0.35) return;
    if (pressedRef_) {
        XPLMSetDatai(pressedRef_, 0);
        pressedRef_ = nullptr;
    }
    XPLMSetDatai(ref, 1);
    pressedRef_ = ref;
    releaseAt_ = elapsed + 0.06;
    lastPress_ = elapsed;
}

bool CduRouteReader::readScreen(CduRefs& refs, std::vector<std::string>& rows) {
    lastReadBytes_ = -1;
    lastReadTotal_ = 0;
    if (!refs.complete()) return false;
    int lines = RdI(refs.lines);
    int width = RdI(refs.width);
    lines = std::max(1, std::min(lines, 32));
    width = std::max(1, std::min(width, 64));
    const int total = lines * width;
    // Size the buffer for a full 32x64 screen regardless of what the line/width
    // datarefs currently report.  A custom CDU display accessor that ignores
    // inMaxLength (right after a page change) could otherwise write past the
    // reported size and corrupt the heap; the resulting crash surfaced later,
    // inside a string helper during the route rebuild.
    constexpr size_t kMaxScreenBytes = 32 * 64;
    std::vector<char> buffer(kMaxScreenBytes, ' ');
    const int bytes = XPLMGetDatab(refs.symbols, buffer.data(), 0, total);
    lastReadBytes_ = bytes;
    lastReadTotal_ = total;
    if (bytes < total) return false;

    rows.resize(static_cast<size_t>(lines));
    for (int r = 0; r < lines; ++r) {
        std::string& line = rows[static_cast<size_t>(r)];
        line.assign(static_cast<size_t>(width), ' ');
        for (int c = 0; c < width; ++c) {
            char ch = buffer[static_cast<size_t>(r * width + c)];
            if (ch < 32 || ch > 126) ch = ' ';
            line[static_cast<size_t>(c)] = ch;
        }
    }
    return true;
}

CduRouteReader::CduRefs* CduRouteReader::chooseSide(std::vector<std::string>& rows,
                                                    CduPage& page) {
    // Prefer the right (second) CDU so the captain's left CDU stays free.
    // Fall back to the left CDU only when the right one is not on a LEGS page.
    bool fullRead = false;
    int bestBytes = -1;
    int bestTotal = 0;
    for (CduRefs* side : {&right_, &left_}) {
        std::vector<std::string> candidate;
        if (!readScreen(*side, candidate)) {
            if (lastReadBytes_ > bestBytes) {
                bestBytes = lastReadBytes_;
                bestTotal = lastReadTotal_;
            }
            continue;
        }
        fullRead = true;
        if (!IsCduLegsPage(candidate)) continue;
        CduPage parsed = ParseCduLegsPage(candidate);
        if (!parsed.valid) continue;
        rows = std::move(candidate);
        page = std::move(parsed);
        lastError_.clear();
        return side;
    }

    if (fullRead) {
        lastError_ = "CDU is not showing a LEGS page";
    } else if (bestBytes >= 0 && bestTotal > 0) {
        char msg[128];
        std::snprintf(msg, sizeof(msg),
                      "CDU display short read (%d/%d bytes); open a CDU screen",
                      bestBytes, bestTotal);
        lastError_ = msg;
    } else {
        lastError_ = "CDU display datarefs missing";
    }
    return nullptr;
}

void CduRouteReader::readAircraftPosition(double& lat, double& lon) {
    lat = RdD(latRef_);
    lon = RdD(lonRef_);
}

void CduRouteReader::readAirnav(double& distanceNm, int& fromTo) {
    distanceNm = RdF(airnavDis_);
    fromTo = RdI(airnavFromTo_);
}

// Compares the currently displayed LEGS page against the cached route.  This
// catches route edits (altitude changes, inserted/deleted legs) without paging
// through all six pages.  The FF757 may start page 1 at the active waypoint,
// so the first displayed ident is located anywhere in the cache first.
bool CduRouteReader::pageMatchesCache(const CduPage& page) const {
    if (!cacheValid_ || !page.valid || page.legs.empty()) return true;
    // Do not compare pageCount here: the last page of a route can be shorter
    // than the cached page count and would otherwise trigger endless rescans.

    const std::string& firstId = page.legs.front().id;
    size_t start = cache_.legs.size();
    for (size_t i = 0; i < cache_.legs.size(); ++i) {
        if (cache_.legs[i].id == firstId) {
            start = i;
            break;
        }
    }
    if (start >= cache_.legs.size()) return false;

    for (size_t i = 0; i < page.legs.size(); ++i) {
        const size_t idx = start + i;
        if (idx >= cache_.legs.size()) return false;
        const CduLeg& displayed = page.legs[i];
        const CduLeg& cached = cache_.legs[idx];
        if (displayed.id != cached.id) return false;
        if (displayed.altFt > 0 && cached.altFt > 0 &&
            std::abs(displayed.altFt - cached.altFt) > 50) {
            return false;
        }
        if (displayed.legDistanceNm > 0.05 && cached.legDistanceNm > 0.05 &&
            std::abs(displayed.legDistanceNm - cached.legDistanceNm) > 0.2) {
            return false;
        }
    }
    return true;
}

// The FF757's AirnavDis/AirnavBrg describe its own active waypoint.  Matching
// both distance and bearing against the cached route is a much stronger
// active-index source than a one-waypoint-at-a-time distance heuristic, and it
// works even while the CDU is left on a page other than LEGS page 1.  Without
// this, passing a fix could leave the plugin one or more legs behind, the
// route course then disagreed with the GPS CDI, and LNAV fell back to CDI
// steering.
bool CduRouteReader::syncActiveFromAirnav(double airnavDistanceNm, double bearingDeg) {
    if (!cacheValid_ || route_.wpts.empty()) return false;

    double lat = 0.0;
    double lon = 0.0;
    readAircraftPosition(lat, lon);
    if (!ValidLatLon(lat, lon)) return false;

    const double magVar = RdF(magVar_);
    const int count = static_cast<int>(route_.wpts.size());
    int best = -1;
    double bestScore = 1.0e30;
    double bestDistDiff = 0.0;
    double bestBrgDiff = 0.0;

    // The FF757 computes pseudo-fixes such as (INTC) on top of an existing
    // waypoint, so two route entries can sit at the same coordinates.  Distance
    // and bearing alone then pick the lowest index, and the active leg jumps
    // backwards: live, active moved from UGOVA (index 2) to (INTC) (index 0)
    // at the same position, the CIFP annotated the stale leg with the ZULS RF
    // arc LS987, and LNAV flew 12.8 NM off the real course.  Collect every
    // near-tied candidate and keep the one closest to the current active
    // index instead of rewinding.
    struct Candidate {
        int index;
        double score;
        double distDiff;
        double brgDiff;
    };
    std::vector<Candidate> candidates;

    for (int i = 0; i < count; ++i) {
        const RouteWaypoint& w = route_.wpts[static_cast<size_t>(i)];
        if (!w.posValid) continue;
        const double d = distanceNm(lat, lon, w.lat, w.lon);
        if (!std::isfinite(d)) continue;
        const double dDiff = std::abs(d - airnavDistanceNm);
        if (dDiff > 2.0) continue;

        const double brgTrue = bearingTrue(lat, lon, w.lat, w.lon);
        const double brgDiffTrue = std::abs(normalize180(brgTrue - bearingDeg));
        const double brgDiffMag = std::abs(normalize180(brgTrue - (bearingDeg + magVar)));
        const double bDiff = std::min(brgDiffTrue, brgDiffMag);
        if (bDiff > 35.0) continue;

        const double score = dDiff * 10.0 + bDiff * 0.1;
        candidates.push_back(Candidate{i, score, dDiff, bDiff});
        if (score < bestScore) {
            bestScore = score;
            bestDistDiff = dDiff;
            bestBrgDiff = bDiff;
            best = i;
        }
    }

    if (best < 0 || bestDistDiff > 1.5 || bestBrgDiff > 25.0) return false;

    // Tie-break: 0.5 score is 0.05 NM or 5 deg, enough to treat two fixes at
    // the same point as equal without merging genuinely distinct waypoints.
    if (activeIndex_ >= 0 && activeIndex_ < count) {
        int chosen = best;
        double chosenGap = std::abs(best - activeIndex_);
        for (const Candidate& c : candidates) {
            if (c.score > bestScore + 0.5 || c.distDiff > 1.5 || c.brgDiff > 25.0) {
                continue;
            }
            const double gap = std::abs(c.index - activeIndex_);
            if (gap < chosenGap) {
                chosen = c.index;
                chosenGap = gap;
                bestDistDiff = c.distDiff;
                bestBrgDiff = c.brgDiff;
            }
        }
        best = chosen;
    }

    if (best != activeIndex_) {
        Logf("CDU active sync: %d (%s) -> %d (%s) airnavDist=%.2f brg=%.1f dDiff=%.2f bDiff=%.1f",
             activeIndex_,
             (activeIndex_ >= 0 && activeIndex_ < count)
                 ? route_.wpts[static_cast<size_t>(activeIndex_)].id.c_str()
                 : "-",
             best, route_.wpts[static_cast<size_t>(best)].id.c_str(),
             airnavDistanceNm, bearingDeg, bestDistDiff, bestBrgDiff);
        activeIndex_ = best;
        buildSnapshot();
    }
    return true;
}

void CduRouteReader::update(double elapsedSeconds) {
    const double elapsed = elapsedSeconds;
    if (!navRootSet_) bindRefs();
    if (elapsed - lastBind_ > 5.0) {
        bindRefs();
        lastBind_ = elapsed;
    }
    releaseButton(elapsed);

    if (state_ == ScanState::Idle) {
        pollIdle(elapsed);
        advanceActiveIfNeeded(elapsed);
    } else {
        updateScan(elapsed);
    }
}

void CduRouteReader::pollIdle(double elapsed) {
    if (elapsed - lastPoll_ < 0.5) return;
    lastPoll_ = elapsed;

    std::vector<std::string> rows;
    CduPage page;
    CduRefs* side = chooseSide(rows, page);
    if (!side) {
        // chooseSide has set lastError_ with the specific reason (no LEGS
        // page, short display read, or missing display datarefs).
        return;
    }

    // Optional EXEC-only mode (kept for users who prefer it).  In the default
    // content-change mode a MOD page is scanned once if its content changes,
    // but the scanner keeps the previous ACT cache if the scan result is MOD.
    if (scanOnExecOnly_ && !page.activeRoute) {
        pendingExec_ = true;
        lastError_ = "route is MOD; waiting for EXEC";
        return;
    }

    const std::string signature = PageSignature(page);
    const bool pageChangedSinceLastScan = signature != lastPageSignature_;
    const bool routeChanged = cacheValid_ && !cache_.routeName.empty() &&
                              !page.routeName.empty() &&
                              page.routeName != cache_.routeName;
    const bool contentChanged = (!cacheValid_ || !pageMatchesCache(page)) &&
                                pageChangedSinceLastScan;
    const bool periodic = cacheValid_ && rescanIntervalSec_ > 0.0 &&
                          elapsed - lastScan_ > rescanIntervalSec_;

    bool needScan;
    if (scanOnExecOnly_) {
        needScan = !cacheValid_ || scanRequested_ || pendingExec_ || routeChanged;
    } else {
        // Content-change mode: one scan per visible page change.  The same
        // rejected MOD page is not rescanned until its content changes.
        needScan = scanRequested_ || contentChanged || periodic || routeChanged;
    }

    if (needScan) {
        if (!side->canPage()) {
            lastError_ = "CDU page keys unavailable; cannot scan";
            scanRequested_ = false;
            pendingExec_ = false;
            return;
        }
        lastPageSignature_ = signature;
        const char* reason = contentChanged ? "page-content"
                             : !cacheValid_ ? "initial"
                             : pendingExec_ ? "exec"
                             : scanRequested_ ? "requested"
                             : routeChanged ? "route-name"
                             : "periodic";
        Logf("CDU scan queued (%s): page %d/%d route=%s", reason, page.page,
             page.pageCount, page.routeName.empty() ? "-" : page.routeName.c_str());
        startScan(page, *side, elapsed);
        return;
    }

    updateActiveFromScreen(page, elapsed);
}

void CduRouteReader::startScan(const CduPage& page, CduRefs& refs, double elapsed) {
    scanSide_ = &refs;
    scanPage_ = page;
    originalPage_ = std::max(1, page.page);
    scanPageCount_ = std::clamp(page.pageCount, 1, kMaxScanPages);
    scannedPages_.clear();
    lastCollectedPage_ = 0;
    scanStarted_ = elapsed;
    restoreStarted_ = elapsed;
    lastAction_ = elapsed;
    collectStall_ = 0;
    seekStall_ = 0;
    scanRequested_ = false;
    pendingExec_ = false;
    lastError_ = "scanning CDU LEGS pages";

    Logf("CDU scan begin: side=%s page=%d/%d route=%s",
         scanSide_ == &right_ ? "R" : "L", originalPage_, scanPageCount_,
         scanPage_.routeName.empty() ? "-" : scanPage_.routeName.c_str());

    if (scanPageCount_ <= 1 && page.page <= 1) {
        scannedPages_.push_back(page);
        lastCollectedPage_ = 1;
        state_ = ScanState::Restore;
        restoreStarted_ = elapsed;
    } else {
        state_ = ScanState::SeekFirst;
    }
}

void CduRouteReader::updateScan(double elapsed) {
    if (!scanSide_ || elapsed - lastAction_ < kActionIntervalSec) return;

    // Hard overall limit.  A failed page key or a CDU that stops updating must
    // never leave the reader permanently in the scanning state.
    if (elapsed - scanStarted_ > kScanTimeoutSec) {
        Logf("CDU scan timeout in state=%s (pages=%zu last=%d/%d)",
             scanStateName(), scannedPages_.size(), lastCollectedPage_,
             scanPageCount_);
        if (state_ == ScanState::Restore) {
            finishScan(elapsed);
        } else {
            abortScan(elapsed);
        }
        return;
    }

    std::vector<std::string> rows;
    if (!readScreen(*scanSide_, rows)) {
        return;
    }
    CduPage page = ParseCduLegsPage(rows);
    if (!page.valid) {
        // The CDU left the LEGS page (user input or a display glitch).  Stop
        // pressing keys; finish with what has been collected so far.
        lastError_ = "CDU left the LEGS page during scan";
        Logf("CDU scan stopped: %s", lastError_.c_str());
        finishScan(elapsed);
        return;
    }

    switch (state_) {
    case ScanState::SeekFirst:
        if (page.page <= 1) {
            scannedPages_.push_back(page);
            lastCollectedPage_ = 1;
            lastAction_ = elapsed;
            seekStall_ = 0;
            if (scanPageCount_ <= 1) {
                state_ = ScanState::Restore;
                restoreStarted_ = elapsed;
            } else {
                state_ = ScanState::Collect;
                press(scanSide_->nextPage, elapsed);
            }
        } else {
            if (++seekStall_ > kMaxStalledPresses) {
                Logf("CDU seek-first stalled at page %d; aborting", page.page);
                abortScan(elapsed);
                return;
            }
            press(scanSide_->prevPage, elapsed);
            lastAction_ = elapsed;
        }
        break;

    case ScanState::Collect:
        if (page.page > lastCollectedPage_) {
            scannedPages_.push_back(page);
            lastCollectedPage_ = page.page;
            lastAction_ = elapsed;
            collectStall_ = 0;
            if (page.page >= scanPageCount_) {
                state_ = ScanState::Restore;
                restoreStarted_ = elapsed;
            } else {
                press(scanSide_->nextPage, elapsed);
            }
        } else if (page.page >= 1 && page.page < scanPageCount_) {
            if (++collectStall_ > kMaxStalledPresses) {
                Logf("CDU page-next stalled at page %d/%d; aborting", page.page,
                     scanPageCount_);
                abortScan(elapsed);
                return;
            }
            press(scanSide_->nextPage, elapsed);
            lastAction_ = elapsed;
        }
        break;

    case ScanState::Restore:
        if (elapsed - restoreStarted_ > kRestoreTimeoutSec) {
            Logf("CDU page restore timeout (on page %d, wanted %d); finishing",
                 page.page, originalPage_);
            finishScan(elapsed);
            return;
        }
        if (page.page == originalPage_) {
            finishScan(elapsed);
        } else if (page.page > originalPage_) {
            press(scanSide_->prevPage, elapsed);
            lastAction_ = elapsed;
        } else {
            press(scanSide_->nextPage, elapsed);
            lastAction_ = elapsed;
        }
        break;

    default:
        break;
    }
}

void CduRouteReader::abortScan(double elapsed) {
    lastError_ = "CDU scan aborted";
    state_ = ScanState::Restore;
    restoreStarted_ = elapsed;
    lastAction_ = elapsed;
    if (!scanSide_ || !scanSide_->canPage()) finishScan(elapsed);
}

void CduRouteReader::finishScan(double elapsed) {
    CduRoute merged;
    for (const CduPage& page : scannedPages_) AppendCduPage(merged, page);

    const bool collectedAll = !scannedPages_.empty() &&
                              lastCollectedPage_ >= scanPageCount_;
    const bool haveNewRoute = merged.valid && !merged.legs.empty();

    if (haveNewRoute && (!collectedAll && cacheValid_)) {
        // A partial scan must never replace a complete cached route.
        lastError_ = "CDU scan incomplete; using cached route";
        Logf("CDU scan incomplete: collected=%zu lastPage=%d/%d; cached route kept (%zu legs)",
             scannedPages_.size(), lastCollectedPage_, scanPageCount_,
             cache_.legs.size());
    } else if (haveNewRoute) {
        int initialActive = 0;
        // Waypoint to keep active across the rescan.  A rescan used to reset
        // activeIndex_ to 0 every time, which restarted the page-1/geometry
        // tug-of-war around the ZULS pseudo-fixes.  While the LEGS page is
        // authoritative, keep the waypoint that page 1 is showing; otherwise
        // keep the current one by ident.
        std::string keepActiveId;
        // The FF757 LEGS page usually starts at the active waypoint, so a
        // rescan mid-route drops the fix behind it.  Keep the previously
        // scanned inbound fix when the new page set starts at the old active
        // waypoint.  That gives the geometry layer a real leg without asking
        // the avionics for a course.
        if (cacheValid_ && !cache_.legs.empty()) {
            int oldActive = activeIndex_;
            if (oldActive < 0) oldActive = 0;
            if (oldActive >= static_cast<int>(cache_.legs.size())) {
                oldActive = static_cast<int>(cache_.legs.size()) - 1;
            }
            int keepIdx = oldActive;
            if (screenActiveFresh() && screenActiveIndex_ >= 0 &&
                screenActiveIndex_ < static_cast<int>(cache_.legs.size())) {
                keepIdx = screenActiveIndex_;
            }
            keepActiveId = cache_.legs[static_cast<size_t>(keepIdx)].id;

            const std::string& newFirst = merged.legs.front().id;
            if (oldActive > 0 && !newFirst.empty() &&
                cache_.legs[static_cast<size_t>(oldActive)].id == newFirst) {
                merged.legs.insert(merged.legs.begin(),
                                   cache_.legs[static_cast<size_t>(oldActive - 1)]);
                initialActive = 1;
            }
        }

        cache_ = merged;
        cacheValid_ = true;
        activeIndex_ = initialActive;
        if (!keepActiveId.empty()) {
            int match = -1;
            int bestDelta = 1 << 30;
            for (int i = 0; i < static_cast<int>(cache_.legs.size()); ++i) {
                if (cache_.legs[static_cast<size_t>(i)].id != keepActiveId) continue;
                const int delta = std::abs(i - initialActive);
                if (delta < bestDelta) {
                    bestDelta = delta;
                    match = i;
                }
            }
            if (match >= 0) activeIndex_ = match;
        }
        buildSnapshot();
        if (route_.valid) {
            lastError_.clear();
        }
    } else if (!cacheValid_) {
        route_.clear();
        lastError_ = "no active LEGS route on CDU";
    } else {
        lastError_ = "CDU scan found no route; using cached route";
    }

    Logf("CDU scan done: states=%zu legs=%zu complete=%d cachedLegs=%d active=%d%s",
         scannedPages_.size(), cache_.legs.size(), collectedAll ? 1 : 0,
         cachedLegCount(), route_.activeWpt,
         lastError_.empty() ? "" : " error=");
    if (!lastError_.empty()) Logf("CDU scan note: %s", lastError_.c_str());

    lastScan_ = elapsed;
    state_ = ScanState::Idle;
    scanSide_ = nullptr;
    collectStall_ = 0;
    seekStall_ = 0;
    if (pressedRef_) {
        XPLMSetDatai(pressedRef_, 0);
        pressedRef_ = nullptr;
    }
}

void CduRouteReader::advanceActiveIfNeeded(double elapsed) {
    if (!cacheValid_ || elapsed - lastActivePoll_ < 0.5) return;
    lastActivePoll_ = elapsed;

    // Geometric sequencing fallback: advance while the active fix is clearly
    // behind the aircraft (bearing to it opposes the leg course).  This keeps
    // the snapshot in step with the FMS when Airnav distance/bearing is not
    // published or the CDU is left on another page; without it an RNP leg can
    // stay on waypoint 0 and never reach its RF branch.
    {
        double lat = 0.0;
        double lon = 0.0;
        readAircraftPosition(lat, lon);
        if (ValidLatLon(lat, lon)) {
            const bool pageFresh = screenActiveFresh();
            for (int guard = 0; guard < 10; ++guard) {
                const int last = static_cast<int>(route_.wpts.size()) - 1;
                if (activeIndex_ < 0 || activeIndex_ + 1 > last) break;
                // Page 1 line 1 is what the FMS is flying to.  While it is
                // authoritative, geometry may catch up to it but must not
                // advance past it; otherwise this fallback walked through the
                // ZULS pseudo-fixes to DM and page 1 pulled the index back
                // every few seconds (active-index tug-of-war).
                if (pageFresh && activeIndex_ >= screenActiveIndex_) break;
                const RouteWaypoint& cur = route_.wpts[static_cast<size_t>(activeIndex_)];
                const RouteWaypoint& nxt = route_.wpts[static_cast<size_t>(activeIndex_ + 1)];
                if (!cur.posValid || !nxt.posValid) break;
                const double dCur = distanceNm(lat, lon, cur.lat, cur.lon);
                if (!std::isfinite(dCur) || dCur < 0.3) break;

                // Passed test: project the aircraft position onto the outbound
                // leg direction from the current fix.  A pure "bearing to the
                // fix opposes the leg course" test over-sequenced short, tightly
                // turning departures: during the initial climb/turn several
                // nearby fixes all read as "behind" and the active index ran
                // past the fix the CDU still showed on page 1.  The along-track
                // projection only advances when the aircraft is genuinely
                // beyond the fix along the leg.
                const double legCrs = bearingTrue(cur.lat, cur.lon, nxt.lat, nxt.lon);
                const double midLatRad = deg2rad((lat + cur.lat) * 0.5);
                const double northNm = (lat - cur.lat) * 60.0;
                const double eastNm = (lon - cur.lon) * 60.0 * std::cos(midLatRad);
                const double alongNm =
                    northNm * std::cos(deg2rad(legCrs)) +
                    eastNm * std::sin(deg2rad(legCrs));
                if (!std::isfinite(alongNm) || alongNm < 0.3) break;
                ++activeIndex_;
            }
            if (activeIndex_ != lastActiveBuilt_) {
                lastActiveBuilt_ = activeIndex_;
                buildSnapshot();
            }
        }
    }

    if (!airnavDis_) return;

    double distance = 0.0;
    int fromTo = 0;
    readAirnav(distance, fromTo);
    if (distance <= 0.0 || distance > 200.0 || fromTo == 0) {
        haveAirnavDis_ = false;
        return;
    }

    const double bearing = RdF(airnavBrg_);
    const bool synced =
        (std::isfinite(bearing) && std::abs(bearing) <= 360.0)
            ? syncActiveFromAirnav(distance, bearing)
            : false;

    // Fallback for builds that do not publish AirnavBrg: advance one leg when
    // the FF757 has just sequenced past a fix.
    if (!synced && haveAirnavDis_ && lastAirnavDis_ < 0.8 && distance > 2.0 &&
        activeIndex_ >= 0 && activeIndex_ < static_cast<int>(cache_.legs.size()) - 1) {
        ++activeIndex_;
        buildSnapshot();
    }
    lastAirnavDis_ = distance;
    haveAirnavDis_ = true;
}

bool CduRouteReader::screenActiveFresh() const {
    // Page polls run every 0.5 s while a LEGS page is displayed.  Ten seconds
    // covers a short CDU visit to another page; after that the geometric
    // sequencing fallback takes over again (CDU left elsewhere).
    return screenActiveIndex_ >= 0 &&
           XPLMGetElapsedTime() - screenActiveAt_ < 10.0;
}

void CduRouteReader::updateActiveFromScreen(const CduPage& page, double elapsed) {
    if (!cacheValid_ || page.page != 1 || page.legs.empty()) return;
    // A MOD/reference page must never move the active waypoint.
    if (!page.activeRoute) return;
    const std::string& firstId = page.legs.front().id;
    if (firstId.empty()) return;

    // Boeing LEGS page 1 line 1 is the waypoint being flown to right now (the
    // next waypoint), so the displayed page is authoritative in **both**
    // directions: starting ahead of the cached index means the index fell
    // behind, starting behind it means the geometric sequencing fallback ran
    // too far.  Previously a match behind the index was ignored, so an
    // over-advance could stick until the FMS happened to sequence again.
    int match = -1;
    int bestDelta = 1 << 30;
    for (int i = 0; i < static_cast<int>(cache_.legs.size()); ++i) {
        if (cache_.legs[static_cast<size_t>(i)].id != firstId) continue;
        const int delta = std::abs(i - activeIndex_);
        if (delta < bestDelta) {
            bestDelta = delta;
            match = i;
        }
    }
    if (match < 0) {
        scanRequested_ = true;
        return;
    }
    // A page that still shows a waypoint the aircraft has already passed is
    // lagging the FMS sequencing (live 1.2.24: page 1 kept showing LXA while
    // the aircraft was 10+ NM past it, and every poll pulled the index back
    // from DM).  Do not rewind onto it; let the geometric sequencing own the
    // index until the page catches up.
    if (match < activeIndex_) {
        double lat = 0.0;
        double lon = 0.0;
        readAircraftPosition(lat, lon);
        if (ValidLatLon(lat, lon) && WaypointBehind(route_, match, lat, lon)) {
            screenActiveIndex_ = -1;
            screenActiveAt_ = -1000.0;
            return;
        }
    }
    // Refresh the authority timestamp even when the index does not change.
    // Page 1 is displayed every 0.5 s, so this keeps the geometric sequencing
    // fallback capped while the page stays in view.
    screenActiveIndex_ = match;
    screenActiveAt_ = elapsed;
    if (match == activeIndex_) return;

    Logf("CDU active from page 1: %d (%s) -> %d (%s)",
         activeIndex_,
         (activeIndex_ >= 0 && activeIndex_ < static_cast<int>(cache_.legs.size()))
             ? cache_.legs[static_cast<size_t>(activeIndex_)].id.c_str()
             : "-",
         match, cache_.legs[static_cast<size_t>(match)].id.c_str());
    activeIndex_ = match;
    buildSnapshot();
}

void CduRouteReader::buildSnapshot() {
    route_.clear();
    const size_t count = cache_.legs.size();
    if (count == 0) {
        lastError_ = "CDU route cache empty";
        return;
    }

    double lat = 0.0;
    double lon = 0.0;
    readAircraftPosition(lat, lon);
    const double anchorLat = ValidLatLon(lat, lon) ? lat : 0.0;
    const double anchorLon = ValidLatLon(lat, lon) ? lon : 0.0;

    std::vector<std::string> idents;
    idents.reserve(count);
    for (const CduLeg& leg : cache_.legs) idents.push_back(leg.id);

    std::vector<NavFix> fixes;
    nav_.resolve(idents, anchorLat, anchorLon, fixes);

    const double dtk = RdF(airnavDtk_);
    double airnavDistance = 0.0;
    int fromTo = 0;
    readAirnav(airnavDistance, fromTo);

    route_.sourceRoute = true;
    route_.rawRteCount = static_cast<int>(count);
    route_.legCount = static_cast<int>(count);
    route_.wptCount = static_cast<int>(count);
    route_.wpts.resize(count);

    double prevLat = anchorLat;
    double prevLon = anchorLon;
    bool prevValid = ValidLatLon(anchorLat, anchorLon);

    for (size_t i = 0; i < count; ++i) {
        const CduLeg& leg = cache_.legs[i];
        RouteWaypoint& wpt = route_.wpts[i];
        wpt.id = leg.id;
        wpt.altFt = leg.altFt > 0 ? leg.altFt : INT_MIN;
        wpt.altValid = leg.altFt > 0;
        wpt.altConstraint = leg.altConstraint;
        wpt.constraintSpeedKt = leg.constraintSpeedKt;
        wpt.estSpeedKt = leg.estSpeedKt;
        wpt.speedValid = leg.estSpeedKt > 0 || leg.constraintSpeedKt > 0;

        bool have = fixes.size() > i && fixes[i].valid;
        if (have) {
            wpt.lat = fixes[i].lat;
            wpt.lon = fixes[i].lon;
        } else if (prevValid && dtk > 0.0) {
            // Navdata lookup failed.  Synthesise the geometry from the active
            // leg's desired track and the leg distance printed on the CDU.
            double dist = 0.0;
            if (i == 0) {
                dist = airnavDistance;
            } else if (leg.legDistanceNm > 0.0) {
                dist = leg.legDistanceNm;
            } else {
                dist = 10.0;
            }
            if (dist > 0.0) {
                double outLat = 0.0;
                double outLon = 0.0;
                destinationPoint(prevLat, prevLon, dtk, dist, outLat, outLon);
                if (ValidLatLon(outLat, outLon)) {
                    wpt.lat = outLat;
                    wpt.lon = outLon;
                    have = true;
                }
            }
        }

        wpt.posValid = have;
        if (have) {
            prevLat = wpt.lat;
            prevLon = wpt.lon;
            prevValid = true;
        } else {
            prevValid = false;
        }
    }

    // Align the cached active index with the FF757's own distance-to-active.
    // A LEGS page can start at the first route waypoint even when the aircraft
    // is already downstream; after such a scan activeIndex_ would otherwise
    // point at the origin.  The FF757 AirnavDis value is exactly the distance
    // to its active waypoint, so it makes a reliable anchor.
    if (airnavDistance > 1.0 && fromTo != 0 && count > 1 && !screenActiveFresh()) {
        int best = -1;
        double bestDiff = 1.0e30;
        for (int i = 0; i < static_cast<int>(count); ++i) {
            const RouteWaypoint& w = route_.wpts[static_cast<size_t>(i)];
            if (!w.posValid) continue;
            const double d = distanceNm(lat, lon, w.lat, w.lon);
            if (!std::isfinite(d)) continue;
            const double diff = std::abs(d - airnavDistance);
            if (diff < bestDiff) {
                bestDiff = diff;
                best = i;
            }
        }
        if (best > activeIndex_ && bestDiff < 1.0) {
            // Only correct a clearly stale index.  On a dense terminal route
            // two fixes can be at nearly the same distance; in that case keep
            // the index the CDU page and the sequencing logic already found.
            double currentDiff = 1.0e30;
            if (activeIndex_ >= 0 && activeIndex_ < static_cast<int>(count)) {
                const RouteWaypoint& cur = route_.wpts[static_cast<size_t>(activeIndex_)];
                if (cur.posValid) {
                    const double d = distanceNm(lat, lon, cur.lat, cur.lon);
                    if (std::isfinite(d)) currentDiff = std::abs(d - airnavDistance);
                }
            }
            if (currentDiff > 1.0) activeIndex_ = best;
        }
    }

    if (activeIndex_ < 0) activeIndex_ = 0;
    if (activeIndex_ >= static_cast<int>(count)) activeIndex_ = static_cast<int>(count) - 1;
    route_.activeWpt = activeIndex_;
    route_.activeLeg = activeIndex_;
    route_.nextLeg = (activeIndex_ + 1 < static_cast<int>(count)) ? activeIndex_ + 1 : -1;

    route_.legs.reserve(count);
    for (int i = 0; i < static_cast<int>(count); ++i) {
        RouteLeg leg;
        leg.type = 0;
        if (i == 0) {
            leg.firstWpt = 0;
            leg.lastWpt = 0;
        } else {
            leg.firstWpt = i - 1;
            leg.lastWpt = i;
        }
        leg.complex = cache_.legs[static_cast<size_t>(i)].hold;
        route_.legs.push_back(leg);
    }

    route_.vtk = RdF(vtk_);

    const RouteWaypoint& activeWpt = route_.wpts[static_cast<size_t>(activeIndex_)];
    const bool activeValid = activeWpt.posValid;
    route_.activeId = activeWpt.id;
    if (route_.nextLeg >= 0) {
        route_.nextId = route_.wpts[static_cast<size_t>(route_.nextLeg)].id;
    }
    route_.valid = activeValid && !activeWpt.id.empty();
    if (!route_.valid) {
        route_.validationError = activeValid ? "active waypoint ident empty"
                                             : "active waypoint coordinates unresolved";
        lastError_ = route_.validationError;
    } else {
        lastError_.clear();
        AnnotateRfArcs(route_, cifp_, nav_);
    }
}

} // namespace lvnv
