#include "XPLMPlugin.h"
#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include "XPLMMenus.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMUtilities.h"

#include "config.h"
#include "geo.h"
#include "guidance.h"
#include "log.h"
#include "route_reader.h"
#include "version.h"

#include <algorithm>
#include <cctype>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>

namespace lvnv {

// ---------------------------------------------------------------------------
// logging

std::ofstream g_logFile;
std::string g_logPath;
bool g_logFailed = false;

void LogLine(const std::string& line) {
    if (g_logFile.is_open()) {
        g_logFile << line << "\n";
        g_logFile.flush();
    }
    std::string dbg = "[757LVNAV+] " + line + "\n";
    XPLMDebugString(dbg.c_str());
}

void Logf(const char* fmt, ...) {
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    LogLine(buf);
}

// ---------------------------------------------------------------------------
// paths

std::string SystemPath() {
    char path[512] = {0};
    XPLMGetSystemPath(path);
    std::string s(path);
    if (!s.empty() && (s.back() == '/' || s.back() == '\\')) return s;
    s += '/';
    return s;
}

std::string ConfigPath() {
    return SystemPath() + "Output/preferences/757LVNAVPlus.cfg";
}

bool EnsureParentDirectory(const std::string& filePath) {
    try {
        std::filesystem::path p(filePath);
        if (p.has_parent_path()) {
            std::filesystem::create_directories(p.parent_path());
        }
        return true;
    } catch (...) {
        return false;
    }
}

// ---------------------------------------------------------------------------
// runtime state

Config g_config;
RouteReader g_routeReader;
Guidance g_guidance;

struct SimRefs {
    XPLMDataRef lat = nullptr;
    XPLMDataRef lon = nullptr;
    XPLMDataRef elevation = nullptr;
    XPLMDataRef truePsi = nullptr;
    XPLMDataRef magPsi = nullptr;
    XPLMDataRef hpath = nullptr;
    XPLMDataRef gs = nullptr;
    XPLMDataRef vhInd = nullptr;
    XPLMDataRef altPilot = nullptr;
    XPLMDataRef ias = nullptr;
    XPLMDataRef mach = nullptr;
    XPLMDataRef rollAngleDeg = nullptr;
    XPLMDataRef onGround = nullptr;
    XPLMDataRef paused = nullptr;
    XPLMDataRef totalTime = nullptr;

    XPLMDataRef autopilotOn = nullptr;
    XPLMDataRef servosOn = nullptr;
    XPLMDataRef headingMode = nullptr;
    XPLMDataRef altitudeMode = nullptr;
    XPLMDataRef navStatus = nullptr;
    XPLMDataRef vnavStatus = nullptr;
    XPLMDataRef fmsVnav = nullptr;

    XPLMDataRef overrideNavHeading = nullptr;
    XPLMDataRef navSteer = nullptr;
    XPLMDataRef fdRollCockpit2 = nullptr;
    XPLMDataRef fdPitchCockpit2 = nullptr;
    XPLMDataRef fdRollLegacy = nullptr;
    XPLMDataRef fdPitchLegacy = nullptr;
    XPLMDataRef overridePitch = nullptr;
    XPLMDataRef overrideRoll = nullptr;
    // Diagnostics for the VNAV pitch-authority investigation: which controller
    // actually owns the pitch axis.
    XPLMDataRef apState = nullptr;
    XPLMDataRef overrideArtstab = nullptr;
    XPLMDataRef artstabPitch = nullptr;

    XPLMDataRef gpsCourse = nullptr;
    XPLMDataRef gpsDist = nullptr;
    XPLMDataRef gpsHdefDot = nullptr;
    XPLMDataRef gpsHdefNmPerDot = nullptr;
    XPLMDataRef gpsFromTo = nullptr;

    XPLMDataRef acfPath = nullptr;
    XPLMDataRef acfIcao = nullptr;

    XPLMDataRef apMode = nullptr;
    XPLMDataRef rollMode = nullptr;
    XPLMDataRef pitchMode = nullptr;
    XPLMDataRef rollArmed = nullptr;
    XPLMDataRef pitchArmed = nullptr;
    XPLMDataRef vnavPhase = nullptr;
    XPLMDataRef leftFd = nullptr;
    XPLMDataRef rightFd = nullptr;
    XPLMDataRef apWarn = nullptr;
    XPLMDataRef atServos = nullptr;

    XPLMDataRef airnavDtk = nullptr;
    XPLMDataRef airnavDis = nullptr;
    XPLMDataRef airnavXtk = nullptr;
    XPLMDataRef airnavBrg = nullptr;
    XPLMDataRef airnavTke = nullptr;
    XPLMDataRef airnavAtd = nullptr;
    XPLMDataRef airnavFromTo = nullptr;
    XPLMDataRef altAct = nullptr;
    XPLMDataRef selectedAlt = nullptr;
    XPLMDataRef xtkPerDot = nullptr;
    XPLMDataRef anp = nullptr;
    XPLMDataRef rnp = nullptr;
    XPLMDataRef crzAlt = nullptr;
    XPLMDataRef transAlt = nullptr;
    XPLMDataRef vmo = nullptr;
    XPLMDataRef vmin = nullptr;

    bool navSteerWritable = false;
    bool fdPitchLegacyWritable = false;
    bool fdPitchCockpit2Writable = false;
    bool fdRollLegacyWritable = false;
    bool fdRollCockpit2Writable = false;
    bool overridePitchWritable = false;
    bool overrideRollWritable = false;
    bool overrideNavHeadingWritable = false;
    bool ffAvionicsFound = false;
    bool ffDatarefsFound = false;
};

SimRefs g_refs;

struct Status {
    bool enabled = false;
    bool aircraftIsFF = false;
    bool routeBound = false;
    bool routeValid = false;

    int apMode = 0;
    int rollMode = 0;
    int pitchMode = 0;
    int vnavPhase = 0;
    bool apEngaged = false;
    bool lnavOverride = false;
    bool lnavMode = false;
    bool vnavMode = false;
    // 0 = none, 1 = VNAV SPD, 2 = VNAV PTH.  Mirrors the FF757 FMA so the
    // status window reports the same submode the PFD shows.
    int vnavSubmode = 0;

    int routeLegs = 0;
    int routeWpts = 0;
    int activeWpt = -1;
    int activeLeg = -1;
    int nextLeg = -1;
    std::string activeId;
    std::string nextId;

    double xtkNm = 0.0;
    double distanceNm = 0.0;
    double baseCourse = 0.0;
    double desiredCourse = 0.0;
    double lnavCorrection = 0.0;
    double targetAltFt = 0.0;
    double requiredVs = 0.0;
    double vsError = 0.0;
    double vnavCorrection = 0.0;

    bool navWritten = false;
    bool pitchWritten = false;
    std::string lnavNote;
    std::string vnavNote;
    std::string message;

    CdiInput cdi;
    LnavResult lnav;
    VnavResult vnav;
};

Status g_status;

AircraftState g_state;
CdiInput g_cdi;
LnavResult g_lnav;
VnavResult g_vnav;

double g_lastVsFpm = 0.0;
double g_vsAccel = 0.0;
double g_lastFlightTime = -1.0;
double g_lastRefRebind = -1000.0;
bool g_filterInit = false;
double g_filteredBaseline = 0.0;
double g_lastBaseline = 0.0;
double g_lastPitchOut = 0.0;
bool g_ownPitchOverride = false;
double g_lastPitchWriteTime = 0.0;
int g_pitchWriteCount = 0;
double g_lastVnavServoLog = -1000.0;
int g_ffPitchOwnCount = 0;
double g_lastLnavServoLog = -1000.0;
double g_lastNavSteerCmd = 0.0;
double g_navSteerFrameStart = 0.0;
double g_lastLnavDamp = 0.0;
double g_lastLnavL1 = 0.0;
bool g_cockpit2ProbeDone = false;
bool g_cockpit2Writable = false;
float g_lastCockpit2RawOut = 1.0e9f;
double g_lastCockpit2Correction = 0.0;

// VNAV pitch priority.  The FF757 writes its own FD pitch from its VNAV path
// while the plugin writes the same dataref; whichever writes last wins, so
// the two controllers can alternate frame by frame ("夺权").  Once the plugin
// has a valid descent path it latches priority, holds a frozen baseline
// reference and keeps writing, and only releases after the path has been
// inactive for ~2 s.
bool g_vnavPriority = false;
bool g_pitchAuthorityInit = false;
double g_pitchAuthorityRef = 0.0;
int g_vnavReleaseCount = 0;
bool g_vnavPriorityLogged = false;

bool g_rollFilterInit = false;
double g_rollFilteredBaseline = 0.0;
double g_rollLastBaseline = 0.0;
double g_rollLastOut = 0.0;
bool g_ownRollOverride = false;
int g_rollWriteCount = 0;
bool g_cockpit2RollProbeDone = false;
bool g_cockpit2RollWritable = false;
float g_lastCockpit2RollOut = 1.0e9f;
double g_lastCockpit2RollCorrection = 0.0;
float g_lastLegacyRollOut = 1.0e9f;
double g_lastLegacyRollCorrection = 0.0;

// The RF arc owns the nav target.  With override_nav_heading set, X-Plane's
// nav controller flies our nav_steer heading instead of the model's course.
// The FF757 v2 build resets the flag every frame; re-asserting it (rather than
// treating that as a failure) matches the 787 reference script, and FD roll is
// skipped while it is live so only one lateral controller commands roll.
bool g_ownNavHeadingOverride = false;
bool g_headingDirectLogged = false;
int g_navOverrideAtFrameStart = 0;

int g_errorCount = 0;
std::string g_lastError;

// Last route shape written to the log.  The signature covers waypoint idents,
// coordinates, altitudes, active index and leg layout, so a rescan that only
// changes a constraint altitude is still reported.
std::uint64_t g_loggedRouteSignature = 0;
bool g_loggedRouteValid = false;
std::string g_lastVnavSource;
std::string g_lastLnavSource;

XPLMFlightLoopID g_flightLoop = nullptr;
XPLMWindowID g_statusWindow = nullptr;
XPLMMenuID g_menu = nullptr;
bool g_statusVisible = false;
double g_aboutUntil = -1.0;

XPLMCommandRef g_cmdToggle = nullptr;
XPLMCommandRef g_cmdLnav = nullptr;
XPLMCommandRef g_cmdVnav = nullptr;
XPLMCommandRef g_cmdFdRoll = nullptr;
XPLMCommandRef g_cmdCdiSource = nullptr;
XPLMCommandRef g_cmdStatus = nullptr;
XPLMCommandRef g_cmdDump = nullptr;

// ---------------------------------------------------------------------------
// published datarefs

struct Published {
    int enabled = 1;
    int lnavEnabled = 1;
    int vnavEnabled = 1;
    int status = 0;

    int lnavActive = 0;
    int vnavActive = 0;
    int routeValid = 0;
    int routeSource = 0;   // 0=none, 1=private rteData, 2=CDU LEGS page
    int routeLegs = 0;
    int routeWpts = 0;
    int activeWpt = -1;
    int activeLeg = -1;
    int nextLeg = -1;
    int apMode = 0;
    int rollMode = 0;
    int pitchMode = 0;
    int vnavPhase = 0;
    int apEngaged = 0;
    int lnavOverride = 0;

    float xtkNm = 0.0f;
    float distanceNm = 0.0f;
    float baseCourseDeg = 0.0f;
    float desiredCourseDeg = 0.0f;
    float lnavCorrectionDeg = 0.0f;
    float targetAltFt = 0.0f;
    float requiredVsFpm = 0.0f;
    float vsErrorFpm = 0.0f;
    float vnavCorrectionDeg = 0.0f;
    float cdiXtkNm = 0.0f;

    std::vector<XPLMDataRef> refs;
};

Published g_pub;

int IntGet(void* refcon) {
    return *static_cast<int*>(refcon);
}

void IntSet(void* refcon, int value) {
    *static_cast<int*>(refcon) = value;
}

float FloatGet(void* refcon) {
    return *static_cast<float*>(refcon);
}

void FloatSet(void* refcon, float value) {
    *static_cast<float*>(refcon) = value;
}

XPLMDataRef RegisterInt(const char* name, int* ptr, bool writable) {
    return XPLMRegisterDataAccessor(
        name, xplmType_Int, writable ? 1 : 0,
        IntGet, writable ? IntSet : nullptr,
        nullptr, nullptr,
        nullptr, nullptr,
        nullptr, nullptr,
        nullptr, nullptr,
        nullptr, nullptr,
        ptr, writable ? ptr : nullptr);
}

XPLMDataRef RegisterFloat(const char* name, float* ptr) {
    return XPLMRegisterDataAccessor(
        name, xplmType_Float, 0,
        nullptr, nullptr,
        FloatGet, nullptr,
        nullptr, nullptr,
        nullptr, nullptr,
        nullptr, nullptr,
        nullptr, nullptr,
        ptr, nullptr);
}

void RegisterPublished() {
    g_pub.refs.clear();
    g_pub.refs.push_back(RegisterInt("757lvnv/enabled", &g_pub.enabled, true));
    g_pub.refs.push_back(RegisterInt("757lvnv/lnav_enabled", &g_pub.lnavEnabled, true));
    g_pub.refs.push_back(RegisterInt("757lvnv/vnav_enabled", &g_pub.vnavEnabled, true));
    g_pub.refs.push_back(RegisterInt("757lvnv/status", &g_pub.status, false));
    g_pub.refs.push_back(RegisterInt("757lvnv/lnav_active", &g_pub.lnavActive, false));
    g_pub.refs.push_back(RegisterInt("757lvnv/vnav_active", &g_pub.vnavActive, false));
    g_pub.refs.push_back(RegisterInt("757lvnv/route_valid", &g_pub.routeValid, false));
    g_pub.refs.push_back(RegisterInt("757lvnv/route_source", &g_pub.routeSource, false));
    g_pub.refs.push_back(RegisterInt("757lvnv/route_legs", &g_pub.routeLegs, false));
    g_pub.refs.push_back(RegisterInt("757lvnv/route_wpts", &g_pub.routeWpts, false));
    g_pub.refs.push_back(RegisterInt("757lvnv/active_wpt", &g_pub.activeWpt, false));
    g_pub.refs.push_back(RegisterInt("757lvnv/active_leg", &g_pub.activeLeg, false));
    g_pub.refs.push_back(RegisterInt("757lvnv/next_leg", &g_pub.nextLeg, false));
    g_pub.refs.push_back(RegisterInt("757lvnv/ap_mode", &g_pub.apMode, false));
    g_pub.refs.push_back(RegisterInt("757lvnv/roll_mode", &g_pub.rollMode, false));
    g_pub.refs.push_back(RegisterInt("757lvnv/pitch_mode", &g_pub.pitchMode, false));
    g_pub.refs.push_back(RegisterInt("757lvnv/vnav_phase", &g_pub.vnavPhase, false));
    g_pub.refs.push_back(RegisterInt("757lvnv/ap_engaged", &g_pub.apEngaged, false));
    g_pub.refs.push_back(RegisterInt("757lvnv/lnav_override", &g_pub.lnavOverride, false));

    g_pub.refs.push_back(RegisterFloat("757lvnv/xtk_nm", &g_pub.xtkNm));
    g_pub.refs.push_back(RegisterFloat("757lvnv/distance_nm", &g_pub.distanceNm));
    g_pub.refs.push_back(RegisterFloat("757lvnv/base_course_deg", &g_pub.baseCourseDeg));
    g_pub.refs.push_back(RegisterFloat("757lvnv/desired_course_deg", &g_pub.desiredCourseDeg));
    g_pub.refs.push_back(RegisterFloat("757lvnv/lnav_correction_deg", &g_pub.lnavCorrectionDeg));
    g_pub.refs.push_back(RegisterFloat("757lvnv/target_alt_ft", &g_pub.targetAltFt));
    g_pub.refs.push_back(RegisterFloat("757lvnv/required_vs_fpm", &g_pub.requiredVsFpm));
    g_pub.refs.push_back(RegisterFloat("757lvnv/vs_error_fpm", &g_pub.vsErrorFpm));
    g_pub.refs.push_back(RegisterFloat("757lvnv/vnav_correction_deg", &g_pub.vnavCorrectionDeg));
    g_pub.refs.push_back(RegisterFloat("757lvnv/cdi_xtk_nm", &g_pub.cdiXtkNm));
}

void UnregisterPublished() {
    for (XPLMDataRef r : g_pub.refs) {
        if (r) XPLMUnregisterDataAccessor(r);
    }
    g_pub.refs.clear();
}

void SyncPublishedFromConfig() {
    g_pub.enabled = g_config.enabled ? 1 : 0;
    g_pub.lnavEnabled = g_config.enableLnav ? 1 : 0;
    g_pub.vnavEnabled = g_config.enableVnav ? 1 : 0;
}

// ---------------------------------------------------------------------------
// dataref binding

XPLMDataRef Find(const char* name) {
    return XPLMFindDataRef(name);
}

void BindRefs() {
    g_refs.lat = Find("sim/flightmodel/position/latitude");
    g_refs.lon = Find("sim/flightmodel/position/longitude");
    g_refs.elevation = Find("sim/flightmodel/position/elevation");
    g_refs.truePsi = Find("sim/flightmodel/position/true_psi");
    g_refs.magPsi = Find("sim/flightmodel/position/mag_psi");
    g_refs.hpath = Find("sim/flightmodel/position/hpath");
    g_refs.gs = Find("sim/flightmodel/position/groundspeed");
    g_refs.vhInd = Find("sim/flightmodel/position/vh_ind_fpm");
    g_refs.altPilot = Find("sim/cockpit2/gauges/indicators/altitude_ft_pilot");
    g_refs.ias = Find("sim/cockpit2/gauges/indicators/airspeed_kts_pilot");
    g_refs.mach = Find("sim/flightmodel/misc/machno");
    g_refs.rollAngleDeg = Find("sim/cockpit2/gauges/indicators/roll_AHARS_deg_pilot");
    g_refs.onGround = Find("sim/flightmodel/failures/onground_any");
    g_refs.paused = Find("sim/time/paused");
    g_refs.totalTime = Find("sim/time/total_flight_time_sec");

    g_refs.autopilotOn = Find("sim/cockpit2/autopilot/autopilot_on");
    g_refs.servosOn = Find("sim/cockpit2/autopilot/servos_on");
    g_refs.headingMode = Find("sim/cockpit2/autopilot/heading_mode");
    g_refs.altitudeMode = Find("sim/cockpit2/autopilot/altitude_mode");
    g_refs.navStatus = Find("sim/cockpit2/autopilot/nav_status");
    g_refs.vnavStatus = Find("sim/cockpit2/autopilot/vnav_status");
    g_refs.fmsVnav = Find("sim/cockpit2/autopilot/fms_vnav");

    g_refs.overrideNavHeading = Find("sim/operation/override/override_nav_heading");
    g_refs.navSteer = Find("sim/cockpit/autopilot/nav_steer_deg_mag");
    g_refs.fdRollCockpit2 = Find("sim/cockpit2/autopilot/flight_director_roll_deg");
    g_refs.fdPitchCockpit2 = Find("sim/cockpit2/autopilot/flight_director_pitch_deg");
    g_refs.fdRollLegacy = Find("sim/cockpit/autopilot/flight_director_roll");
    g_refs.fdPitchLegacy = Find("sim/cockpit/autopilot/flight_director_pitch");
    g_refs.overridePitch = Find("sim/operation/override/override_flightdir_ptch");
    g_refs.overrideRoll = Find("sim/operation/override/override_flightdir_roll");
    g_refs.apState = Find("sim/cockpit/autopilot/autopilot_state");
    g_refs.overrideArtstab = Find("sim/operation/override/override_artstab");
    g_refs.artstabPitch = Find("sim/joystick/artstab_pitch_ratio");

    g_refs.gpsCourse = Find("sim/cockpit/radios/gps_course_degtm");
    g_refs.gpsDist = Find("sim/cockpit/radios/gps_dme_dist_m");
    g_refs.gpsHdefDot = Find("sim/cockpit/radios/gps_hdef_dot");
    g_refs.gpsHdefNmPerDot = Find("sim/cockpit/radios/gps_hdef_nm_per_dot");
    g_refs.gpsFromTo = Find("sim/cockpit/radios/gps_fromto");

    g_refs.acfPath = Find("sim/aircraft/view/acf_relative_path");
    g_refs.acfIcao = Find("sim/aircraft/view/acf_ICAO");

    g_refs.apMode = Find("757Avionics/ap/ap_mode");
    g_refs.rollMode = Find("757Avionics/ap/roll_mode_engaged");
    g_refs.pitchMode = Find("757Avionics/ap/pitch_mode_engaged");
    g_refs.rollArmed = Find("757Avionics/ap/roll_mode_armed");
    g_refs.pitchArmed = Find("757Avionics/ap/pitch_mode_armed");
    g_refs.vnavPhase = Find("757Avionics/fms/vnav_phase");
    g_refs.leftFd = Find("757Avionics/ap/leftFdState");
    g_refs.rightFd = Find("757Avionics/ap/rghtFdState");
    g_refs.apWarn = Find("757Avionics/ap/warn_flags");
    g_refs.atServos = Find("757Avionics/ap/at_servos_is_active");

    g_refs.airnavDtk = Find("757Avionics/fms/AirnavDtk");
    g_refs.airnavDis = Find("757Avionics/fms/AirnavDis");
    g_refs.airnavXtk = Find("757Avionics/fms/AirnavXtk");
    g_refs.airnavBrg = Find("757Avionics/fms/AirnavBrg");
    g_refs.airnavTke = Find("757Avionics/fms/AirnavTke");
    g_refs.airnavAtd = Find("757Avionics/fms/AirnavAtd");
    g_refs.airnavFromTo = Find("757Avionics/fms/AirnavFromTo");
    g_refs.altAct = Find("757Avionics/ap/alt_act");
    g_refs.selectedAlt = Find("sim/cockpit/autopilot/altitude");
    g_refs.xtkPerDot = Find("757Avionics/fms/xtk_per_dot");
    g_refs.anp = Find("757Avionics/fms/anp");
    g_refs.rnp = Find("757Avionics/fms/rnp");
    g_refs.crzAlt = Find("757Avionics/fms/crz_alt");
    g_refs.transAlt = Find("757Avionics/fms/trans_alt");
    g_refs.vmo = Find("757Avionics/fms/vmo");
    g_refs.vmin = Find("757Avionics/fms/vmin");

    g_refs.navSteerWritable = g_refs.navSteer && XPLMCanWriteDataRef(g_refs.navSteer);
    g_refs.fdPitchLegacyWritable =
        g_refs.fdPitchLegacy && XPLMCanWriteDataRef(g_refs.fdPitchLegacy);
    g_refs.fdPitchCockpit2Writable =
        g_refs.fdPitchCockpit2 && XPLMCanWriteDataRef(g_refs.fdPitchCockpit2);
    g_refs.fdRollLegacyWritable =
        g_refs.fdRollLegacy && XPLMCanWriteDataRef(g_refs.fdRollLegacy);
    g_refs.fdRollCockpit2Writable =
        g_refs.fdRollCockpit2 && XPLMCanWriteDataRef(g_refs.fdRollCockpit2);
    g_refs.overridePitchWritable =
        g_refs.overridePitch && XPLMCanWriteDataRef(g_refs.overridePitch);
    g_refs.overrideRollWritable =
        g_refs.overrideRoll && XPLMCanWriteDataRef(g_refs.overrideRoll);
    g_refs.overrideNavHeadingWritable =
        g_refs.overrideNavHeading && XPLMCanWriteDataRef(g_refs.overrideNavHeading);

    g_refs.ffAvionicsFound = g_refs.apMode || g_refs.rollMode || g_refs.pitchMode ||
                             g_refs.vnavPhase || g_refs.airnavDtk;
    g_refs.ffDatarefsFound = g_refs.ffAvionicsFound || g_refs.airnavDis;

    g_cockpit2ProbeDone = false;
    g_cockpit2Writable = false;
    g_lastCockpit2RawOut = 1.0e9f;
    g_lastCockpit2Correction = 0.0;

    g_cockpit2RollProbeDone = false;
    g_cockpit2RollWritable = false;
    g_lastCockpit2RollOut = 1.0e9f;
    g_lastCockpit2RollCorrection = 0.0;
    g_lastLegacyRollOut = 1.0e9f;
    g_lastLegacyRollCorrection = 0.0;
    g_rollFilterInit = false;
    g_ownRollOverride = false;
}

std::string ReadAircraftPath() {
    if (!g_refs.acfPath) return {};
    char buf[512] = {0};
    int n = XPLMGetDatab(g_refs.acfPath, buf, 0, static_cast<int>(sizeof(buf) - 1));
    if (n <= 0) return {};
    std::string s(buf, static_cast<size_t>(n));
    return s;
}

std::string ReadAircraftIcao() {
    if (!g_refs.acfIcao) return {};
    char buf[64] = {0};
    int n = XPLMGetDatab(g_refs.acfIcao, buf, 0, static_cast<int>(sizeof(buf) - 1));
    if (n <= 0) return {};
    return std::string(buf, static_cast<size_t>(n));
}

bool AircraftLooksLikeFF757() {
    if (g_refs.ffDatarefsFound) return true;

    std::string path = ReadAircraftPath();
    std::string lowerPath = path;
    std::transform(lowerPath.begin(), lowerPath.end(), lowerPath.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (lowerPath.find("boeing757") != std::string::npos ||
        lowerPath.find("boeing767") != std::string::npos ||
        lowerPath.find("757") != std::string::npos ||
        lowerPath.find("767") != std::string::npos) {
        return true;
    }

    std::string icao = ReadAircraftIcao();
    if (icao.size() >= 3 && icao[0] == 'B' &&
        (icao[1] == '7' && (icao[2] == '5' || icao[2] == '6'))) {
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// small dataref helpers

double RdD(XPLMDataRef r, double fallback = 0.0) {
    if (!r) return fallback;
    int t = XPLMGetDataRefTypes(r);
    double v = fallback;
    if (t & xplmType_Double) v = XPLMGetDatad(r);
    else if (t & xplmType_Float) v = XPLMGetDataf(r);
    else if (t & xplmType_Int) v = XPLMGetDatai(r);
    return std::isfinite(v) ? v : fallback;
}

float RdF(XPLMDataRef r, float fallback = 0.0f) {
    if (!r) return fallback;
    int t = XPLMGetDataRefTypes(r);
    float v = fallback;
    if (t & xplmType_Float) v = XPLMGetDataf(r);
    else if (t & xplmType_Double) v = static_cast<float>(XPLMGetDatad(r));
    else if (t & xplmType_Int) v = static_cast<float>(XPLMGetDatai(r));
    return std::isfinite(v) ? v : fallback;
}

int RdI(XPLMDataRef r, int fallback = 0) {
    if (!r) return fallback;
    int t = XPLMGetDataRefTypes(r);
    if (t & xplmType_Int) return XPLMGetDatai(r);
    if (t & xplmType_Float) {
        float f = XPLMGetDataf(r);
        if (std::isfinite(f)) return static_cast<int>(std::llround(f));
        return fallback;
    }
    if (t & xplmType_Double) {
        double d = XPLMGetDatad(r);
        if (std::isfinite(d)) return static_cast<int>(std::llround(d));
        return fallback;
    }
    return fallback;
}

bool Finite(double v) {
    return std::isfinite(v);
}

// ---------------------------------------------------------------------------
// state reading

void ReadAircraftState(double dt) {
    g_state = AircraftState{};

    if (!g_refs.lat || !g_refs.lon) {
        g_state.valid = false;
        return;
    }

    g_state.lat = RdD(g_refs.lat);
    g_state.lon = RdD(g_refs.lon);
    const float hpath = RdF(g_refs.hpath, NAN);
    g_state.trackTrueDeg = Finite(hpath) ? hpath : RdF(g_refs.truePsi, 0.0f);
    const double truePsi = RdF(g_refs.truePsi, 0.0f);
    const double magPsi = RdF(g_refs.magPsi, 0.0f);
    g_state.magVarDeg = normalize180(magPsi - truePsi);

    g_state.gsKts = std::max(0.0, static_cast<double>(RdF(g_refs.gs)) * kKtPerMps);
    g_state.altFt = g_refs.altPilot ? RdF(g_refs.altPilot)
                                    : (RdD(g_refs.elevation) * 3.280839895);
    g_state.vsFpm = RdF(g_refs.vhInd);
    g_state.iasKts = RdF(g_refs.ias);
    g_state.fdRollDeg = RdF(g_refs.fdRollCockpit2, 0.0f);

    if (!Finite(g_state.trackTrueDeg)) g_state.trackTrueDeg = 0.0;
    if (!Finite(g_state.gsKts)) g_state.gsKts = 0.0;
    if (!Finite(g_state.altFt)) g_state.altFt = 0.0;
    if (!Finite(g_state.vsFpm)) g_state.vsFpm = 0.0;
    if (!Finite(g_state.magVarDeg)) g_state.magVarDeg = 0.0;
    if (!Finite(g_state.fdRollDeg)) g_state.fdRollDeg = 0.0;

    g_state.onGround = RdI(g_refs.onGround, 1) != 0;
    g_state.valid = Finite(g_state.lat) && Finite(g_state.lon) &&
                    Finite(g_state.altFt) && Finite(g_state.gsKts) &&
                    Finite(g_state.trackTrueDeg);
    if (std::abs(g_state.lat) > 90.0 || std::abs(g_state.lon) > 180.0) {
        g_state.valid = false;
    }

    if (g_lastVsFpm != 0.0 && dt > 0.001) {
        double accel = (g_state.vsFpm - g_lastVsFpm) / dt;
        accel = clampd(accel, -2000.0, 2000.0);
        g_vsAccel = 0.85 * g_vsAccel + 0.15 * accel;
    }
    g_lastVsFpm = g_state.vsFpm;
    g_state.vsAccelFpmPerSec = g_vsAccel;

    g_status.apMode = RdI(g_refs.apMode);
    g_status.rollMode = RdI(g_refs.rollMode);
    g_status.pitchMode = RdI(g_refs.pitchMode);
    g_status.vnavPhase = RdI(g_refs.vnavPhase);

    // Selected (MCP) altitude.  The FF757 exposes its own active value; the
    // standard dataref is a fallback for builds that do not.
    double selectedAlt = RdF(g_refs.altAct, 0.0f);
    if (!Finite(selectedAlt) || selectedAlt <= 0.0) {
        selectedAlt = RdF(g_refs.selectedAlt, 0.0f);
    }
    if (Finite(selectedAlt) && selectedAlt > 1000.0 && selectedAlt < 60000.0) {
        g_state.selectedAltFt = selectedAlt;
    }

    const int stdAutopilot = RdI(g_refs.autopilotOn);
    const int stdServos = RdI(g_refs.servosOn);
    const bool haveCustomApMode = g_refs.apMode != nullptr;
    if (haveCustomApMode) {
        // The FF757's own AP mode is the source of truth.  X-Plane's
        // autopilot_on stays 1 with the FD switch on even when the FF757 CMD
        // is not engaged, which used to keep LNAV active and writing nav_steer
        // into an AP that was not flying (live: apMode=0, fdRoll=-20 matching
        // our command, but the aircraft never rolled and drifted 12 NM off).
        g_state.apEngaged = g_status.apMode >= g_config.apCmdMode;
    } else {
        g_state.apEngaged = stdAutopilot != 0 || (stdServos != 0 && g_refs.autopilotOn == nullptr);
    }

    const int stdHeadingMode = RdI(g_refs.headingMode);
    const int stdNavStatus = RdI(g_refs.navStatus);
    const bool lnavModeByStd = (stdHeadingMode == 2 || stdHeadingMode == 13 ||
                                stdNavStatus >= 2);
    const bool lnavModeByFF = (g_status.rollMode == g_config.lnavRollMode) ||
                              (g_config.lnavWideMode && g_status.rollMode != 0 &&
                               g_status.rollMode > 0);
    g_state.lnavModeDetected = (g_refs.rollMode || g_refs.headingMode ||
                                g_refs.navStatus)
                                   ? (lnavModeByStd || lnavModeByFF)
                                   : true;
    if (g_config.forceLnav) g_state.lnavModeDetected = true;

    // This build does not set override_nav_heading in LNAV, so it is used only
    // to decide whether the nav_steer path is available, not as an LNAV gate.
    const int overrideNav = RdI(g_refs.overrideNavHeading);
    g_state.lnavOverride = g_refs.overrideNavHeading ? (overrideNav != 0) : false;

    const int stdAltMode = RdI(g_refs.altitudeMode);
    const int stdVnavStatus = RdI(g_refs.vnavStatus);
    const int stdFmsVnav = RdI(g_refs.fmsVnav);
    const bool vnavByFF = (g_status.pitchMode == g_config.vnavPitchModeA) ||
                          (g_status.pitchMode == g_config.vnavPitchModeB) ||
                          (g_status.pitchMode == g_config.vnavPitchModeC);
    const bool vnavByStd = (stdAltMode == 9 || stdAltMode == 20 ||
                            stdVnavStatus >= 2 || stdFmsVnav != 0);
    g_state.vnavMode = vnavByFF || vnavByStd || g_config.forceVnav;

    // VNAV SPD is the climb vertical mode: pitch holds the FMC target speed.
    // The pitch-mode enum alone does not split SPD from PTH: mode 5 has been
    // observed with an FMA showing both VNAV SPD (climb phase) and VNAV PTH
    // (phase 3, level/descent path).  Use the FMC phase and actual vertical
    // speed instead of treating mode 5 as unconditionally SPD.
    const bool climbVnavPhase =
        g_refs.vnavPhase && vnavByFF && g_status.vnavPhase >= 0 &&
        g_status.vnavPhase <= 2;
    // Any established climb is flown in VNAV SPD (pitch holds speed).  Treating
    // it as speed mode even if the phase label lags prevents the plugin from
    // injecting a path correction into the speed loop.
    const bool climbingNow = g_state.vsFpm > 300.0;
    g_state.vnavSpeedMode = climbVnavPhase || climbingNow;
    g_status.vnavSubmode = !g_state.vnavMode ? 0
                         : (g_state.vnavSpeedMode ? 1 : 2);

    g_status.lnavOverride = g_state.lnavOverride;
    g_status.lnavMode = g_state.lnavModeDetected;
    g_status.vnavMode = g_state.vnavMode;
    g_status.apEngaged = g_state.apEngaged;
}

void ReadCdiInput() {
    g_cdi = CdiInput{};

    const float course = RdF(g_refs.gpsCourse, NAN);
    const float dist = RdF(g_refs.gpsDist, 0.0f);
    const float dot = RdF(g_refs.gpsHdefDot, NAN);
    float nmPerDot = RdF(g_refs.gpsHdefNmPerDot, 0.0f);
    const int fromTo = RdI(g_refs.gpsFromTo, 0);

    if (nmPerDot <= 0.001f || !Finite(nmPerDot)) {
        // Typical enroute CDI scale.  Only used if the aircraft does not
        // publish a sensitivity value.
        nmPerDot = 0.1f;
    }

    if (g_refs.gpsCourse && g_refs.gpsDist && Finite(course) && dist > 0.01f) {
        g_cdi.valid = (fromTo != 0);
        g_cdi.courseMagDeg = normalize360(course);
        g_cdi.distanceNm = dist;
        g_cdi.fromTo = fromTo;
        if (Finite(dot) && nmPerDot > 0.001f) {
            g_cdi.xtkNm = static_cast<double>(dot) * static_cast<double>(nmPerDot);
            g_cdi.xtkValid = true;
        }
        if (g_cdi.valid) return;
    }

    // Fallback to the FF757 AIRNAV active-leg values.
    const float airnavDtk = RdF(g_refs.airnavDtk, NAN);
    const float airnavDis = RdF(g_refs.airnavDis, 0.0f);
    const float airnavBrg = RdF(g_refs.airnavBrg, NAN);
    const int airnavFromTo = RdI(g_refs.airnavFromTo, 0);

    if (!Finite(airnavDtk) || airnavDis <= 0.01f) return;

    // AIRNAV active-leg distances are assumed to be nautical miles, matching
    // the standard GPS CDI distance.  This is only a fallback when the GPS
    // CDI is unavailable.
    const double distanceNm = airnavDis;

    const double currentMag = magneticFromTrue(g_state.trackTrueDeg, g_state.magVarDeg);
    const double asMag = normalize360(airnavDtk);
    const double asTrue = magneticFromTrue(airnavDtk, g_state.magVarDeg);
    const double errMag = std::abs(normalize180(asMag - currentMag));
    const double errTrue = std::abs(normalize180(asTrue - currentMag));
    g_cdi.courseMagDeg = (errTrue < errMag) ? asTrue : asMag;

    g_cdi.distanceNm = distanceNm;
    g_cdi.fromTo = airnavFromTo;
    g_cdi.valid = (airnavFromTo != 0) || distanceNm > 0.01;

    if (Finite(airnavBrg)) {
        const double brgAsTrue = airnavBrg;
        const double brgAsMag = airnavBrg;
        const double diffTrue = std::abs(normalize180(brgAsTrue - g_state.trackTrueDeg));
        const double diffMag = std::abs(normalize180(brgAsMag - currentMag));
        const double brgTrack = (diffTrue < diffMag) ? g_state.trackTrueDeg : currentMag;
        const double diff = normalize180(airnavBrg - brgTrack);
        g_cdi.xtkNm = std::sin(deg2rad(diff)) * distanceNm;  // positive = turn right
        g_cdi.xtkValid = true;
        g_cdi.signIsCorrection = true;
    }
}

// ---------------------------------------------------------------------------
// FD pitch writing

float ReadFdPitchBaseline() {
    // Read the avionics-owned FD command, never the legacy dataref that this
    // plugin writes.  If the FF757 writes the cockpit2 FD every frame, our
    // previous value is overwritten before this read and no correction is
    // needed.  If it does not, detect that cockpit2 still holds our previous
    // output and subtract the correction to recover the avionics baseline.
    float c2 = RdF(g_refs.fdPitchCockpit2, NAN);
    if (!Finite(c2) || std::abs(c2) > 30.0f) return NAN;
    if (g_cockpit2Writable &&
        std::abs(c2 - g_lastCockpit2RawOut) < 0.002f) {
        c2 = static_cast<float>(static_cast<double>(c2) - g_lastCockpit2Correction);
    }
    return c2;
}

void ClearPitchOverride() {
    if (g_ownPitchOverride && g_refs.overridePitch && g_refs.overridePitchWritable) {
        XPLMSetDatai(g_refs.overridePitch, 0);
    }
    g_ownPitchOverride = false;
    g_filterInit = false;
    g_lastPitchOut = 0.0;
    g_pitchWriteCount = 0;
    g_lastCockpit2RawOut = 1.0e9f;
    g_lastCockpit2Correction = 0.0;
    g_pitchAuthorityInit = false;
}

bool WriteFdPitch(double correctionDeg, double dt, bool authority) {
    if (!g_refs.fdPitchLegacy || !g_refs.fdPitchLegacyWritable) {
        g_status.pitchWritten = false;
        g_vnav.note = "legacy FD pitch dataref is not writable";
        return false;
    }
    if (!g_refs.fdPitchCockpit2) {
        g_status.pitchWritten = false;
        g_vnav.note = "independent FD pitch baseline missing";
        return false;
    }

    const float baselineF = ReadFdPitchBaseline();
    if (!Finite(baselineF)) {
        ClearPitchOverride();
        g_status.pitchWritten = false;
        g_vnav.note = "FD pitch baseline unavailable";
        return false;
    }

    const double baseline = baselineF;
    // Never replace a near-zero FF757 pitch command with our own.  A zero
    // baseline usually means the avionics are not publishing a command (or
    // the aircraft is in a mode that must not be overridden); pass through.
    // With latched priority the plugin is the publisher, so the check is
    // skipped: a zero here is our own command being read back.
    if (!authority && std::abs(baseline) < 0.5) {
        ClearPitchOverride();
        g_status.pitchWritten = false;
        g_vnav.note = "FD pitch baseline near zero; pass-through";
        return false;
    }
    if (!g_filterInit || std::abs(baseline - g_lastBaseline) > 5.0) {
        g_filterInit = true;
        g_filteredBaseline = baseline;
        g_lastPitchOut = baseline;
    }

    const double tau = std::max(0.02, g_config.vnavSmoothingTauSec);
    const double alpha = clampd(1.0 - std::exp(-dt / tau), 0.0, 1.0);
    g_filteredBaseline = alpha * baseline + (1.0 - alpha) * g_filteredBaseline;

    // Priority: hold the reference captured when the plugin took over instead
    // of chasing the FF757's changing command.  The reference only re-centres
    // while our own correction is small, i.e. while the two controllers
    // actually agree; on disagreement it stays put and our loop owns pitch.
    double baselineForTarget = g_filteredBaseline;
    if (authority) {
        if (!g_pitchAuthorityInit) {
            g_pitchAuthorityRef = baseline;
            g_pitchAuthorityInit = true;
        } else if (std::abs(correctionDeg) < 0.5) {
            g_pitchAuthorityRef += (baseline - g_pitchAuthorityRef) * 0.02;
        }
        baselineForTarget = g_pitchAuthorityRef;
    } else {
        g_pitchAuthorityInit = false;
    }

    double target = baselineForTarget + clampd(correctionDeg,
                                               -g_config.vnavMaxCorrectionDeg,
                                               g_config.vnavMaxCorrectionDeg);
    target = clampd(target, -20.0, 25.0);

    const double maxRate = g_config.vnavRateLimitDegPerSec * std::max(dt, 0.001);
    double out = g_lastPitchOut + clampd(target - g_lastPitchOut, -maxRate, maxRate);
    out = clampd(out, -20.0, 25.0);

    if (g_refs.overridePitch && g_refs.overridePitchWritable) {
        if (XPLMGetDatai(g_refs.overridePitch) == 0) {
            XPLMSetDatai(g_refs.overridePitch, 1);
            g_ownPitchOverride = true;
        } else if (g_ownPitchOverride) {
            XPLMSetDatai(g_refs.overridePitch, 1);
        }
    }

    XPLMSetDataf(g_refs.fdPitchLegacy, static_cast<float>(out));

    // The FF757 may use cockpit2 FD pitch as the actual servo command.  The
    // dataref is marked read-only in DataRefs.txt, but the FF757 itself writes
    // it, so probe it once with a small offset instead of trusting the flag.
    if (g_refs.fdPitchCockpit2 && std::abs(correctionDeg) > 0.1) {
        if (!g_cockpit2ProbeDone) {
            const float probe = static_cast<float>(out + 0.25);
            XPLMSetDataf(g_refs.fdPitchCockpit2, probe);
            const float readback = XPLMGetDataf(g_refs.fdPitchCockpit2);
            g_cockpit2Writable = Finite(readback) &&
                                 std::abs(readback - probe) < 0.05f;
            g_cockpit2ProbeDone = true;
            if (!g_cockpit2Writable) {
                g_vnav.note = "cockpit2 FD is read-only; legacy FD only";
            }
        }
        if (g_cockpit2Writable) {
            XPLMSetDataf(g_refs.fdPitchCockpit2, static_cast<float>(out));
        }
    }

    g_lastCockpit2RawOut = static_cast<float>(out);
    g_lastCockpit2Correction = correctionDeg;
    g_lastBaseline = baseline;
    g_lastPitchOut = out;
    g_lastPitchWriteTime = XPLMGetElapsedTime();
    ++g_pitchWriteCount;

    g_status.pitchWritten = true;
    return true;
}

// ---------------------------------------------------------------------------
// FD roll writing (LNAV target-level augmentation)

float ReadFdRollBaseline() {
    float c2 = RdF(g_refs.fdRollCockpit2, NAN);
    if (Finite(c2) && std::abs(c2) > 40.0f) c2 = NAN;
    if (Finite(c2) && g_cockpit2RollWritable &&
        std::abs(c2 - g_lastCockpit2RollOut) < 0.002f) {
        c2 = static_cast<float>(static_cast<double>(c2) - g_lastCockpit2RollCorrection);
    }

    float legacy = RdF(g_refs.fdRollLegacy, NAN);
    if (Finite(legacy) && std::abs(legacy) > 40.0f) legacy = NAN;
    if (Finite(legacy) &&
        std::abs(legacy - g_lastLegacyRollOut) < 0.002f) {
        legacy = static_cast<float>(static_cast<double>(legacy) - g_lastLegacyRollCorrection);
    }

    const bool c2ok = Finite(c2);
    const bool legok = Finite(legacy);
    if (!c2ok && !legok) return NAN;

    float chosen = c2ok ? c2 : legacy;
    if (c2ok && legok && std::abs(legacy) > std::abs(c2)) chosen = legacy;

    // Do not initiate a turn from an almost-zero FD bank command.  Only blend
    // into a bank the FF757 is already commanding.  This is the key safety
    // guard against a constant off-route bank / circling.
    if (std::abs(chosen) < 2.0f) return NAN;
    return chosen;
}

void ClearRollOverride() {
    if (g_ownRollOverride && g_refs.overrideRoll && g_refs.overrideRollWritable) {
        XPLMSetDatai(g_refs.overrideRoll, 0);
    }
    g_ownRollOverride = false;
    g_rollFilterInit = false;
    g_rollLastOut = 0.0;
    g_rollWriteCount = 0;
    g_lastCockpit2RollOut = 1.0e9f;
    g_lastCockpit2RollCorrection = 0.0;
    g_lastLegacyRollOut = 1.0e9f;
    g_lastLegacyRollCorrection = 0.0;
}

void ClearNavHeadingOverride() {
    if (g_ownNavHeadingOverride && g_refs.overrideNavHeading &&
        g_refs.overrideNavHeadingWritable) {
        XPLMSetDatai(g_refs.overrideNavHeading, 0);
    }
    g_ownNavHeadingOverride = false;
    g_headingDirectLogged = false;
}

bool WriteFdRoll(double correctionDeg, double dt) {
    if (!g_refs.fdRollLegacy || !g_refs.fdRollLegacyWritable) {
        g_lnav.note = "legacy FD roll dataref is not writable";
        return false;
    }
    if (!g_refs.fdRollCockpit2) {
        g_lnav.note = "independent FD roll baseline missing";
        return false;
    }

    const float baselineF = ReadFdRollBaseline();
    if (!Finite(baselineF)) {
        ClearRollOverride();
        g_lnav.note = "FD roll baseline unavailable";
        return false;
    }

    const double baseline = baselineF;
    if (!g_rollFilterInit || std::abs(baseline - g_rollLastBaseline) > 8.0) {
        g_rollFilterInit = true;
        g_rollFilteredBaseline = baseline;
        g_rollLastOut = baseline;
    }

    const double tau = std::max(0.02, g_config.lnavRollSmoothingTauSec);
    const double alpha = clampd(1.0 - std::exp(-dt / tau), 0.0, 1.0);
    g_rollFilteredBaseline = alpha * baseline + (1.0 - alpha) * g_rollFilteredBaseline;

    double target = g_rollFilteredBaseline +
                    clampd(correctionDeg,
                           -g_config.lnavMaxRollCorrectionDeg,
                           g_config.lnavMaxRollCorrectionDeg);
    target = clampd(target, -35.0, 35.0);

    const double maxRate = g_config.lnavRollRateLimitDegPerSec * std::max(dt, 0.001);
    double out = g_rollLastOut + clampd(target - g_rollLastOut, -maxRate, maxRate);
    out = clampd(out, -35.0, 35.0);

    if (g_refs.overrideRoll && g_refs.overrideRollWritable) {
        if (XPLMGetDatai(g_refs.overrideRoll) == 0) {
            XPLMSetDatai(g_refs.overrideRoll, 1);
            g_ownRollOverride = true;
        } else if (g_ownRollOverride) {
            XPLMSetDatai(g_refs.overrideRoll, 1);
        }
    }

    XPLMSetDataf(g_refs.fdRollLegacy, static_cast<float>(out));

    if (g_refs.fdRollCockpit2 && std::abs(correctionDeg) > 0.1) {
        if (!g_cockpit2RollProbeDone) {
            const float probe = static_cast<float>(out + 0.25);
            XPLMSetDataf(g_refs.fdRollCockpit2, probe);
            const float readback = XPLMGetDataf(g_refs.fdRollCockpit2);
            g_cockpit2RollWritable = Finite(readback) &&
                                     std::abs(readback - probe) < 0.05f;
            g_cockpit2RollProbeDone = true;
            if (!g_cockpit2RollWritable) {
                g_lnav.note = "cockpit2 FD roll read-only; legacy FD path";
            }
        }
        if (g_cockpit2RollWritable) {
            XPLMSetDataf(g_refs.fdRollCockpit2, static_cast<float>(out));
        }
    }

    g_lastCockpit2RollOut = static_cast<float>(out);
    g_lastCockpit2RollCorrection = correctionDeg;
    g_lastLegacyRollOut = static_cast<float>(out);
    g_lastLegacyRollCorrection = correctionDeg;
    g_rollLastBaseline = baseline;
    g_rollLastOut = out;
    ++g_rollWriteCount;
    return true;
}

// ---------------------------------------------------------------------------
// diagnostics

std::string PrintablePreview(const std::vector<char>& buf, int maxChars = 240) {
    std::string out;
    int n = std::min(static_cast<int>(buf.size()), maxChars);
    for (int i = 0; i < n; ++i) {
        unsigned char c = static_cast<unsigned char>(buf[static_cast<size_t>(i)]);
        if (c == '\r' || c == '\n') out += "\\n";
        else if (c == '\t') out += "\\t";
        else if (c == '"') out += "\\\"";
        else if (c >= 32 && c < 127) out += static_cast<char>(c);
        else out += '.';
    }
    return out;
}

void ProbeDataRef(const char* name) {
    XPLMDataRef r = XPLMFindDataRef(name);
    if (!r) {
        Logf("probe %-68s MISSING", name);
        return;
    }

    const int types = XPLMGetDataRefTypes(r);
    std::string desc;
    char tmp[256];

    if (types & xplmType_Int) {
        std::snprintf(tmp, sizeof(tmp), " int=%d", XPLMGetDatai(r));
        desc += tmp;
    }
    if (types & xplmType_Float) {
        std::snprintf(tmp, sizeof(tmp), " float=%.4f", XPLMGetDataf(r));
        desc += tmp;
    }
    if (types & xplmType_Double) {
        std::snprintf(tmp, sizeof(tmp), " double=%.4f", XPLMGetDatad(r));
        desc += tmp;
    }
    if (types & xplmType_IntArray) {
        int values[8] = {0};
        int n = XPLMGetDatavi(r, values, 0, 8);
        std::snprintf(tmp, sizeof(tmp), " iarr[%d]={%d,%d,%d,%d}", n,
                      values[0], values[1], values[2], values[3]);
        desc += tmp;
    }
    if (types & xplmType_FloatArray) {
        float values[8] = {0};
        int n = XPLMGetDatavf(r, values, 0, 8);
        std::snprintf(tmp, sizeof(tmp), " farr[%d]={%.4f,%.4f,%.4f,%.4f}", n,
                      values[0], values[1], values[2], values[3]);
        desc += tmp;
    }
    if ((types & xplmType_Data) && !(types & (xplmType_IntArray | xplmType_FloatArray))) {
        std::vector<char> buf(1024, 0);
        int n = XPLMGetDatab(r, buf.data(), 0, static_cast<int>(buf.size()));
        if (n > 0) {
            buf.resize(static_cast<size_t>(n));
            std::snprintf(tmp, sizeof(tmp), " data[%d]=\"%s\"", n,
                          PrintablePreview(buf).c_str());
            desc += tmp;
        } else {
            desc += " data[0]";
        }
    }

    Logf("probe %-68s types=0x%02x%s", name, types, desc.c_str());
}

void DumpCduScreen(const char* side);

void LogRouteProbe() {
    Logf("---- dataref probe begin ----");
    static const char* kCandidates[] = {
        "1-sim/for-internal-usage-only/fms",
        "1-sim/for-internal-usage-only/fms/rteData/rteCount",
        "1-sim/for-internal-usage-only/fms/rteData/rteOffsets",
        "1-sim/for-internal-usage-only/fms/rteData/rteTypes",
        "1-sim/for-internal-usage-only/fms/rteData/rteSizes",
        "1-sim/for-internal-usage-only/fms/rteData/wptsLat",
        "1-sim/for-internal-usage-only/fms/rteData/wptsLon",
        "1-sim/for-internal-usage-only/fms/rteData/wptsEstimatedAlt",
        "1-sim/for-internal-usage-only/fms/rteData/wptsId",
        "1-sim/for-internal-usage-only/fms/rteData/actWpt/idx",
        "1-sim/for-internal-usage-only/fms/rteData/vtk",
        "757Avionics/fms/rteData/rteCount",
        "757Avionics/fms/rteData/wptsLat",
        "757Avionics/fms/rteData/wptsLon",
        "757Avionics/fms/rteData/actWpt/idx",
        "757Avionics/rteData/rteCount",
        "757Avionics/rteData/wptsLat",
        "1-sim/rteData/rteCount",
        "1-sim/rteData/wptsLat",
        "rteData/rteCount",
        "rteData/wptsLat",
        "for-internal-usage-only/fms/rteData/rteCount",
        "for-internal-usage-only/fms/rteData/wptsLat",
        "1-sim/cduL/display/numOfLines",
        "1-sim/cduL/display/numOfSymbolsInLine",
        "1-sim/cduL/display/symbols",
        "1-sim/cduR/display/numOfLines",
        "1-sim/cduR/display/numOfSymbolsInLine",
        "1-sim/cduR/display/symbols",
        "sim/cockpit2/fmc/num_entries",
        "sim/cockpit2/fmc/fms_plan_nav_id",
        "sim/cockpit2/fmc/fms_plan_altitude",
        "sim/cockpit2/fmc/fms_plan_latitude",
        "sim/cockpit2/fmc/fms_plan_longitude",
        "sim/cockpit2/fmc/fms_plan_type",
        "sim/cockpit2/fmc/fms_plan_airport_id",
        "757Avionics/fms/AirnavDtk",
        "757Avionics/fms/AirnavDis",
        "757Avionics/fms/AirnavBrg",
        "757Avionics/fms/vnav_phase",
        "757Avionics/ap/roll_mode_engaged",
        "757Avionics/ap/pitch_mode_engaged",
        "sim/operation/override/override_nav_heading",
        "sim/operation/override/override_flightdir_roll",
        "sim/cockpit2/autopilot/flight_director_roll_deg",
        "sim/cockpit2/autopilot/flight_director_pitch_deg",
        "sim/cockpit/autopilot/nav_steer_deg_mag",
        "sim/cockpit/radios/gps_course_degtm",
        "sim/cockpit/radios/gps_dme_dist_m",
        "sim/cockpit/radios/gps_hdef_dot",
    };
    for (const char* name : kCandidates) ProbeDataRef(name);

    for (int i = 1; i <= 14; ++i) {
        char name[64];
        std::snprintf(name, sizeof(name), "1-sim/AP/lamp/%d", i);
        ProbeDataRef(name);
    }
    DumpCduScreen("cduL");
    DumpCduScreen("cduR");
    Logf("---- dataref probe end ----");
}

// FF757 CDU screen text is exposed through 1-sim/cduL|R/display/*.  It is a
// usable fallback route source when the private rteData tree is unavailable.
void DumpCduScreen(const char* side) {
    std::string base = std::string("1-sim/") + side + "/display/";
    XPLMDataRef linesRef = XPLMFindDataRef((base + "numOfLines").c_str());
    XPLMDataRef widthRef = XPLMFindDataRef((base + "numOfSymbolsInLine").c_str());
    XPLMDataRef symRef = XPLMFindDataRef((base + "symbols").c_str());
    if (!linesRef || !widthRef || !symRef) {
        Logf("CDU %s display refs missing", side);
        return;
    }
    const int lines = std::min(std::max(XPLMGetDatai(linesRef), 0), 32);
    const int width = std::min(std::max(XPLMGetDatai(widthRef), 0), 64);
    if (lines <= 0 || width <= 0) {
        Logf("CDU %s display empty (%dx%d)", side, lines, width);
        return;
    }
    std::vector<char> buf(static_cast<size_t>(lines) * static_cast<size_t>(width), ' ');
    const int n = XPLMGetDatab(symRef, buf.data(), 0, static_cast<int>(buf.size()));
    Logf("---- CDU %s text begin (%d lines x %d, %d bytes) ----", side, lines, width, n);
    for (int row = 0; row < lines; ++row) {
        std::string text;
        for (int col = 0; col < width; ++col) {
            const size_t idx = static_cast<size_t>(row) * static_cast<size_t>(width) +
                               static_cast<size_t>(col);
            char c = idx < static_cast<size_t>(n) ? buf[idx] : ' ';
            if (c < 32 || c > 126) c = ' ';
            text.push_back(c);
        }
        Logf("CDU|%s|", text.c_str());
    }
    Logf("---- CDU %s text end ----", side);
}

std::uint64_t RouteSignature(const RouteSnapshot& r) {
    std::uint64_t h = 1469598103934665603ull;
    auto mix = [&h](std::uint64_t v) {
        h ^= v;
        h *= 1099511628211ull;
    };
    mix(r.valid ? 1u : 0u);
    mix(static_cast<std::uint64_t>(r.wptCount));
    mix(static_cast<std::uint64_t>(r.activeWpt + 1));
    for (const RouteWaypoint& w : r.wpts) {
        for (char c : w.id) mix(static_cast<std::uint64_t>(static_cast<unsigned char>(c)));
        mix(static_cast<std::uint64_t>(w.altValid ? w.altFt + 1 : 0));
        mix(static_cast<std::uint64_t>(std::llround(w.lat * 1.0e6)));
        mix(static_cast<std::uint64_t>(std::llround(w.lon * 1.0e6)));
    }
    for (const RouteLeg& l : r.legs) {
        mix(static_cast<std::uint64_t>(l.firstWpt + 1));
        mix(static_cast<std::uint64_t>(l.lastWpt + 1));
        mix(l.complex ? 1u : 0u);
    }
    return h;
}

void LogRouteSnapshot(const RouteSnapshot& r) {
    Logf("---- FF757 route begin ----");
    Logf("route: valid=%d source=%d rteCount=%d wpts=%d legs=%d active=%d activeLeg=%d nextLeg=%d vtk=%.2f error=%s",
         r.valid ? 1 : 0, r.sourceRoute ? 1 : 0, r.rawRteCount, r.wptCount,
         r.legCount, r.activeWpt, r.activeLeg, r.nextLeg, r.vtk,
         r.validationError.empty() ? "-" : r.validationError.c_str());
    const size_t maxLegs = std::min<size_t>(r.legs.size(), 64);
    for (size_t i = 0; i < maxLegs; ++i) {
        const RouteLeg& l = r.legs[i];
        Logf("leg[%zu] type=%d first=%d last=%d complex=%d", i, l.type,
             l.firstWpt, l.lastWpt, l.complex ? 1 : 0);
    }
    const size_t maxWpts = std::min<size_t>(r.wpts.size(), 64);
    for (size_t i = 0; i < maxWpts; ++i) {
        const RouteWaypoint& w = r.wpts[i];
        Logf("wpt[%zu] id=%s lat=%.6f lon=%.6f alt=%d altValid=%d", i,
             w.id.empty() ? "-" : w.id.c_str(), w.lat, w.lon, w.altFt,
             w.altValid ? 1 : 0);
    }
    Logf("activeId=%s nextId=%s",
         r.activeId.empty() ? "-" : r.activeId.c_str(),
         r.nextId.empty() ? "-" : r.nextId.c_str());
    Logf("---- FF757 route end ----");
}

void LogCurrentDiagnostics() {
    LogRouteSnapshot(g_routeReader.route());
    Logf("diag: apMode=%d rollMode=%d pitchMode=%d vnavPhase=%d overrideNav=%d",
         g_status.apMode, g_status.rollMode, g_status.pitchMode, g_status.vnavPhase,
         RdI(g_refs.overrideNavHeading));
    Logf("diag: fdRoll=%.2f fdPitch=%.2f navSteer=%.2f gpsCourse=%.1f gpsDist=%.2f gpsDot=%.3f nmPerDot=%.3f fromTo=%d",
         RdF(g_refs.fdRollCockpit2), RdF(g_refs.fdPitchCockpit2),
         RdF(g_refs.navSteer), RdF(g_refs.gpsCourse), RdF(g_refs.gpsDist),
         RdF(g_refs.gpsHdefDot), RdF(g_refs.gpsHdefNmPerDot), RdI(g_refs.gpsFromTo));
    Logf("diag: aircraftIsFF=%d ffRefs=%d routeBound=%d routeValid=%d cdu=%d lnavMode=%d vnavMode=%d",
         g_status.aircraftIsFF ? 1 : 0, g_refs.ffDatarefsFound ? 1 : 0,
         g_status.routeBound ? 1 : 0, g_status.routeValid ? 1 : 0,
         g_routeReader.cduBased() ? 1 : 0,
         g_status.lnavMode ? 1 : 0, g_status.vnavMode ? 1 : 0);
    Logf("diag: cduState=%s cduCacheLegs=%d cduPages=%d scanning=%d vnavSpeed=%d selAlt=%.0f",
         g_routeReader.cduScanState().c_str(), g_routeReader.cduCacheLegs(),
         g_routeReader.cduCachePages(), g_routeReader.cduScanning() ? 1 : 0,
         g_state.vnavSpeedMode ? 1 : 0, g_state.selectedAltFt);
    Logf("diag: cduError=%s", g_routeReader.cduError().c_str());
}

// ---------------------------------------------------------------------------
// menu / commands

enum MenuId {
    MI_ENABLE = 1,
    MI_LNAV,
    MI_VNAV,
    MI_SMOOTHING,
    MI_FD_ROLL,
    MI_CDI_SOURCE,
    MI_CDU_SCAN,
    MI_STATUS,
    MI_RELOAD,
    MI_RESET,
    MI_DUMP,
    MI_ABOUT
};

void SaveConfig() {
    if (!EnsureParentDirectory(ConfigPath())) {
        Logf("cannot create config directory");
    }
    if (!g_config.save(ConfigPath())) {
        Logf("cannot save config to %s", ConfigPath().c_str());
    }
}

void UpdateMenuChecks() {
    if (!g_menu) return;
    XPLMCheckMenuItem(g_menu, 0, g_config.enabled ? xplm_Menu_Checked : xplm_Menu_Unchecked);
    XPLMCheckMenuItem(g_menu, 1, g_config.enableLnav ? xplm_Menu_Checked : xplm_Menu_Unchecked);
    XPLMCheckMenuItem(g_menu, 2, g_config.enableVnav ? xplm_Menu_Checked : xplm_Menu_Unchecked);
    XPLMCheckMenuItem(g_menu, 3, g_config.enableVnavSmoothing ? xplm_Menu_Checked : xplm_Menu_Unchecked);
    XPLMCheckMenuItem(g_menu, 4, g_config.enableLnavFdRoll ? xplm_Menu_Checked : xplm_Menu_Unchecked);
    XPLMCheckMenuItem(g_menu, 5, g_config.lnavForceCdi ? xplm_Menu_Checked : xplm_Menu_Unchecked);
}

void SetWindowVisible(bool visible) {
    g_statusVisible = visible;
    if (g_statusWindow) XPLMSetWindowIsVisible(g_statusWindow, visible ? 1 : 0);
}

void ToggleEnabled() {
    g_config.enabled = !g_config.enabled;
    g_pub.enabled = g_config.enabled ? 1 : 0;
    if (!g_config.enabled) {
        ClearPitchOverride();
        ClearRollOverride();
        ClearNavHeadingOverride();
    }
    SaveConfig();
    UpdateMenuChecks();
    Logf("plugin %s", g_config.enabled ? "enabled" : "disabled");
}

void ToggleLnav() {
    g_config.enableLnav = !g_config.enableLnav;
    g_pub.lnavEnabled = g_config.enableLnav ? 1 : 0;
    if (!g_config.enableLnav) {
        ClearRollOverride();
        ClearNavHeadingOverride();
    }
    SaveConfig();
    UpdateMenuChecks();
    Logf("LNAV enhancement %s", g_config.enableLnav ? "enabled" : "disabled");
}

void ToggleVnav() {
    g_config.enableVnav = !g_config.enableVnav;
    g_pub.vnavEnabled = g_config.enableVnav ? 1 : 0;
    if (!g_config.enableVnav) ClearPitchOverride();
    SaveConfig();
    UpdateMenuChecks();
    Logf("VNAV enhancement %s", g_config.enableVnav ? "enabled" : "disabled");
}

void ToggleSmoothing() {
    g_config.enableVnavSmoothing = !g_config.enableVnavSmoothing;
    if (!g_config.enableVnavSmoothing) ClearPitchOverride();
    SaveConfig();
    UpdateMenuChecks();
    Logf("VNAV smoothing %s", g_config.enableVnavSmoothing ? "enabled" : "disabled");
}

void ToggleFdRoll() {
    g_config.enableLnavFdRoll = !g_config.enableLnavFdRoll;
    if (!g_config.enableLnavFdRoll) ClearRollOverride();
    SaveConfig();
    UpdateMenuChecks();
    Logf("LNAV FD roll bridge %s", g_config.enableLnavFdRoll ? "enabled" : "disabled");
}

void ToggleCdiSource() {
    g_config.lnavForceCdi = !g_config.lnavForceCdi;
    // Drop the stale heading latch so the next frame starts from the new
    // source instead of holding the previous route course.
    ClearNavHeadingOverride();
    g_guidance.reset();
    SaveConfig();
    UpdateMenuChecks();
    Logf("LNAV source: %s",
         g_config.lnavForceCdi ? "GPS CDI (forced)" : "route");
}

void ReloadConfig() {
    g_config.load(ConfigPath());
    g_config.clampValues();
    g_routeReader.setRescanInterval(g_config.cduPeriodicRescan
                                        ? g_config.cduRescanIntervalSec
                                        : 0.0);
    g_routeReader.setScanOnExecOnly(g_config.cduScanOnExecOnly);
    SyncPublishedFromConfig();
    g_guidance.reset();
    UpdateMenuChecks();
    Logf("configuration reloaded");
}

void ResetDefaults() {
    g_config = Config{};
    g_routeReader.setRescanInterval(g_config.cduPeriodicRescan
                                        ? g_config.cduRescanIntervalSec
                                        : 0.0);
    g_routeReader.setScanOnExecOnly(g_config.cduScanOnExecOnly);
    SyncPublishedFromConfig();
    g_pub.enabled = 1;
    g_pub.lnavEnabled = 1;
    g_pub.vnavEnabled = 1;
    g_guidance.reset();
    SaveConfig();
    UpdateMenuChecks();
    Logf("configuration reset to defaults");
}

void ShowAbout() {
    g_aboutUntil = XPLMGetElapsedTime() + 10.0;
    SetWindowVisible(true);
    Logf("%s v%s -- %s", LVNV_NAME, LVNV_VERSION, LVNV_DESC);
}

void MenuHandler(void* menuRef, void* itemRef) {
    (void)menuRef;
    int id = static_cast<int>(reinterpret_cast<intptr_t>(itemRef));
    switch (id) {
        case MI_ENABLE: ToggleEnabled(); break;
        case MI_LNAV: ToggleLnav(); break;
        case MI_VNAV: ToggleVnav(); break;
        case MI_SMOOTHING: ToggleSmoothing(); break;
        case MI_FD_ROLL: ToggleFdRoll(); break;
        case MI_CDI_SOURCE: ToggleCdiSource(); break;
        case MI_CDU_SCAN:
            g_routeReader.requestScan();
            Logf("CDU route rescan requested");
            break;
        case MI_STATUS: SetWindowVisible(!g_statusVisible); break;
        case MI_RELOAD: ReloadConfig(); break;
        case MI_RESET: ResetDefaults(); break;
        case MI_DUMP:
            // Diagnostics only.  A route scan happens on EXEC or the explicit
            // "Rescan CDU route" action, never as a side effect of a dump.
            LogRouteProbe();
            LogCurrentDiagnostics();
            break;
        case MI_ABOUT: ShowAbout(); break;
        default: break;
    }
}

int CommandHandler(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon) {
    (void)cmd;
    if (phase != xplm_CommandBegin) return 1;
    int id = static_cast<int>(reinterpret_cast<intptr_t>(refcon));
    switch (id) {
        case MI_ENABLE: ToggleEnabled(); break;
        case MI_LNAV: ToggleLnav(); break;
        case MI_VNAV: ToggleVnav(); break;
        case MI_FD_ROLL: ToggleFdRoll(); break;
        case MI_CDI_SOURCE: ToggleCdiSource(); break;
        case MI_CDU_SCAN:
            g_routeReader.requestScan();
            Logf("CDU route rescan requested");
            break;
        case MI_STATUS: SetWindowVisible(!g_statusVisible); break;
        case MI_DUMP:
            // Diagnostics only; no implicit rescan.
            LogRouteProbe();
            LogCurrentDiagnostics();
            break;
        default: break;
    }
    return 1;
}

void CreateMenuAndCommands() {
    XPLMMenuID pluginsMenu = XPLMFindPluginsMenu();
    int item = pluginsMenu ? XPLMAppendMenuItem(pluginsMenu, LVNV_NAME, nullptr, 1) : 0;
    if (pluginsMenu) {
        g_menu = XPLMCreateMenu(LVNV_NAME, pluginsMenu, item, MenuHandler, nullptr);
    }
    if (!g_menu) {
        Logf("could not create plugin menu");
    }

    if (g_menu) {
        XPLMAppendMenuItem(g_menu, "Enable plugin", reinterpret_cast<void*>(MI_ENABLE), 1);
        XPLMAppendMenuItem(g_menu, "LNAV enhancement", reinterpret_cast<void*>(MI_LNAV), 1);
        XPLMAppendMenuItem(g_menu, "VNAV enhancement", reinterpret_cast<void*>(MI_VNAV), 1);
        XPLMAppendMenuItem(g_menu, "VNAV smoothing", reinterpret_cast<void*>(MI_SMOOTHING), 1);
        XPLMAppendMenuItem(g_menu, "LNAV FD roll (experimental)", reinterpret_cast<void*>(MI_FD_ROLL), 1);
        XPLMAppendMenuItem(g_menu, "LNAV source: GPS CDI", reinterpret_cast<void*>(MI_CDI_SOURCE), 1);
        XPLMAppendMenuItem(g_menu, "Rescan CDU route", reinterpret_cast<void*>(MI_CDU_SCAN), 1);
        XPLMAppendMenuItem(g_menu, "Show status window", reinterpret_cast<void*>(MI_STATUS), 1);
        XPLMAppendMenuItem(g_menu, "Reload config", reinterpret_cast<void*>(MI_RELOAD), 1);
        XPLMAppendMenuItem(g_menu, "Reset defaults", reinterpret_cast<void*>(MI_RESET), 1);
        XPLMAppendMenuItem(g_menu, "Dump diagnostics", reinterpret_cast<void*>(MI_DUMP), 1);
        XPLMAppendMenuItem(g_menu, "About", reinterpret_cast<void*>(MI_ABOUT), 1);
    }
    UpdateMenuChecks();

    g_cmdToggle = XPLMCreateCommand("757lvnv/toggle", "Toggle 757 LNAV/VNAV Plus");
    g_cmdLnav = XPLMCreateCommand("757lvnv/toggle_lnav", "Toggle LNAV enhancement");
    g_cmdVnav = XPLMCreateCommand("757lvnv/toggle_vnav", "Toggle VNAV enhancement");
    g_cmdStatus = XPLMCreateCommand("757lvnv/status_window", "Toggle status window");
    g_cmdDump = XPLMCreateCommand("757lvnv/dump", "Dump 757 LNAV/VNAV Plus diagnostics");
    g_cmdFdRoll = XPLMCreateCommand("757lvnv/toggle_fd_roll", "Toggle experimental LNAV FD roll bridge");
    g_cmdCdiSource = XPLMCreateCommand("757lvnv/toggle_cdi_source", "Toggle LNAV source between route and GPS CDI");

    if (g_cmdToggle) XPLMRegisterCommandHandler(g_cmdToggle, CommandHandler, 1, reinterpret_cast<void*>(MI_ENABLE));
    if (g_cmdLnav) XPLMRegisterCommandHandler(g_cmdLnav, CommandHandler, 1, reinterpret_cast<void*>(MI_LNAV));
    if (g_cmdVnav) XPLMRegisterCommandHandler(g_cmdVnav, CommandHandler, 1, reinterpret_cast<void*>(MI_VNAV));
    if (g_cmdFdRoll) XPLMRegisterCommandHandler(g_cmdFdRoll, CommandHandler, 1, reinterpret_cast<void*>(MI_FD_ROLL));
    if (g_cmdCdiSource) XPLMRegisterCommandHandler(g_cmdCdiSource, CommandHandler, 1, reinterpret_cast<void*>(MI_CDI_SOURCE));
    if (g_cmdStatus) XPLMRegisterCommandHandler(g_cmdStatus, CommandHandler, 1, reinterpret_cast<void*>(MI_STATUS));
    if (g_cmdDump) XPLMRegisterCommandHandler(g_cmdDump, CommandHandler, 1, reinterpret_cast<void*>(MI_DUMP));
}

void DestroyMenuAndCommands() {
    if (g_cmdToggle) XPLMUnregisterCommandHandler(g_cmdToggle, CommandHandler, 1, reinterpret_cast<void*>(MI_ENABLE));
    if (g_cmdLnav) XPLMUnregisterCommandHandler(g_cmdLnav, CommandHandler, 1, reinterpret_cast<void*>(MI_LNAV));
    if (g_cmdVnav) XPLMUnregisterCommandHandler(g_cmdVnav, CommandHandler, 1, reinterpret_cast<void*>(MI_VNAV));
    if (g_cmdFdRoll) XPLMUnregisterCommandHandler(g_cmdFdRoll, CommandHandler, 1, reinterpret_cast<void*>(MI_FD_ROLL));
    if (g_cmdCdiSource) XPLMUnregisterCommandHandler(g_cmdCdiSource, CommandHandler, 1, reinterpret_cast<void*>(MI_CDI_SOURCE));
    if (g_cmdStatus) XPLMUnregisterCommandHandler(g_cmdStatus, CommandHandler, 1, reinterpret_cast<void*>(MI_STATUS));
    if (g_cmdDump) XPLMUnregisterCommandHandler(g_cmdDump, CommandHandler, 1, reinterpret_cast<void*>(MI_DUMP));
    g_cmdToggle = g_cmdLnav = g_cmdVnav = g_cmdFdRoll = g_cmdCdiSource =
        g_cmdStatus = g_cmdDump = nullptr;
    if (g_menu) {
        XPLMDestroyMenu(g_menu);
        g_menu = nullptr;
    }
}

// ---------------------------------------------------------------------------
// status window

void DrawText(int x, int y, const char* text, float r, float g, float b) {
    float color[3] = {r, g, b};
    XPLMDrawString(color, x, y, const_cast<char*>(text), nullptr, xplmFont_Basic);
}

void DrawStatusWindow(XPLMWindowID window, void* refcon) {
    (void)refcon;
    int left = 0, top = 0, right = 0, bottom = 0;
    XPLMGetWindowGeometry(window, &left, &top, &right, &bottom);
    XPLMDrawTranslucentDarkBox(left, top, right, bottom);

    int x = left + 10;
    int y = top - 18;
    const int lineH = 15;

    DrawText(x, y, LVNV_NAME "  v" LVNV_VERSION, 0.55f, 0.85f, 1.0f);
    y -= lineH;
    DrawText(x, y, "Unofficial enhancement layer - does not modify SASL", 0.75f, 0.75f, 0.75f);
    y -= lineH + 4;

    char buf[512];

    std::snprintf(buf, sizeof(buf), "Plugin  : %s   Aircraft: %s",
                  g_config.enabled ? "ENABLED" : "DISABLED",
                  g_status.aircraftIsFF ? "FF757/767" : "not FF757/767");
    DrawText(x, y, buf, 1, 1, 1);
    y -= lineH;

    std::snprintf(buf, sizeof(buf), "AP      : mode=%d  roll=%d  pitch=%d  vnavPhase=%d",
                  g_status.apMode, g_status.rollMode, g_status.pitchMode, g_status.vnavPhase);
    DrawText(x, y, buf, 1, 1, 1);
    y -= lineH;

    std::snprintf(buf, sizeof(buf), "          engaged=%d  lnavMode=%d  navOverride=%d  vnavMode=%d",
                  g_status.apEngaged ? 1 : 0, g_status.lnavMode ? 1 : 0,
                  g_status.lnavOverride ? 1 : 0, g_status.vnavMode ? 1 : 0);
    DrawText(x, y, buf, 1, 1, 1);
    y -= lineH + 4;

    const LnavResult& ln = g_status.lnav;
    std::snprintf(buf, sizeof(buf), "LNAV    : %s  src=%s  xtk=%+.2f nm  dist=%.1f nm",
                  ln.active ? "ACTIVE" : "idle", ln.source.empty() ? "-" : ln.source.c_str(),
                  ln.xtkNm, ln.distanceNm);
    DrawText(x, y, buf, ln.active ? 0.55f : 0.8f, ln.active ? 1.0f : 0.8f, ln.active ? 0.55f : 0.8f);
    y -= lineH;

    std::snprintf(buf, sizeof(buf), "          base=%6.1f  target=%6.1f  corr=%+.1f deg  %s=%.0f%%",
                  ln.baseCourseMagDeg, ln.desiredCourseMagDeg, ln.correctionDeg,
                  ln.source == "RF ARC" ? "RFarc" : "turn",
                  ln.turnProgress * 100.0);
    DrawText(x, y, buf, 0.9f, 0.9f, 0.9f);
    y -= lineH;

    std::snprintf(buf, sizeof(buf), "          written=%d  active #%d -> next #%d",
                  g_status.navWritten ? 1 : 0, g_status.activeWpt, g_status.nextLeg);
    DrawText(x, y, buf, 0.9f, 0.9f, 0.9f);
    y -= lineH;

    // 777-style NAV UNABLE RNP: amber/red when ANP has exceeded RNP, grey
    // otherwise.  Hidden when monitoring is off or LNAV is idle.
    if (g_config.rnpEnabled && ln.active && ln.rnpNm > 0.0) {
        // "ANP(trk)" is deliberate: this value is the track-keeping error
        // against the active path (cross-track, or radial error on RF arcs),
        // not the real ANP, which is the 95% position-error estimate of the
        // navigation solution that the FMS gets from GPS/IRS.
        std::snprintf(buf, sizeof(buf), "          RNP=%.1f nm  ANP(trk)=%.2f nm  %s",
                      ln.rnpNm, ln.anpNm,
                      ln.rnpUnable ? "NAV UNABLE RNP" : "OK");
        DrawText(x, y, buf,
                 ln.rnpUnable ? 1.0f : 0.9f,
                 ln.rnpUnable ? 0.45f : 0.9f,
                 ln.rnpUnable ? 0.35f : 0.9f);
        y -= lineH;
    }
    y -= 4;

    const VnavResult& vn = g_status.vnav;
    const bool vnavWriting = vn.active && !vn.passThrough;
    // Show the submode the FF757 FMA is displaying.  "STBY" means the plugin
    // recognised the mode but deliberately left pitch alone (MCP hold, pre-TOD,
    // deadband, or VNAV SPD); it is not a detection failure.
    const char* vnavSubmodeName = g_status.vnavSubmode == 1 ? "SPD"
                                : (g_status.vnavSubmode == 2 ? "PTH" : "-");
    std::snprintf(buf, sizeof(buf), "VNAV    : %s %s  src=%s  targetAlt=%.0f ft",
                  vnavSubmodeName,
                  vnavWriting ? "ACTIVE" : (vn.passThrough ? "STBY" : "idle"),
                  vn.source.empty() ? "-" : vn.source.c_str(),
                  vn.targetAltFt);
    DrawText(x, y, buf, vnavWriting ? 0.55f : 0.8f, vnavWriting ? 1.0f : 0.8f,
             vnavWriting ? 0.55f : 0.8f);
    y -= lineH;

    std::snprintf(buf, sizeof(buf), "          reqVS=%+.0f fpm  vs=%+.0f fpm  corr=%+.2f deg",
                  vn.requiredVsFpm, g_state.vsFpm, vn.correctionDeg);
    DrawText(x, y, buf, 0.9f, 0.9f, 0.9f);
    y -= lineH;

    if (vn.targetSpdKt > 0.5) {
        std::snprintf(buf, sizeof(buf),
                      "          speed limit=%.0f kt @ %.1f NM  decel lead=%.1f NM",
                      vn.targetSpdKt, vn.targetSpdDistNm, vn.targetSpdLeadNm);
        DrawText(x, y, buf, 0.85f, 0.9f, 0.6f);
        y -= lineH;
    }

    std::snprintf(buf, sizeof(buf), "          smoothedFD=%d  written=%d  cdiSign=%+d",
                  g_config.enableVnavSmoothing ? 1 : 0, g_status.pitchWritten ? 1 : 0,
                  g_guidance.cdiSign());
    DrawText(x, y, buf, 0.9f, 0.9f, 0.9f);
    y -= lineH + 4;

    std::snprintf(buf, sizeof(buf), "CDI     : valid=%d  course=%.1f  dist=%.1f  dotXtK=%+.2f",
                  g_status.cdi.valid ? 1 : 0, g_status.cdi.courseMagDeg,
                  g_status.cdi.distanceNm, g_status.cdi.xtkNm);
    DrawText(x, y, buf, 0.9f, 0.9f, 0.9f);
    y -= lineH;

    const char* routeSource = g_routeReader.hasRouteDatarefs() ? "rteData"
                              : (g_routeReader.cduBased() ? "CDU" : "-");
    std::snprintf(buf, sizeof(buf), "Route   : %s  bound=%d  valid=%d  legs=%d  wpts=%d",
                  routeSource, g_status.routeBound ? 1 : 0, g_status.routeValid ? 1 : 0,
                  g_status.routeLegs, g_status.routeWpts);
    DrawText(x, y, buf, 0.9f, 0.9f, 0.9f);
    y -= lineH;

    std::snprintf(buf, sizeof(buf), "          active=%s  next=%s",
                  g_status.activeId.c_str(), g_status.nextId.c_str());
    DrawText(x, y, buf, 0.9f, 0.9f, 0.9f);
    y -= lineH + 4;

    std::snprintf(buf, sizeof(buf), "Config  : LNAV=%d VNAV=%d smooth=%d FDRoll=%d debug=%d",
                  g_config.enableLnav ? 1 : 0, g_config.enableVnav ? 1 : 0,
                  g_config.enableVnavSmoothing ? 1 : 0,
                  g_config.enableLnavFdRoll ? 1 : 0, g_config.debug ? 1 : 0);
    DrawText(x, y, buf, 0.9f, 0.9f, 0.9f);
    y -= lineH;

    if (!g_status.message.empty()) {
        std::snprintf(buf, sizeof(buf), "Status  : %s", g_status.message.c_str());
        DrawText(x, y, buf, 1.0f, 0.8f, 0.2f);
        y -= lineH;
    }
    if (!g_status.lnavNote.empty() || !g_status.vnavNote.empty()) {
        std::snprintf(buf, sizeof(buf), "Notes   : %s | %s",
                      g_status.lnavNote.c_str(), g_status.vnavNote.c_str());
        DrawText(x, y, buf, 0.75f, 0.75f, 0.75f);
        y -= lineH;
    }

    if (g_aboutUntil > 0.0 && XPLMGetElapsedTime() < g_aboutUntil) {
        y -= 4;
        DrawText(x, y, "Reads FF757 FMS/flight-guidance datarefs; writes only standard X-Plane", 0.6f, 0.9f, 1.0f);
        y -= lineH;
        DrawText(x, y, "nav/FD datarefs. No aircraft files or SASL scripts are touched.", 0.6f, 0.9f, 1.0f);
    }

    std::snprintf(buf, sizeof(buf), "Menu: Plugins -> %s", LVNV_NAME);
    DrawText(left + 10, bottom + 10, buf, 0.65f, 0.65f, 0.65f);
}

void CreateStatusWindow() {
    int sl = 0, st = 0, sr = 0, sb = 0;
    XPLMGetScreenBoundsGlobal(&sl, &st, &sr, &sb);

    XPLMCreateWindow_t params{};
    params.structSize = static_cast<int>(sizeof(params));
    params.left = sl + 60;
    params.top = st - 60;
    params.right = params.left + 560;
    params.bottom = params.top - 380;
    params.visible = 0;
    params.drawWindowFunc = DrawStatusWindow;
    params.handleMouseClickFunc = nullptr;
    params.handleKeyFunc = nullptr;
    params.handleCursorFunc = nullptr;
    params.handleMouseWheelFunc = nullptr;
    params.refcon = nullptr;
    params.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;
    params.layer = xplm_WindowLayerFloatingWindows;

    g_statusWindow = XPLMCreateWindowEx(&params);
    if (g_statusWindow) {
        XPLMSetWindowTitle(g_statusWindow, LVNV_NAME);
        XPLMSetWindowIsVisible(g_statusWindow, 0);
    } else {
        Logf("could not create status window");
    }
}

// ---------------------------------------------------------------------------
// flight loop

void UpdateStatusFromResults(const LnavResult& ln, const VnavResult& vn,
                             const CdiInput& cdi, bool navWritten, bool pitchWritten) {
    g_status.cdi = cdi;
    g_status.lnav = ln;
    g_status.vnav = vn;
    g_status.navWritten = navWritten;
    g_status.pitchWritten = pitchWritten;

    // Log the NAV UNABLE RNP transitions once per change, like the 777 FMC
    // caution, so the event is traceable in Log.txt after the flight.
    static bool s_prevRnpUnable = false;
    if (ln.rnpUnable != s_prevRnpUnable) {
        s_prevRnpUnable = ln.rnpUnable;
        if (ln.rnpUnable) {
            Logf("NAV UNABLE RNP: ANP %.2f nm exceeds RNP %.2f nm",
                 ln.anpNm, ln.rnpNm);
        } else {
            Logf("RNP alert cleared: ANP %.2f nm within RNP %.2f nm",
                 ln.anpNm, ln.rnpNm);
        }
    }

    g_status.xtkNm = ln.xtkNm;
    g_status.distanceNm = ln.distanceNm;
    g_status.baseCourse = ln.baseCourseMagDeg;
    g_status.desiredCourse = ln.desiredCourseMagDeg;
    g_status.lnavCorrection = ln.correctionDeg;
    g_status.targetAltFt = vn.targetAltFt;
    g_status.requiredVs = vn.requiredVsFpm;
    g_status.vsError = vn.vsErrorFpm;
    g_status.vnavCorrection = vn.correctionDeg;
    g_status.lnavNote = ln.note;
    g_status.vnavNote = vn.note;

    // Log the VNAV authority state whenever it changes.  This makes it easy to
    // tell a pass-through mode (FF757 owns pitch) from a plugin correction.
    if (vn.source != g_lastVnavSource) {
        Logf("VNAV authority: source=%s active=%d passThrough=%d corr=%+.2f target=%.0f written=%d note=%s",
             vn.source.empty() ? "-" : vn.source.c_str(), vn.active ? 1 : 0,
             vn.passThrough ? 1 : 0, vn.correctionDeg, vn.targetAltFt,
             pitchWritten ? 1 : 0, vn.note.c_str());
        g_lastVnavSource = vn.source;
    }
    if (ln.source != g_lastLnavSource) {
        Logf("LNAV authority: source=%s active=%d wpt=%s leg=%d xtk=%+.2f note=%s",
             ln.source.empty() ? "-" : ln.source.c_str(), ln.active ? 1 : 0,
             ln.activeWpt >= 0 && ln.activeWpt < static_cast<int>(g_routeReader.route().wpts.size())
                 ? g_routeReader.route().wpts[static_cast<size_t>(ln.activeWpt)].id.c_str()
                 : "-",
             ln.activeLeg, ln.xtkNm, ln.note.c_str());
        g_lastLnavSource = ln.source;
    }
    if (g_cockpit2ProbeDone && !g_cockpit2Writable && g_refs.fdPitchCockpit2) {
        if (!g_status.vnavNote.empty()) g_status.vnavNote += " | ";
        g_status.vnavNote += "cockpit2 FD read-only; legacy FD path";
    }

    const RouteSnapshot& r = g_routeReader.route();
    // Dump the route when it first becomes valid, and again whenever a rescan
    // or sequencing changes its content (idents, altitudes, coordinates, active
    // index).  This makes route edits visible in the log without logging every
    // frame.
    const std::uint64_t routeSig = r.valid ? RouteSignature(r) : 0;
    const bool routeChangedForLog =
        r.valid && (!g_loggedRouteValid || routeSig != g_loggedRouteSignature);
    if (r.valid && (!g_status.routeValid || routeChangedForLog)) {
        LogRouteSnapshot(r);
        Logf("guidance: lnav=%d vnav=%d targetAlt=%.0f reqVs=%.0f notes=<%s>|<%s>",
             ln.active ? 1 : 0, vn.active ? 1 : 0, vn.targetAltFt,
             vn.requiredVsFpm, ln.note.c_str(), vn.note.c_str());
        Logf("guidance: vnavSpeed=%d selAlt=%.0f source=%s",
             g_state.vnavSpeedMode ? 1 : 0, g_state.selectedAltFt,
             vn.source.empty() ? "-" : vn.source.c_str());
        g_loggedRouteSignature = routeSig;
        g_loggedRouteValid = true;
    } else if (!r.valid) {
        g_loggedRouteSignature = 0;
        g_loggedRouteValid = false;
    }
    g_status.routeValid = r.valid;
    g_status.routeLegs = r.legCount;
    g_status.routeWpts = r.wptCount;
    g_status.activeWpt = r.activeWpt;
    g_status.activeLeg = r.activeLeg;
    g_status.nextLeg = r.nextLeg;
    g_status.activeId = r.activeId.empty() ? "-" : r.activeId;
    g_status.nextId = r.nextId.empty() ? "-" : r.nextId;

    g_pub.status = !g_config.enabled ? 0
                  : (!g_status.aircraftIsFF ? 3
                  : ((ln.active && g_config.enableLnav && navWritten) ||
                     (vn.active && g_config.enableVnav && pitchWritten) ? 2 : 1));
    g_pub.lnavActive = (ln.active && g_config.enableLnav && navWritten) ? 1 : 0;
    g_pub.vnavActive = (vn.active && g_config.enableVnav && pitchWritten) ? 1 : 0;
    g_pub.routeValid = r.valid ? 1 : 0;
    g_pub.routeSource = g_routeReader.hasRouteDatarefs()
                            ? 1
                            : (g_routeReader.cduBased() ? 2 : 0);
    g_pub.routeLegs = r.legCount;
    g_pub.routeWpts = r.wptCount;
    g_pub.activeWpt = r.activeWpt;
    g_pub.activeLeg = r.activeLeg;
    g_pub.nextLeg = r.nextLeg;
    g_pub.apMode = g_status.apMode;
    g_pub.rollMode = g_status.rollMode;
    g_pub.pitchMode = g_status.pitchMode;
    g_pub.vnavPhase = g_status.vnavPhase;
    g_pub.apEngaged = g_state.apEngaged ? 1 : 0;
    g_pub.lnavOverride = (g_state.lnavOverride || g_ownNavHeadingOverride) ? 1 : 0;

    g_pub.xtkNm = static_cast<float>(ln.xtkNm);
    g_pub.distanceNm = static_cast<float>(ln.distanceNm);
    g_pub.baseCourseDeg = static_cast<float>(ln.baseCourseMagDeg);
    g_pub.desiredCourseDeg = static_cast<float>(ln.desiredCourseMagDeg);
    g_pub.lnavCorrectionDeg = static_cast<float>(ln.correctionDeg);
    g_pub.targetAltFt = static_cast<float>(vn.targetAltFt);
    g_pub.requiredVsFpm = static_cast<float>(vn.requiredVsFpm);
    g_pub.vsErrorFpm = static_cast<float>(vn.vsErrorFpm);
    g_pub.vnavCorrectionDeg = static_cast<float>(vn.correctionDeg);
    g_pub.cdiXtkNm = static_cast<float>(cdi.xtkNm);
}

void SyncConfigFromPublished() {
    bool changed = false;
    if (g_pub.enabled != (g_config.enabled ? 1 : 0)) {
        g_config.enabled = g_pub.enabled != 0;
        changed = true;
    }
    if (g_pub.lnavEnabled != (g_config.enableLnav ? 1 : 0)) {
        g_config.enableLnav = g_pub.lnavEnabled != 0;
        changed = true;
    }
    if (g_pub.vnavEnabled != (g_config.enableVnav ? 1 : 0)) {
        g_config.enableVnav = g_pub.vnavEnabled != 0;
        changed = true;
    }
    if (changed) {
        g_pub.enabled = g_config.enabled ? 1 : 0;
        g_pub.lnavEnabled = g_config.enableLnav ? 1 : 0;
        g_pub.vnavEnabled = g_config.enableVnav ? 1 : 0;
        if (!g_config.enabled || !g_config.enableVnav) ClearPitchOverride();
        if (!g_config.enabled || !g_config.enableLnav) {
            ClearRollOverride();
            ClearNavHeadingOverride();
        }
        SaveConfig();
        UpdateMenuChecks();
        Logf("published switches changed: enabled=%d lnav=%d vnav=%d",
             g_config.enabled ? 1 : 0, g_config.enableLnav ? 1 : 0,
             g_config.enableVnav ? 1 : 0);
    }
}

void UpdateMessages() {
    if (!g_config.enabled) {
        g_status.message = "Plugin disabled (menu or 757lvnv/enabled)";
        return;
    }
    if (!g_status.aircraftIsFF || !g_refs.ffDatarefsFound) {
        g_status.message = "FF757/767 avionics datarefs not found; plugin idle";
        return;
    }
    if (!g_status.routeBound) {
        g_status.message = "No FF757 route source yet";
        if (!g_routeReader.lastError().empty()) {
            g_status.message += " (" + g_routeReader.lastError() + ")";
        }
        return;
    }
    if (!g_status.routeValid) {
        g_status.message = "Route not valid yet";
        if (!g_routeReader.lastError().empty()) {
            g_status.message += " (" + g_routeReader.lastError() + ")";
        }
        return;
    }
    g_status.message = "OK";
}

float FlightLoopCallback(float elapsedSinceLastCall, float, int, void*) {
    try {
        const double dt = clampd(elapsedSinceLastCall, 0.001, 0.25);
        g_status.enabled = g_config.enabled;

        SyncConfigFromPublished();

        if (RdI(g_refs.paused, 0) != 0) {
            ClearPitchOverride();
            ClearRollOverride();
            ClearNavHeadingOverride();
            g_status.message = "Simulator paused";
            return -1.0f;
        }

        if (!g_config.enabled || !g_status.aircraftIsFF || !g_refs.ffDatarefsFound) {
            const double now = XPLMGetElapsedTime();
            if (!g_refs.ffDatarefsFound && now - g_lastRefRebind > 5.0) {
                const bool wasFound = g_refs.ffDatarefsFound;
                BindRefs();
                g_lastRefRebind = now;
                g_status.aircraftIsFF = AircraftLooksLikeFF757();
                if (!wasFound && g_refs.ffDatarefsFound) {
                    Logf("FF avionics datarefs appeared; re-bound");
                    Logf("writable: nav_steer=%d fd_pitch_legacy=%d fd_pitch_c2=%d override_pitch=%d",
                         g_refs.navSteerWritable ? 1 : 0,
                         g_refs.fdPitchLegacyWritable ? 1 : 0,
                         g_refs.fdPitchCockpit2Writable ? 1 : 0,
                         g_refs.overridePitchWritable ? 1 : 0);
                    Logf("route datarefs: %s", g_routeReader.report().c_str());
                    LogRouteProbe();
                }
            }
            ClearPitchOverride();
            ClearRollOverride();
            ClearNavHeadingOverride();
            g_guidance.reset();
            g_lastVsFpm = 0.0;
            g_vsAccel = 0.0;
            g_status.navWritten = false;
            g_status.pitchWritten = false;
            g_status.routeBound = g_routeReader.hasRouteDatarefs() || g_routeReader.cduBased();
            UpdateStatusFromResults(LnavResult{}, VnavResult{}, CdiInput{}, false, false);
            UpdateMessages();
            return -1.0f;
        }

        g_routeReader.refresh(XPLMGetElapsedTime(), false);
        ReadAircraftState(dt);
        // Use the avionics-owned roll baseline (not our own cockpit2 write) for
        // CDI sign calibration.
        const float fdRollBaseline = ReadFdRollBaseline();
        if (Finite(fdRollBaseline)) {
            g_state.fdRollDeg = fdRollBaseline;
        }
        ReadCdiInput();
        // Sample the flag before this frame's writes so the RF override logic
        // can tell whether the model let our previous write survive.
        g_navOverrideAtFrameStart = RdI(g_refs.overrideNavHeading);
        // Sample the previous frame's nav target before overwriting it: if it
        // differs from our last command, something else (the FF757) is also
        // publishing nav_steer.
        g_navSteerFrameStart = RdF(g_refs.navSteer, NAN);

        g_lnav = g_guidance.computeLnav(g_state, g_routeReader.route(), g_cdi, g_config);
        g_vnav = g_guidance.computeVnav(g_state, g_routeReader.route(), g_config, dt);

        bool navWritten = false;
        bool rollWritten = false;
        const bool rfArcNow = g_lnav.source == "RF ARC";
        if (g_config.enableLnav && g_lnav.active && g_state.apEngaged) {
            // Heading-direct guidance: the plugin owns the lateral target on
            // every LNAV leg (lnav_heading_direct, default) or, when that
            // switch is off, on RF arcs only (lnav_rf_nav_override).  Without
            // the override X-Plane's generic AP ignores nav_steer, so the
            // target would only be monitored instead of flown.
            const bool headingDirect = g_config.lnavHeadingDirect ||
                                       (rfArcNow && g_config.lnavRfNavOverride);
            double navTargetMag = normalize360(g_lnav.desiredCourseMagDeg);
            double dampUsed = 0.0;
            double l1Used = 0.0;
            if (g_refs.navSteerWritable) {
                if (headingDirect) {
                    // The nav controller consumes nav_steer_deg_mag as a
                    // HEADING, while our guidance output is a desired TRACK.
                    // Convert with the wind-correction angle (hdg - trk) and add
                    // track-error damping, the same command shape as the 787
                    // reference: heading = track + damp + WCA.
                    const double trackMag =
                        magneticFromTrue(g_state.trackTrueDeg, g_state.magVarDeg);
                    const double headingMag = RdF(g_refs.magPsi, NAN);
                    const double trackErr =
                        normalize180(trackMag - g_lnav.desiredCourseMagDeg);
                    const double damp =
                        clampd(-g_config.lnavTrackDampGain * trackErr,
                               -g_config.lnavTrackDampMaxDeg,
                               g_config.lnavTrackDampMaxDeg);
                    dampUsed = damp;
                    const double wca = Finite(headingMag)
                                           ? normalize180(headingMag - trackMag)
                                           : 0.0;
                    navTargetMag =
                        normalize360(g_lnav.desiredCourseMagDeg + damp + wca);

                    // L1 cross-track term (787 reference).  The base capture
                    // homes at atan(xtk / distance-to-fix); on a long leg that
                    // is a fraction of a degree and leaves a small steady offset
                    // (0.15 NM at 28 NM commanded only 0.3 deg).  Add the
                    // difference to an L1 look-ahead correction, clamped so the
                    // large-offset Boeing capture is not disturbed.  RF arcs
                    // already use the radial correction and are skipped.
                    if (!rfArcNow && Finite(g_lnav.xtkNm) &&
                        g_lnav.distanceNm > 0.5 && g_config.lnavL1Factor > 0.0) {
                        const double turnR =
                            turnRadiusNm(g_state.gsKts, g_config.lnavMaxBankDeg);
                        const double l1 = clampd(g_config.lnavL1Factor * turnR,
                                                 0.5, 8.0);
                        const double homingDeg = rad2deg(std::atan2(
                            -g_lnav.xtkNm, std::max(0.2, g_lnav.distanceNm)));
                        const double l1Deg = rad2deg(std::atan2(-g_lnav.xtkNm, l1));
                        const double extra = clampd(l1Deg - homingDeg, -4.0, 4.0);
                        l1Used = extra;
                        navTargetMag = normalize360(navTargetMag + extra);
                    }
                }
                XPLMSetDataf(g_refs.navSteer, static_cast<float>(navTargetMag));
                g_lastNavSteerCmd = navTargetMag;
                g_lastLnavDamp = dampUsed;
                g_lastLnavL1 = l1Used;
                navWritten = true;
                if (!g_state.lnavOverride && !headingDirect) {
                    g_lnav.note = "nav target write (override_nav_heading=0)";
                }
                if (g_config.debug && g_pitchWriteCount < 3) {
                    Logf("LNAV write: nav_steer=%.2f (base=%.2f xtk=%+.2f dist=%.2f)",
                         navTargetMag, g_lnav.baseCourseMagDeg, g_lnav.xtkNm,
                         g_lnav.distanceNm);
                }
            } else {
                g_lnav.note = "nav_steer dataref is not writable";
            }

            // Heading-direct: hold override_nav_heading every frame.  The model
            // clears it each frame - that is normal (the 787 reference counts
            // and re-asserts the same way), not a failure - so re-assert it and
            // keep nav_steer in step.  FD roll is skipped below, leaving a single
            // lateral controller in command.
            bool navOverrideLive = false;
            const bool navOverrideWanted = headingDirect && navWritten &&
                                           g_refs.overrideNavHeadingWritable;
            if (navOverrideWanted) {
                if (g_navOverrideAtFrameStart == 0) {
                    XPLMSetDatai(g_refs.overrideNavHeading, 1);
                    g_ownNavHeadingOverride = true;
                    if (!g_headingDirectLogged) {
                        g_headingDirectLogged = true;
                        Logf("LNAV: heading direct, override_nav_heading=1 (target %.1f, %s)",
                             navTargetMag, rfArcNow ? "RF arc" : "route");
                    }
                } else if (g_ownNavHeadingOverride) {
                    XPLMSetDatai(g_refs.overrideNavHeading, 1);
                }
                navOverrideLive = true;
            } else {
                ClearNavHeadingOverride();
            }

            g_status.lnavOverride = g_state.lnavOverride || navOverrideLive;
            if (navOverrideLive) {
                if (rfArcNow) {
                    g_lnav.note = "RF arc: heading direct (override=1)";
                } else if (g_lnav.note.empty() ||
                           g_lnav.note.find("nav target write") != std::string::npos) {
                    g_lnav.note = "heading direct (override=1)";
                }
            }

            // FD-roll bridge: only when the nav override is not live.  On RF arcs
            // (with heading-direct off) it is limited to +/-4 deg; the global path
            // (enable_lnav_fd_roll) uses the configured correction.
            const bool rfRoll = rfArcNow && g_config.lnavRfFdRoll && !navOverrideLive;
            const bool globalRoll = g_config.enableLnavFdRoll && !navOverrideLive;
            if (globalRoll || rfRoll) {
                double corr = g_lnav.rollCorrectionDeg;
                if (rfRoll && !globalRoll) {
                    corr = clampd(corr, -4.0, 4.0);
                }
                rollWritten = WriteFdRoll(corr, dt);
                if (rollWritten && g_lnav.note.find("read-only") == std::string::npos) {
                    if (globalRoll) {
                        g_lnav.note = navWritten ? "nav target + FD roll"
                                                 : "FD roll path (nav_steer override off)";
                    } else {
                        g_lnav.note = "RF arc FD roll (max 4 deg)";
                    }
                }
            } else {
                ClearRollOverride();
                if (g_lnav.note.empty()) {
                    g_lnav.note = "FD roll augmentation disabled (safe default)";
                }
            }
        } else {
            ClearRollOverride();
            ClearNavHeadingOverride();
        }
        const bool lnavWritten = navWritten || rollWritten;

        // Periodic LNAV diagnostic: compare the plugin's route cross-track and
        // target with the FF757's GPS CDI and with the previous nav_steer read
        // so a slow straight-leg drift can be traced to geometry, the steering
        // command, or another controller overwriting nav_steer.
        if (g_lnav.active && XPLMGetElapsedTime() - g_lastLnavServoLog > 5.0) {
            g_lastLnavServoLog = XPLMGetElapsedTime();
            const double trackMag =
                magneticFromTrue(g_state.trackTrueDeg, g_state.magVarDeg);
            Logf("LNAV servo: src=%s track=%.1f base=%.1f course=%.1f "
                 "xtk=%+.2f dist=%.1f corr=%+.2f cmd=%.1f prev=%.1f "
                 "damp=%+.2f l1=%+.2f ovr=%d fmsDot=%+.3f turn=%.0f "
                 "active=%d next=%d",
                 g_lnav.source.c_str(), trackMag, g_lnav.baseCourseMagDeg,
                 g_lnav.desiredCourseMagDeg, g_lnav.xtkNm, g_lnav.distanceNm,
                 g_lnav.correctionDeg, g_lastNavSteerCmd,
                 g_navSteerFrameStart, g_lastLnavDamp, g_lastLnavL1,
                 g_state.lnavOverride ? 1 : 0,
                 RdF(g_refs.gpsHdefDot, NAN), g_lnav.turnProgress,
                 g_lnav.activeWpt, g_lnav.nextLeg);
        }

        bool pitchWritten = false;
        if (g_config.enableVnav && g_state.apEngaged && g_state.vnavMode) {
            // The FF757 takes the pitch axis back near approach/landing: the
            // diagnostics showed servos=0 and override_artstab=1 while the
            // plugin still held priority and wrote FD pitch (out 8.8 deg).
            // Release after ~1 s of that state so a single transition frame
            // cannot start a pitch fight.
            const bool ffOwnsPitch =
                (g_refs.overrideArtstab &&
                 RdI(g_refs.overrideArtstab, 0) != 0) ||
                (g_refs.servosOn && RdI(g_refs.servosOn, 1) == 0);
            if (ffOwnsPitch) {
                ++g_ffPitchOwnCount;
            } else {
                g_ffPitchOwnCount = 0;
            }

            if (ffOwnsPitch && g_ffPitchOwnCount >= 20) {
                if (g_vnavPriority) {
                    g_vnavPriority = false;
                    g_vnavPriorityLogged = false;
                    g_vnavReleaseCount = 0;
                    Logf("VNAV pitch released: FF757 owns pitch (servos/artstab)");
                }
                ClearPitchOverride();
                g_status.pitchWritten = false;
                g_vnav.note += " | FF757 servos/artstab owns pitch";
            } else {
            // Latch plugin pitch priority while our own vertical path is
            // valid.  Without the latch the plugin drops to pass-through the
            // moment a frame looks "level" or the FD baseline reads near zero,
            // the FF757 takes pitch back, we take it again - the visible
            // tug-of-war during DES.  Release needs ~2 s of continuous
            // inactivity (40 frames at the 20 Hz flight loop).
            const bool wantPriority = g_config.vnavPriority && g_vnav.active &&
                                      g_vnav.pathBased && !g_vnav.passThrough;
            if (wantPriority) {
                if (!g_vnavPriority && !g_vnavPriorityLogged) {
                    g_vnavPriorityLogged = true;
                    Logf("VNAV pitch priority latched: reqVS=%+.0f vs=%+.0f targetAlt=%.0f",
                         g_vnav.requiredVsFpm, g_state.vsFpm, g_vnav.targetAltFt);
                }
                g_vnavPriority = true;
                g_vnavReleaseCount = 0;
            } else if (g_vnavPriority) {
                if (++g_vnavReleaseCount >= 40) {
                    g_vnavPriority = false;
                    g_vnavReleaseCount = 0;
                    Logf("VNAV pitch priority released");
                }
            } else {
                g_vnavReleaseCount = 0;
            }

            if (g_vnav.passThrough) {
                // VNAV SPD: leave the FF757's pitch/speed loop completely
                // untouched.  Writing a smoothed baseline here would still
                // fight the speed controller.
                ClearPitchOverride();
                g_status.pitchWritten = false;
            } else if (g_vnav.active) {
                pitchWritten = WriteFdPitch(g_vnav.correctionDeg, dt, g_vnavPriority);
                if (g_vnavPriority) g_vnav.note += " | plugin pitch priority";
            } else if (g_config.enableVnavSmoothing) {
                pitchWritten = WriteFdPitch(0.0, dt, false);
                if (g_vnav.note.empty()) g_vnav.note = "smoothing only; no vertical path";
            } else {
                ClearPitchOverride();
            }
            }  // end FF757-owns-pitch guard
        } else {
            g_vnavPriority = false;
            g_vnavPriorityLogged = false;
            g_vnavReleaseCount = 0;
            g_ffPitchOwnCount = 0;
            ClearPitchOverride();
        }

        // Periodic pitch-authority diagnostic while the plugin is flying a
        // vertical path.  It answers whether our legacy FD write sticks and
        // whether X-Plane's AP (servos) or an artstab override owns pitch.
        if (g_vnav.active && g_vnav.pathBased &&
            XPLMGetElapsedTime() - g_lastVnavServoLog > 5.0) {
            g_lastVnavServoLog = XPLMGetElapsedTime();
            Logf("VNAV servo: corr=%+.2f out=%.2f legacyRead=%.2f fdC2=%.2f "
                 "ovrFd=%d apOn=%d servos=%d altMode=%d apState=0x%x "
                 "ovrArtstab=%d artPitch=%.3f vs=%+.0f req=%+.0f "
                 "pathErr=%+.0f held=%d",
                 g_vnav.correctionDeg, g_lastPitchOut,
                 RdF(g_refs.fdPitchLegacy, NAN),
                 RdF(g_refs.fdPitchCockpit2, NAN),
                 RdI(g_refs.overridePitch, 0),
                 RdI(g_refs.autopilotOn, 0), RdI(g_refs.servosOn, 0),
                 RdI(g_refs.altitudeMode, 0), RdI(g_refs.apState, 0),
                 RdI(g_refs.overrideArtstab, 0),
                 RdF(g_refs.artstabPitch, NAN),
                 g_state.vsFpm, g_vnav.requiredVsFpm,
                 g_vnav.pathErrorFt, g_vnav.pathHeld ? 1 : 0);
        }

        // A CDU LEGS-page cache is a valid route source on builds that do not
        // expose the private rteData tree.
        g_status.routeBound = g_routeReader.hasRouteDatarefs() ||
                              g_routeReader.cduBased();
        UpdateStatusFromResults(g_lnav, g_vnav, g_cdi, lnavWritten, pitchWritten);
        UpdateMessages();

        if (g_config.debug && pitchWritten && g_pitchWriteCount <= 3) {
            Logf("VNAV write: corr=%+.2f out=%.2f reqVS=%+.0f vs=%+.0f",
                 g_vnav.correctionDeg, g_filteredBaseline + g_vnav.correctionDeg,
                 g_vnav.requiredVsFpm, g_state.vsFpm);
        }
    } catch (const std::exception& e) {
        ++g_errorCount;
        g_lastError = e.what();
        Logf("exception in flight loop: %s", e.what());
        ClearPitchOverride();
        ClearRollOverride();
        ClearNavHeadingOverride();
    } catch (...) {
        ++g_errorCount;
        g_lastError = "unknown exception";
        Logf("unknown exception in flight loop");
        ClearPitchOverride();
        ClearRollOverride();
        ClearNavHeadingOverride();
    }
    return -1.0f;
}

void ResetRuntimeState() {
    g_routeReader.reset();
    g_guidance.reset();
    g_state = AircraftState{};
    g_cdi = CdiInput{};
    g_lnav = LnavResult{};
    g_vnav = VnavResult{};
    g_status = Status{};
    g_lastVsFpm = 0.0;
    g_vsAccel = 0.0;
    g_lastFlightTime = -1.0;
    g_loggedRouteSignature = 0;
    g_loggedRouteValid = false;
    g_lastVnavSource.clear();
    g_lastLnavSource.clear();
    g_vnavPriority = false;
    g_vnavPriorityLogged = false;
    g_vnavReleaseCount = 0;
    g_pitchAuthorityInit = false;
    ClearPitchOverride();
    ClearRollOverride();
    ClearNavHeadingOverride();
}

// ---------------------------------------------------------------------------
// XPLM entry points

PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc) {
    std::snprintf(outName, 256, "%s", LVNV_NAME);
    std::snprintf(outSig, 256, "%s", LVNV_SIG);
    std::snprintf(outDesc, 1024, "%s", LVNV_DESC);

    g_logPath = lvnv::SystemPath() + "Output/757LVNAVPlus.log";
    if (lvnv::EnsureParentDirectory(g_logPath)) {
        g_logFile.open(g_logPath, std::ios::trunc);
    }
    if (!g_logFile.is_open()) g_logFailed = true;

    lvnv::Logf("%s v%s starting", LVNV_NAME, LVNV_VERSION);
    lvnv::Logf("license: free to redistribute (original or modified, incl. commercial); "
               "author credit \"JACKGONG\" is required. MIT, (c) 2026 JACKGONG.");

    lvnv::g_config.load(lvnv::ConfigPath());
    lvnv::g_config.clampValues();
    lvnv::g_routeReader.setRescanInterval(
        lvnv::g_config.cduPeriodicRescan ? lvnv::g_config.cduRescanIntervalSec : 0.0);
    lvnv::g_routeReader.setScanOnExecOnly(lvnv::g_config.cduScanOnExecOnly);
    // Persist migrated defaults and newly added keys on every start.
    lvnv::SaveConfig();

    lvnv::g_routeReader.bind();
    lvnv::BindRefs();
    lvnv::g_status.aircraftIsFF = lvnv::AircraftLooksLikeFF757();
    lvnv::LogRouteProbe();

    lvnv::SyncPublishedFromConfig();
    lvnv::RegisterPublished();
    lvnv::CreateMenuAndCommands();
    lvnv::CreateStatusWindow();

    lvnv::Logf("FF avionics datarefs: %s", lvnv::g_refs.ffDatarefsFound ? "found" : "missing");
    lvnv::Logf("route datarefs: %s", lvnv::g_routeReader.report().c_str());
    lvnv::Logf("writable: nav_steer=%d fd_pitch_legacy=%d fd_pitch_c2=%d fd_roll_legacy=%d fd_roll_c2=%d override_pitch=%d override_roll=%d override_nav_heading=%d",
               lvnv::g_refs.navSteerWritable ? 1 : 0,
               lvnv::g_refs.fdPitchLegacyWritable ? 1 : 0,
               lvnv::g_refs.fdPitchCockpit2Writable ? 1 : 0,
               lvnv::g_refs.fdRollLegacyWritable ? 1 : 0,
               lvnv::g_refs.fdRollCockpit2Writable ? 1 : 0,
               lvnv::g_refs.overridePitchWritable ? 1 : 0,
               lvnv::g_refs.overrideRollWritable ? 1 : 0,
               lvnv::g_refs.overrideNavHeadingWritable ? 1 : 0);
    lvnv::Logf("aircraft detected as FF757/767: %d", lvnv::g_status.aircraftIsFF ? 1 : 0);

    return 1;
}

PLUGIN_API void XPluginStop(void) {
    lvnv::Logf("stopping");
    if (lvnv::g_flightLoop) {
        XPLMDestroyFlightLoop(lvnv::g_flightLoop);
        lvnv::g_flightLoop = nullptr;
    }
    lvnv::ClearPitchOverride();
    lvnv::ClearRollOverride();
    lvnv::ClearNavHeadingOverride();
    lvnv::SaveConfig();
    lvnv::DestroyMenuAndCommands();
    lvnv::UnregisterPublished();
    if (lvnv::g_statusWindow) {
        XPLMDestroyWindow(lvnv::g_statusWindow);
        lvnv::g_statusWindow = nullptr;
    }
    if (lvnv::g_logFile.is_open()) {
        lvnv::g_logFile.close();
    }
}

PLUGIN_API int XPluginEnable(void) {
    if (lvnv::g_flightLoop) return 1;

    XPLMCreateFlightLoop_t params{};
    params.structSize = static_cast<int>(sizeof(params));
    params.phase = xplm_FlightLoop_Phase_BeforeFlightModel;
    params.callbackFunc = lvnv::FlightLoopCallback;
    params.refcon = nullptr;

    lvnv::g_flightLoop = XPLMCreateFlightLoop(&params);
    if (!lvnv::g_flightLoop) {
        lvnv::Logf("could not create flight loop");
        return 0;
    }
    XPLMScheduleFlightLoop(lvnv::g_flightLoop, -1.0f, 1);
    lvnv::Logf("flight loop enabled");
    return 1;
}

PLUGIN_API void XPluginDisable(void) {
    if (lvnv::g_flightLoop) {
        XPLMDestroyFlightLoop(lvnv::g_flightLoop);
        lvnv::g_flightLoop = nullptr;
    }
    lvnv::ClearPitchOverride();
    lvnv::ClearRollOverride();
    lvnv::ClearNavHeadingOverride();
    lvnv::SetWindowVisible(false);
    lvnv::Logf("flight loop disabled");
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int message, void*) {
    switch (message) {
        case XPLM_MSG_PLANE_LOADED:
            lvnv::ResetRuntimeState();
            lvnv::BindRefs();
            lvnv::g_routeReader.bind();
            lvnv::g_routeReader.reset();
            lvnv::g_status.aircraftIsFF = lvnv::AircraftLooksLikeFF757();
            lvnv::Logf("plane loaded; aircraft detected=%d", lvnv::g_status.aircraftIsFF ? 1 : 0);
            lvnv::LogRouteProbe();
            break;
        case XPLM_MSG_PLANE_UNLOADED:
            lvnv::ClearPitchOverride();
            lvnv::ClearRollOverride();
            lvnv::ResetRuntimeState();
            break;
        case XPLM_MSG_AIRPORT_LOADED:
            lvnv::g_routeReader.reset();
            lvnv::g_guidance.reset();
            break;
        default:
            break;
    }
}

} // namespace lvnv
