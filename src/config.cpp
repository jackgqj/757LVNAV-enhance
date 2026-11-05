#include "config.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace lvnv {

namespace {

std::string trim(const std::string& s) {
    size_t a = 0;
    while (a < s.size() && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    size_t b = s.size();
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

bool parseBool(const std::string& v, bool& out) {
    std::string s = lower(trim(v));
    if (s == "1" || s == "true" || s == "yes" || s == "on") { out = true; return true; }
    if (s == "0" || s == "false" || s == "no" || s == "off") { out = false; return true; }
    return false;
}

bool parseDouble(const std::string& v, double& out) {
    try {
        size_t used = 0;
        double d = std::stod(trim(v), &used);
        if (used == 0 || std::isnan(d)) return false;
        out = d;
        return true;
    } catch (...) {
        return false;
    }
}

bool parseInt(const std::string& v, int& out) {
    try {
        size_t used = 0;
        int i = static_cast<int>(std::stol(trim(v), &used));
        if (used == 0) return false;
        out = i;
        return true;
    } catch (...) {
        return false;
    }
}

} // namespace

void Config::clampValues() {
    lnavGain = std::clamp(lnavGain, 0.0, 2.0);
    lnavMaxInterceptDeg = std::clamp(lnavMaxInterceptDeg, 5.0, 45.0);
    lnavTurnLeadFactor = std::clamp(lnavTurnLeadFactor, 0.5, 3.0);
    lnavMaxBankDeg = std::clamp(lnavMaxBankDeg, 10.0, 35.0);
    lnavMaxCourseErrorDeg = std::clamp(lnavMaxCourseErrorDeg, 20.0, 90.0);
    lnavInterceptDistanceNm = std::clamp(lnavInterceptDistanceNm, 0.1, 5.0);
    lnavMinSpeedKts = std::clamp(lnavMinSpeedKts, 30.0, 150.0);
    lnavMinDistanceNm = std::clamp(lnavMinDistanceNm, 0.0, 1.0);
    lnavRollGain = std::clamp(lnavRollGain, 0.0, 1.0);
    lnavMaxRollCorrectionDeg = std::clamp(lnavMaxRollCorrectionDeg, 0.0, 15.0);
    lnavRollRateLimitDegPerSec = std::clamp(lnavRollRateLimitDegPerSec, 0.2, 15.0);
    lnavRollSmoothingTauSec = std::clamp(lnavRollSmoothingTauSec, 0.02, 1.0);

    vnavVsGain = std::clamp(vnavVsGain, 0.0, 2.0);
    vnavAccelGain = std::clamp(vnavAccelGain, 0.0, 0.02);
    vnavMaxCorrectionDeg = std::clamp(vnavMaxCorrectionDeg, 0.0, 8.0);
    vnavRateLimitDegPerSec = std::clamp(vnavRateLimitDegPerSec, 0.2, 10.0);
    vnavSmoothingTauSec = std::clamp(vnavSmoothingTauSec, 0.02, 1.0);
    vnavMaxVsFpm = std::clamp(vnavMaxVsFpm, 200.0, 6000.0);
    vnavMinDistanceNm = std::clamp(vnavMinDistanceNm, 0.0, 2.0);
    vnavMaxAltDeltaFt = std::clamp(vnavMaxAltDeltaFt, 1000.0, 60000.0);
    cduRescanIntervalSec = std::clamp(cduRescanIntervalSec, 0.0, 600.0);

    lnavRollMode = std::clamp(lnavRollMode, -1, 12);
    vnavPitchModeA = std::clamp(vnavPitchModeA, -1, 12);
    vnavPitchModeB = std::clamp(vnavPitchModeB, -1, 12);
    vnavPitchModeC = std::clamp(vnavPitchModeC, -1, 12);
    apCmdMode = std::clamp(apCmdMode, 0, 5);
}

bool Config::load(const std::string& path) {
    std::ifstream in(path);
    if (!in.is_open()) {
        clampValues();
        return false;
    }

    std::string line;
    while (std::getline(in, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#' || line[0] == ';') continue;
        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string key = lower(trim(line.substr(0, eq)));
        std::string value = trim(line.substr(eq + 1));

        bool b = false;
        double d = 0.0;
        int i = 0;

        if (key == "enabled") { if (parseBool(value, b)) enabled = b; }
        else if (key == "enable_lnav") { if (parseBool(value, b)) enableLnav = b; }
        else if (key == "enable_vnav") { if (parseBool(value, b)) enableVnav = b; }
        else if (key == "enable_vnav_smoothing") { if (parseBool(value, b)) enableVnavSmoothing = b; }
        else if (key == "enable_lnav_fd_roll") { if (parseBool(value, b)) enableLnavFdRoll = b; }
        else if (key == "lnav_gain") { if (parseDouble(value, d)) lnavGain = d; }
        else if (key == "lnav_max_intercept_deg") { if (parseDouble(value, d)) lnavMaxInterceptDeg = d; }
        else if (key == "lnav_turn_lead_factor") { if (parseDouble(value, d)) lnavTurnLeadFactor = d; }
        else if (key == "lnav_max_bank_deg") { if (parseDouble(value, d)) lnavMaxBankDeg = d; }
        else if (key == "lnav_max_course_error_deg") { if (parseDouble(value, d)) lnavMaxCourseErrorDeg = d; }
        else if (key == "lnav_intercept_distance_nm") { if (parseDouble(value, d)) lnavInterceptDistanceNm = d; }
        else if (key == "lnav_min_speed_kts") { if (parseDouble(value, d)) lnavMinSpeedKts = d; }
        else if (key == "lnav_min_distance_nm") { if (parseDouble(value, d)) lnavMinDistanceNm = d; }
        else if (key == "lnav_roll_gain") { if (parseDouble(value, d)) lnavRollGain = d; }
        else if (key == "lnav_max_roll_correction_deg") { if (parseDouble(value, d)) lnavMaxRollCorrectionDeg = d; }
        else if (key == "lnav_roll_rate_limit_deg_s") { if (parseDouble(value, d)) lnavRollRateLimitDegPerSec = d; }
        else if (key == "lnav_roll_smoothing_tau_s") { if (parseDouble(value, d)) lnavRollSmoothingTauSec = d; }
        else if (key == "vnav_vs_gain") { if (parseDouble(value, d)) vnavVsGain = d; }
        else if (key == "vnav_accel_gain") { if (parseDouble(value, d)) vnavAccelGain = d; }
        else if (key == "vnav_max_correction_deg") { if (parseDouble(value, d)) vnavMaxCorrectionDeg = d; }
        else if (key == "vnav_rate_limit_deg_s") { if (parseDouble(value, d)) vnavRateLimitDegPerSec = d; }
        else if (key == "vnav_smoothing_tau_s") { if (parseDouble(value, d)) vnavSmoothingTauSec = d; }
        else if (key == "vnav_max_vs_fpm") { if (parseDouble(value, d)) vnavMaxVsFpm = d; }
        else if (key == "vnav_min_distance_nm") { if (parseDouble(value, d)) vnavMinDistanceNm = d; }
        else if (key == "vnav_max_alt_delta_ft") { if (parseDouble(value, d)) vnavMaxAltDeltaFt = d; }
        else if (key == "vnav_augment_speed_mode") { if (parseBool(value, b)) vnavAugmentSpeedMode = b; }
        else if (key == "vnav_respect_selected_alt") { if (parseBool(value, b)) vnavRespectSelectedAlt = b; }
        else if (key == "cdu_rescan_interval_s") { if (parseDouble(value, d)) cduRescanIntervalSec = d; }
        else if (key == "cdu_periodic_rescan") { if (parseBool(value, b)) cduPeriodicRescan = b; }
        else if (key == "cdu_scan_on_exec_only") { if (parseBool(value, b)) cduScanOnExecOnly = b; }
        else if (key == "lnav_roll_mode") { if (parseInt(value, i)) lnavRollMode = i; }
        else if (key == "vnav_pitch_mode_a") { if (parseInt(value, i)) vnavPitchModeA = i; }
        else if (key == "vnav_pitch_mode_b") { if (parseInt(value, i)) vnavPitchModeB = i; }
        else if (key == "vnav_pitch_mode_c") { if (parseInt(value, i)) vnavPitchModeC = i; }
        else if (key == "ap_cmd_mode") { if (parseInt(value, i)) apCmdMode = i; }
        else if (key == "lnav_wide_mode") { if (parseBool(value, b)) lnavWideMode = b; }
        else if (key == "force_lnav") { if (parseBool(value, b)) forceLnav = b; }
        else if (key == "force_vnav") { if (parseBool(value, b)) forceVnav = b; }
        else if (key == "debug") { if (parseBool(value, b)) debug = b; }
    }

    // Migrate the untouched 1.0.0 default mode block.  These four values were
    // the original guessed defaults; a live FF757 build showed CMD=1,
    // LNAV=2, VNAV pitch modes 0/5/6 (early climb / SPD / PTH).  Only migrate
    // the exact legacy combination so a deliberate user setting is preserved.
    if (lnavRollMode == 3 && vnavPitchModeA == 4 &&
        vnavPitchModeB == 5 && apCmdMode == 2) {
        lnavRollMode = 2;
        vnavPitchModeA = 0;
        vnavPitchModeB = 6;
        vnavPitchModeC = 5;
        apCmdMode = 1;
    }

    // Migrate the legacy 1.1.0 default turn lead (1.35).  Starting the turn
    // 35 % early cut the corner and forced a longer bank on roll-out, which
    // showed up as an overshoot.  Only the exact legacy value is migrated.
    if (std::abs(lnavTurnLeadFactor - 1.35) < 1e-6) lnavTurnLeadFactor = 1.0;

    clampValues();
    return true;
}

bool Config::save(const std::string& path) const {
    std::ofstream out(path, std::ios::trunc);
    if (!out.is_open()) return false;

    out << "# 757 LNAV/VNAV Plus configuration\n";
    out << "# Delete this file to restore defaults.\n\n";
    out << "enabled=" << (enabled ? 1 : 0) << "\n";
    out << "enable_lnav=" << (enableLnav ? 1 : 0) << "\n";
    out << "enable_vnav=" << (enableVnav ? 1 : 0) << "\n";
    out << "enable_vnav_smoothing=" << (enableVnavSmoothing ? 1 : 0) << "\n";
    out << "enable_lnav_fd_roll=" << (enableLnavFdRoll ? 1 : 0) << "\n\n";

    out << "lnav_gain=" << lnavGain << "\n";
    out << "lnav_max_intercept_deg=" << lnavMaxInterceptDeg << "\n";
    out << "lnav_turn_lead_factor=" << lnavTurnLeadFactor << "\n";
    out << "lnav_max_bank_deg=" << lnavMaxBankDeg << "\n";
    out << "lnav_max_course_error_deg=" << lnavMaxCourseErrorDeg << "\n";
    out << "lnav_intercept_distance_nm=" << lnavInterceptDistanceNm << "\n";
    out << "lnav_min_speed_kts=" << lnavMinSpeedKts << "\n";
    out << "lnav_min_distance_nm=" << lnavMinDistanceNm << "\n";
    out << "lnav_roll_gain=" << lnavRollGain << "\n";
    out << "lnav_max_roll_correction_deg=" << lnavMaxRollCorrectionDeg << "\n";
    out << "lnav_roll_rate_limit_deg_s=" << lnavRollRateLimitDegPerSec << "\n";
    out << "lnav_roll_smoothing_tau_s=" << lnavRollSmoothingTauSec << "\n\n";

    out << "vnav_vs_gain=" << vnavVsGain << "\n";
    out << "vnav_accel_gain=" << vnavAccelGain << "\n";
    out << "vnav_max_correction_deg=" << vnavMaxCorrectionDeg << "\n";
    out << "vnav_rate_limit_deg_s=" << vnavRateLimitDegPerSec << "\n";
    out << "vnav_smoothing_tau_s=" << vnavSmoothingTauSec << "\n";
    out << "vnav_max_vs_fpm=" << vnavMaxVsFpm << "\n";
    out << "vnav_min_distance_nm=" << vnavMinDistanceNm << "\n";
    out << "vnav_max_alt_delta_ft=" << vnavMaxAltDeltaFt << "\n";
    out << "vnav_augment_speed_mode=" << (vnavAugmentSpeedMode ? 1 : 0) << "\n";
    out << "vnav_respect_selected_alt=" << (vnavRespectSelectedAlt ? 1 : 0) << "\n\n";

    out << "cdu_rescan_interval_s=" << cduRescanIntervalSec << "\n";
    out << "cdu_periodic_rescan=" << (cduPeriodicRescan ? 1 : 0) << "\n";
    out << "cdu_scan_on_exec_only=" << (cduScanOnExecOnly ? 1 : 0) << "\n\n";

    out << "# Undocumented FF757 mode enum values.  Change only if the\n";
    out << "# status window shows that a different value is used on your build.\n";
    out << "lnav_roll_mode=" << lnavRollMode << "\n";
    out << "vnav_pitch_mode_a=" << vnavPitchModeA << "\n";
    out << "vnav_pitch_mode_b=" << vnavPitchModeB << "\n";
    out << "vnav_pitch_mode_c=" << vnavPitchModeC << "\n";
    out << "ap_cmd_mode=" << apCmdMode << "\n";
    out << "lnav_wide_mode=" << (lnavWideMode ? 1 : 0) << "\n";
    out << "force_lnav=" << (forceLnav ? 1 : 0) << "\n";
    out << "force_vnav=" << (forceVnav ? 1 : 0) << "\n";
    out << "debug=" << (debug ? 1 : 0) << "\n";

    return out.good();
}

} // namespace lvnv
