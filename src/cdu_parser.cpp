#include "cdu_parser.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace lvnv {

namespace {

bool IsSpace(char c) {
    return std::isspace(static_cast<unsigned char>(c)) != 0;
}

std::string Upper(const std::string& value) {
    std::string out = value;
    for (char& c : out) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return out;
}

bool StartsWith(const std::string& value, const char* prefix) {
    const size_t n = std::strlen(prefix);
    return value.size() >= n && value.compare(0, n, prefix) == 0;
}

std::string FirstToken(const std::string& value) {
    size_t i = 0;
    while (i < value.size() && IsSpace(value[i])) ++i;
    size_t j = i;
    while (j < value.size() && !IsSpace(value[j])) ++j;
    return value.substr(i, j - i);
}

// Extracts the first integer in a string.  Returns -1 when none is present.
int FirstInteger(const std::string& value, size_t start = 0) {
    size_t i = start;
    while (i < value.size() && !std::isdigit(static_cast<unsigned char>(value[i]))) ++i;
    if (i >= value.size()) return -1;
    int result = 0;
    while (i < value.size() && std::isdigit(static_cast<unsigned char>(value[i]))) {
        result = result * 10 + (value[i] - '0');
        ++i;
    }
    return result;
}

// Parses the distance printed immediately before "NM" (e.g. "  82NM").
double ParseNm(const std::string& value) {
    std::string up = Upper(value);
    const size_t nm = up.find("NM");
    if (nm == std::string::npos) return 0.0;
    size_t i = nm;
    while (i > 0 && IsSpace(value[i - 1])) --i;
    size_t end = i;
    while (i > 0 && (std::isdigit(static_cast<unsigned char>(value[i - 1])) ||
                     value[i - 1] == '.')) {
        --i;
    }
    if (i >= end) return 0.0;
    return std::strtod(value.substr(i, end - i).c_str(), nullptr);
}

// The LEGS page course line starts with a three-digit magnetic course
// (" 344     16NM").  Returns 0 when the line has no valid course, so the
// caller falls back to the active desired track.
int ParseCourseDeg(const std::string& value) {
    size_t i = 0;
    while (i < value.size() && IsSpace(value[i])) ++i;
    int result = 0;
    int digits = 0;
    while (i < value.size() && digits < 3 &&
           std::isdigit(static_cast<unsigned char>(value[i]))) {
        result = result * 10 + (value[i] - '0');
        ++i;
        ++digits;
    }
    if (digits != 3) return 0;
    if (result < 1 || result > 360) return 0;
    return result;
}

bool HasDashField(const std::string& value) {
    return value.find("---") != std::string::npos;
}

constexpr size_t kMaxCduLegs = 160;

// Qualifier printed after the altitude digits: "FL157A" = at or above,
// "6890B" = at or below, plain "11765" = at the fix.
AltConstraint ParseAltQualifier(const std::string& text) {
    std::string up = Upper(TrimCdu(text));
    if (up.find("ABOVE") != std::string::npos) return AltAtOrAbove;
    if (up.find("BELOW") != std::string::npos) return AltAtOrBelow;

    size_t lastDigit = std::string::npos;
    for (size_t i = 0; i < up.size(); ++i) {
        if (std::isdigit(static_cast<unsigned char>(up[i]))) lastDigit = i;
    }
    if (lastDigit == std::string::npos) return AltAt;

    size_t j = lastDigit + 1;
    while (j < up.size() && IsSpace(up[j])) ++j;
    if (j < up.size()) {
        if (up[j] == 'A') return AltAtOrAbove;
        if (up[j] == 'B') return AltAtOrBelow;
    }
    return AltAt;
}

// Parses a CDU speed/altitude field.  The FF757 LEGS page prints SPEED first
// and ALTITUDE second; live example: "EKIMU  .806/FL315".  The same layout is
// used for hard constraints ("250/FL250").  A tolerant swap handles the rare
// reverse presentation, where the right half is clearly a speed.
void ParseSpeedAltField(const std::string& text, int& altFt, int& speedKt,
                        AltConstraint& altType) {
    altFt = 0;
    speedKt = 0;
    altType = AltAt;
    const size_t slash = text.find('/');
    if (slash == std::string::npos) return;

    const std::string left = TrimCdu(text.substr(0, slash));
    const std::string right = TrimCdu(text.substr(slash + 1));
    if (left.empty() || right.empty()) return;

    int alt = HasDashField(right) ? 0 : ParseCduAltitude(right);
    int spd = HasDashField(left) ? 0 : ParseCduSpeed(left);
    const std::string* altText = &right;

    if (alt <= 0) {
        const int altLeft = HasDashField(left) ? 0 : ParseCduAltitude(left);
        const int spdRight = HasDashField(right) ? 0 : ParseCduSpeed(right);
        if (altLeft > 0 && (spdRight > 0 || HasDashField(right))) {
            alt = altLeft;
            spd = spdRight;
            altText = &left;
        }
    }

    if (alt > 0 && alt <= 100000) {
        altFt = alt;
        altType = ParseAltQualifier(*altText);
    }
    if (spd > 0 && spd <= 500) speedKt = spd;
}

} // namespace

std::string TrimCdu(const std::string& value) {
    size_t begin = 0;
    size_t end = value.size();
    while (begin < end && IsSpace(value[begin])) ++begin;
    while (end > begin && IsSpace(value[end - 1])) --end;
    return value.substr(begin, end - begin);
}

bool IsCduLegsPage(const std::vector<std::string>& rows) {
    if (rows.empty()) return false;
    return Upper(rows[0]).find("LEGS") != std::string::npos;
}

int ParseCduAltitude(const std::string& text) {
    std::string value = TrimCdu(text);
    while (!value.empty() && (value[0] == '*' || value[0] == '>' || value[0] == '<')) {
        value.erase(value.begin());
    }
    if (value.empty()) return 0;

    std::string up = Upper(value);
    if (StartsWith(up, "FL")) {
        const int fl = FirstInteger(value, 2);
        if (fl > 0 && fl <= 700) return fl * 100;
        return 0;
    }

    // Stop at the first non-numeric character so "325/210" and "325FT" work.
    int number = 0;
    size_t i = 0;
    while (i < value.size() && std::isdigit(static_cast<unsigned char>(value[i]))) {
        number = number * 10 + (value[i] - '0');
        ++i;
    }
    if (i == 0) return 0;
    if (number <= 0) return 0;
    if (number <= 700) return number * 100;  // CDU shows FL325 as "325"
    if (number <= 100000) return number;
    return 0;
}

int ParseCduSpeed(const std::string& text) {
    std::string value = TrimCdu(text);
    if (value.empty()) return 0;
    const int number = FirstInteger(value);
    if (number <= 0 || number > 500) return 0;
    return number;
}

CduPage ParseCduLegsPage(const std::vector<std::string>& rows, int pageHint) {
    CduPage page;
    if (rows.empty()) return page;

    page.title = TrimCdu(rows[0]);
    const std::string upTitle = Upper(page.title);
    if (upTitle.find("LEGS") == std::string::npos) return page;

    page.valid = true;
    const size_t legsPos = upTitle.find("LEGS");
    page.routeName = TrimCdu(page.title.substr(0, legsPos));
    page.activeRoute = upTitle.find("MOD") == std::string::npos &&
                       (upTitle.find("ACT") != std::string::npos ||
                        page.routeName.find("RTE") != std::string::npos);

    // Page number follows LEGS as "1/2".
    std::string tail = page.title.substr(legsPos + 4);
    int first = -1;
    int second = -1;
    size_t i = 0;
    while (i < tail.size() && !std::isdigit(static_cast<unsigned char>(tail[i]))) ++i;
    if (i < tail.size()) {
        first = 0;
        while (i < tail.size() && std::isdigit(static_cast<unsigned char>(tail[i]))) {
            first = first * 10 + (tail[i] - '0');
            ++i;
        }
        if (i < tail.size() && tail[i] == '/') {
            ++i;
            second = 0;
            while (i < tail.size() && std::isdigit(static_cast<unsigned char>(tail[i]))) {
                second = second * 10 + (tail[i] - '0');
                ++i;
            }
        }
    }
    page.page = first > 0 ? first : pageHint;
    page.pageCount = second > 0 ? second : 1;

    const int lineCount = static_cast<int>(rows.size());
    for (int r = 1; r + 1 < lineCount; ++r) {
        std::string top = TrimCdu(rows[static_cast<size_t>(r)]);
        std::string bottom = TrimCdu(rows[static_cast<size_t>(r + 1)]);

        // Footer/page-link rows end the leg list.  Do not treat the constraint
        // field "---/-----" on a normal waypoint line as a footer.
        if (top.find("-----") != std::string::npos ||
            bottom.find("<RTE") != std::string::npos ||
            bottom.find("<LEGS") != std::string::npos ||
            bottom.find("RTE DATA") != std::string::npos) {
            break;
        }
        if (bottom.empty() && top.empty()) continue;

        std::string upBottom = Upper(bottom);
        if (upBottom.find("END OF ROUTE") != std::string::npos ||
            upBottom.find("NO ACTIVE ROUTE") != std::string::npos) {
            break;
        }
        if (bottom.find("---") != std::string::npos && bottom.size() > 3 &&
            bottom.find('/') == std::string::npos) {
            // A stand-alone discontinuity line.
            ++r;
            continue;
        }
        if (upBottom.find("DISCONTINUITY") != std::string::npos ||
            upBottom.find("ROUTE DISCONTINUITY") != std::string::npos) {
            ++r;
            continue;
        }

        // The FF757 prints a hold with the direction on the course line:
        //   HOLD LEFT
        //   WX205        270/FL200
        // (some pages add a standalone "HOLD" continuation line; both forms
        // must mark the WX205 leg as a hold and never become extra waypoints).
        const std::string upTop = Upper(TrimCdu(top));
        if (upTop == "HOLD") {
            std::string holdIdent = FirstToken(TrimCdu(bottom));
            while (!holdIdent.empty() &&
                   (holdIdent[0] == '*' || holdIdent[0] == '>' || holdIdent[0] == '<')) {
                holdIdent.erase(holdIdent.begin());
            }
            if (!holdIdent.empty()) {
                if (!page.legs.empty() && page.legs.back().id == holdIdent) {
                    page.legs.back().hold = true;
                } else {
                    CduLeg hleg;
                    hleg.id = holdIdent;
                    hleg.hold = true;
                    hleg.page = page.page;
                    std::string restH = bottom;
                    const size_t idPos = restH.find(holdIdent);
                    if (idPos != std::string::npos) {
                        restH = restH.substr(idPos + holdIdent.size());
                    }
                    int hAlt = 0;
                    int hSpd = 0;
                    AltConstraint hType = AltAt;
                    ParseSpeedAltField(restH, hAlt, hSpd, hType);
                    if (hAlt > 0) {
                        hleg.altFt = hAlt;
                        hleg.altConstraint = hType;
                    }
                    if (hSpd > 0) hleg.estSpeedKt = hSpd;
                    page.legs.push_back(hleg);
                }
            }
            ++r;
            continue;
        }
        const bool holdFromTop = upTop.find("HOLD") != std::string::npos;

        std::string ident = FirstToken(bottom);
        while (!ident.empty() &&
               (ident[0] == '*' || ident[0] == '>' || ident[0] == '<')) {
            ident.erase(ident.begin());
        }
        std::string upIdent = Upper(ident);
        if (ident.empty() || ident.find("---") != std::string::npos) {
            ++r;
            continue;
        }
        if (upIdent == "TOC" || upIdent == "TOD" || upIdent == "[TOC]" ||
            upIdent == "[TOD]") {
            ++r;
            continue;
        }

        CduLeg leg;
        leg.id = ident;
        leg.page = page.page;

        // The top line carries COURSE and DISTANCE, never the altitude.  The
        // altitude estimate is on the waypoint line itself (".806/FL315").
        leg.legCourseDeg = ParseCourseDeg(top);
        leg.legDistanceNm = ParseNm(top);

        std::string rest = bottom;
        const size_t identPos = rest.find(ident);
        if (identPos != std::string::npos) {
            rest = rest.substr(identPos + ident.size());
        }

        int estAlt = 0;
        int estSpeed = 0;
        AltConstraint estType = AltAt;
        ParseSpeedAltField(rest, estAlt, estSpeed, estType);
        if (estAlt > 0) {
            leg.altFt = estAlt;
            leg.altConstraint = estType;
        }
        if (estSpeed > 0) leg.estSpeedKt = estSpeed;

        // A hard constraint, when present, is printed after "NM" on the
        // course line.  It takes precedence over the estimate.
        const size_t nmPos = Upper(top).find("NM");
        if (nmPos != std::string::npos && nmPos + 2 < top.size()) {
            int conAlt = 0;
            int conSpeed = 0;
            AltConstraint conType = AltAt;
            ParseSpeedAltField(top.substr(nmPos + 2), conAlt, conSpeed, conType);
            if (conAlt > 0) {
                leg.constraintAltFt = conAlt;
                leg.altFt = conAlt;
                leg.altConstraint = conType;
            }
            if (conSpeed > 0) leg.constraintSpeedKt = conSpeed;
        }

        if (upIdent.find("(HLD)") != std::string::npos ||
            upBottom.find("HOLD") != std::string::npos ||
            upBottom.find(" LEFT") != std::string::npos ||
            upBottom.find(" RIGHT") != std::string::npos ||
            holdFromTop) {
            leg.hold = true;
        }

        page.legs.push_back(leg);
        ++r; // consume the waypoint line as well
    }

    return page;
}

void AppendCduPage(CduRoute& route, const CduPage& page) {
    if (!page.valid) return;
    if (page.activeRoute) {
        if (!route.valid || route.routeName.empty()) {
            route.routeName = page.routeName;
        }
        route.activeRoute = true;
    }
    route.pageCount = std::max(route.pageCount, page.pageCount);
    for (const CduLeg& leg : page.legs) {
        if (route.legs.size() >= kMaxCduLegs) break;
        // Pages should not overlap, but be tolerant of a repeated join leg.
        if (!route.legs.empty() && route.legs.back().id == leg.id &&
            route.legs.back().page == leg.page) {
            continue;
        }
        route.legs.push_back(leg);
    }
    route.valid = route.activeRoute && !route.legs.empty();
}

} // namespace lvnv
