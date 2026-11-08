#include "cdu_parser.h"

#include <iostream>
#include <string>
#include <vector>

namespace {

int g_checks = 0;
int g_failures = 0;

void Check(bool condition, const std::string& what) {
    ++g_checks;
    if (!condition) {
        ++g_failures;
        std::cout << "FAIL: " << what << "\n";
    }
}

std::vector<std::string> Rows(std::vector<std::string> lines, int width = 24, int count = 14) {
    lines.resize(static_cast<size_t>(count), std::string(static_cast<size_t>(width), ' '));
    for (std::string& line : lines) {
        if (static_cast<int>(line.size()) < width) {
            line.append(static_cast<size_t>(width) - line.size(), ' ');
        } else if (static_cast<int>(line.size()) > width) {
            line.resize(static_cast<size_t>(width));
        }
    }
    return lines;
}

void TestLegsPage() {
    // Live FF757 format: the top line is COURSE + DISTANCE, the waypoint line
    // carries the estimated speed/altitude after the ident.
    std::vector<std::string> lines = {
        "ACT RTE 1 LEGS       1/2",
        " 336     14NM           ",
        "EKIMU        .806/FL315 ",
        " 010      6NM           ",
        "POMOK        .806/FL329 ",
        " 342     33NM           ",
        "SS320        .805/FL197A",
        "",
        "",
        "",
        "",
        "------------------------",
        "<RTE 2 LEGS    RTE DATA>",
        "",
    };
    const std::vector<std::string> rows = Rows(lines);
    const lvnv::CduPage page = lvnv::ParseCduLegsPage(rows);

    Check(page.valid, "LEGS page is valid");
    Check(page.activeRoute, "ACT RTE is active");
    Check(page.page == 1, "page number is 1");
    Check(page.pageCount == 2, "page count is 2");
    Check(page.routeName == "ACT RTE 1", "route name is ACT RTE 1");
    Check(page.legs.size() == 3, "three legs parsed");

    if (page.legs.size() == 3) {
        Check(page.legs[0].id == "EKIMU", "leg 0 ident EKIMU");
        Check(page.legs[1].id == "POMOK", "leg 1 ident POMOK");
        Check(page.legs[2].id == "SS320", "leg 2 ident SS320");
        Check(page.legs[0].altFt == 31500, "leg 0 altitude FL315");
        Check(page.legs[1].altFt == 32900, "leg 1 altitude FL329");
        Check(page.legs[2].altFt == 19700, "leg 2 altitude FL197A");
        Check(page.legs[2].altConstraint == lvnv::AltAtOrAbove,
              "FL197A is an at-or-above constraint");
        // Regression: the course on the line above must never be read as an
        // altitude (336 -> 33600, 010 -> 1000 was the old behaviour).
        Check(page.legs[0].altFt != 33600, "course 336 is not FL336");
        Check(page.legs[1].altFt != 1000, "course 010 is not 1000 ft");
        Check(page.legs[0].legDistanceNm > 13.9 && page.legs[0].legDistanceNm < 14.1,
              "leg 0 distance 14 NM");
        Check(page.legs[1].legDistanceNm > 5.9 && page.legs[1].legDistanceNm < 6.1,
              "leg 1 distance 6 NM");
        Check(page.legs[2].legDistanceNm > 32.9 && page.legs[2].legDistanceNm < 33.1,
              "leg 2 distance 33 NM");
        Check(page.legs[0].legCourseDeg == 336, "leg 0 course 336");
        Check(page.legs[1].legCourseDeg == 10, "leg 1 course 010");
        Check(page.legs[2].legCourseDeg == 342, "leg 2 course 342");
        Check(page.legs[0].constraintAltFt == 0, "leg 0 has no hard constraint");
    }
}

void TestConstraints() {
    std::vector<std::string> lines = {
        "ACT RTE 1 LEGS       1/1",
        " 250     14NM           ",
        "WPT1           250/FL250 ",
        " 090      8NM           ",
        "WPT2           ---/FL180 ",
        " 300     11NM           ",
        "WPT3           ---/----- ",
        " 299     22NM           ",
        "WPT4           299/6890B",
        "",
        "",
        "",
        "------------------------",
        "<LEGS                  >",
        "",
    };
    const std::vector<std::string> rows = Rows(lines);
    const lvnv::CduPage page = lvnv::ParseCduLegsPage(rows);
    Check(page.valid, "constraint page valid");
    Check(page.legs.size() == 4, "constraint page four legs");
    if (page.legs.size() == 4) {
        Check(page.legs[0].altFt == 25000, "speed/alt field FL250");
        Check(page.legs[0].altConstraint == lvnv::AltAt,
              "no suffix is an exact crossing");
        Check(page.legs[0].constraintAltFt == 0,
              "waypoint-line value is an estimate, not a hard constraint");
        Check(page.legs[1].altFt == 18000, "FL180 estimate");
        Check(page.legs[2].altFt == 0, "dash altitude is unavailable");
        Check(page.legs[3].altFt == 6890, "6890B altitude parsed");
        Check(page.legs[3].altConstraint == lvnv::AltAtOrBelow,
              "6890B is an at-or-below constraint");
    }
}

void TestModRoute() {
    std::vector<std::string> lines = {
        "MOD RTE 1 LEGS       1/1",
        " 250     10NM           ",
        "WPT1           ---/----- ",
        "",
    };
    const lvnv::CduPage page = lvnv::ParseCduLegsPage(Rows(lines));
    Check(page.valid, "MOD LEGS page is a legs page");
    Check(!page.activeRoute, "MOD RTE is not active");
}

void TestMerge() {
    lvnv::CduRoute route;
    lvnv::CduPage page1 = lvnv::ParseCduLegsPage(Rows({
        "ACT RTE 1 LEGS       1/2",
        " 325     82NM           ",
        "PIKAS         ---/----- ",
    }));
    lvnv::CduPage page2 = lvnv::ParseCduLegsPage(Rows({
        "ACT RTE 1 LEGS       2/2",
        " 250     18NM           ",
        "WPT4           ---/----- ",
    }));
    lvnv::AppendCduPage(route, page1);
    lvnv::AppendCduPage(route, page2);
    Check(route.valid, "merged route valid");
    Check(route.legs.size() == 2, "merged route has two legs");
    Check(route.pageCount == 2, "merged page count is 2");
}

void TestAltitudeParsing() {
    Check(lvnv::ParseCduAltitude("FL325") == 32500, "FL325 -> 32500");
    Check(lvnv::ParseCduAltitude("325") == 32500, "325 -> 32500");
    Check(lvnv::ParseCduAltitude("35000") == 35000, "35000 -> 35000");
    Check(lvnv::ParseCduAltitude("2500") == 2500, "2500 -> 2500");
    Check(lvnv::ParseCduAltitude("---") == 0, "--- -> 0");
    Check(lvnv::ParseCduSpeed("210") == 210, "speed 210");
}

} // namespace

int main() {
    TestLegsPage();
    TestConstraints();
    TestModRoute();
    TestMerge();
    TestAltitudeParsing();
    std::cout << g_checks << " checks, " << g_failures << " failures\n";
    return g_failures == 0 ? 0 : 1;
}
