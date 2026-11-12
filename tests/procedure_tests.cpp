#include "procedure_parser.h"

#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

using namespace lvnv;

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond, msg) do { \
    ++g_checks; \
    if (!(cond)) { \
        ++g_failures; \
        std::printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); \
    } \
} while (0)

#define CHECK_NEAR(a, b, eps, msg) CHECK(std::abs((a) - (b)) <= (eps), msg)

static std::vector<std::string> CifpLines() {
    return {
        R"(SID:010,5,DEP10L,RW10L,RW28R,ZU,P,G,G   , ,302,IF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,24600, ,   ,    ,   , , , , , , ,F,D;)",
        R"(SID:020,5,DEP10L,RW10L,LS804,ZU,P,C,E   , ,302,TF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,     , ,   ,    ,   , , , , , , ,F,D;)",
        R"(SID:030,5,DEP10L,RW10L,LS508,ZU,P,C,E   ,L,302,RF, , , , , ,003300,    ,    ,    ,0016, ,     ,     ,     , ,   ,    ,   ,LS042,ZU,P,C, , ,F,D;)",
        R"(SID:040,5,DEP10L,RW10L,LS512,ZU,P,C,E   ,R,302,RF, , , , , ,004600,    ,    ,    ,0037, ,     ,     ,     ,-,220,    ,   ,LS002,ZU,P,C, , ,F,D;)",
        R"(SID:050,5,DEP10L,RW10L,LS516,ZU,P,C,E   ,L,302,RF, , , , , ,020300,    ,    ,    ,0044, ,     ,     ,     , ,   ,    ,   ,LS003,ZU,P,C, , ,F,D;)",
    };
}

static CifpProcedureDatabase::FixResolver FakeResolver(bool withCenters) {
    return [withCenters](const std::string& ident, double, double, double& lat,
                         double& lon) {
        static const std::map<std::string, std::pair<double, double>> kFixes = {
            {"LS804", {29.302833, 91.005500}},
            {"LS508", {29.309611, 91.034472}},
            {"LS512", {29.315778, 91.102639}},
            {"LS516", {29.301917, 91.184556}},
            {"LS042", {29.357389, 91.004417}},
            {"LS002", {29.335000, 91.090000}},
            {"LS003", {29.320000, 91.170000}},
        };
        const auto it = kFixes.find(ident);
        if (it == kFixes.end()) return false;
        if (!withCenters && ident.size() > 2 && ident[0] == 'L' &&
            ident[1] == 'S' && ident.size() == 5) {
            // Disable only the three centre idents, not the leg fixes.
            if (ident == "LS042" || ident == "LS002" || ident == "LS003") {
                return false;
            }
        }
        lat = it->second.first;
        lon = it->second.second;
        return true;
    };
}

int main() {
    CHECK(CifpProcedureDatabase::NormalizeTransition("RW10L") == "10L",
          "CIFP runway transition RW10L normalizes to 10L");
    CHECK(CifpProcedureDatabase::NormalizeTransition("rw28r") == "28R",
          "transition normalization is case-insensitive");
    CHECK(CifpProcedureDatabase::NormalizeTransition("DUMIX") == "DUMIX",
          "enroute transition is preserved");

    const std::vector<Procedure> procs =
        CifpProcedureDatabase::ParseCifpSidLines(CifpLines(), "ZULS");
    CHECK(procs.size() == 1, "one CIFP SID parsed");
    CHECK(procs.size() == 1 && procs[0].ident == "DEP10L" &&
              procs[0].transition == "10L",
          "CIFP SID ident/transition normalized");
    CHECK(procs.size() == 1 && procs[0].legs.size() == 5,
          "CIFP SID has five legs");
    CHECK(procs.size() == 1 && procs[0].legs[0].type == "IF" &&
              procs[0].legs[0].fix == "RW28R",
          "first leg is the runway IF");
    CHECK(procs.size() == 1 && procs[0].legs[2].rf &&
              procs[0].legs[2].rfCenter == "LS042" &&
              std::abs(procs[0].legs[2].rfRadiusNm - 3.3) < 1e-9,
          "CIFP RF centre/radius parsed");
    CHECK(procs.size() == 1 && procs[0].legs[2].rfTurn == -1,
          "CIFP RF left turn parsed");
    CHECK(procs.size() == 1 && procs[0].legs[3].rfTurn == 1,
          "CIFP RF right turn parsed");
    CHECK(procs.size() == 1 && !procs[0].legs[0].posValid,
          "CIFP rows carry no explicit coordinates");

    Procedure resolved = procs[0];
    ResolveStats stats;
    CifpProcedureDatabase::ResolveProcedure(resolved, FakeResolver(true),
                                            29.296667, 90.911667, &stats);
    CHECK(stats.legs == 5, "five legs resolved");
    CHECK(stats.resolved == 4, "four non-runway fixes resolved");
    CHECK(stats.rf == 3, "three RF legs seen");
    CHECK(stats.rfCenters == 3, "three RF centres resolved");
    CHECK(!resolved.legs[0].posValid,
          "runway IF is left unresolved without runway coordinates");
    CHECK(resolved.legs[1].posValid, "LS804 resolved");
    CHECK_NEAR(resolved.legs[1].lat, 29.302833, 1e-6, "LS804 latitude");
    CHECK(resolved.legs[2].rfCenterValid, "LS042 centre resolved");
    CHECK_NEAR(resolved.legs[2].rfCenterLat, 29.357389, 1e-6,
               "LS042 centre latitude");
    CHECK(resolved.legs[3].rfCenterValid, "LS002 centre resolved");
    CHECK(resolved.legs[4].rfCenterValid, "LS003 centre resolved");

    // If a centre is absent from the nav database, reconstruct it from the
    // entry fix, inbound course and radius.
    Procedure reconstructed = procs[0];
    ResolveStats reconStats;
    CifpProcedureDatabase::ResolveProcedure(
        reconstructed, FakeResolver(false), 29.296667, 90.911667, &reconStats);
    CHECK(reconStats.rfCenters == 3,
          "missing centres are reconstructed from entry/radius");
    CHECK(reconstructed.legs[2].rfCenterValid,
          "reconstructed LS042 centre is valid");

    Procedure missing;
    CHECK(!CifpProcedureDatabase::ParseCifpSidLines({}, "ZULS").size(),
          "empty CIFP input yields no procedure");

    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
