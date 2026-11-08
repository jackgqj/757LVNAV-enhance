#include "cifp_parser.h"

#include <cstdio>
#include <string>
#include <vector>

using namespace lvnv;

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, msg) do { \
    ++g_checks; \
    if (!(cond)) { \
        ++g_failures; \
        std::printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); \
    } \
} while (0)

#define CHECK_NEAR(a, b, eps, msg) CHECK(std::abs((a) - (b)) <= (eps), msg)

int main() {
    // Real rows from the XP12 CIFP file for KSEA (Resources/default
    // data/CIFP/KSEA.dat), RNAV (RNP) Z RWY 16C approach: a slinky of three
    // RF arcs HEGEM -> SEGAW -> PINTR -> EPUDE.
    std::vector<std::string> lines = {
        "APPCH:010,A,R16CZ,PPAUL,PPAUL,K1,P,C,E  B, ,   ,IF, , , , , ,      ,    ,    ,    ,    ,+,05900,     ,18000, ,   ,    ,   , , , , , ,A,P,S;",
        "APPCH:020,A,R16CZ,PPAUL,FERIE,K1,P,C,E   , ,010,TF, , , , , ,      ,    ,    ,    ,    ,+,05300,     ,     , ,   ,    ,   , , , , , ,A,P,S;",
        "APPCH:030,A,R16CZ,PPAUL,HEGEM,K1,P,C,E   , ,010,TF, , , , , ,      ,    ,    ,    ,    ,+,05000,     ,     ,-,230,    ,   , , , , , ,A,P,S;",
        "APPCH:040,A,R16CZ,PPAUL,SEGAW,K1,P,C,E   ,L,010,RF, , , , , ,003150,    ,    ,    ,0040,+,03700,     ,     ,-,210,    ,   ,CFBRK,K1,P,C, ,A,P,S;",
        "APPCH:050,A,R16CZ,PPAUL,PINTR,K1,P,C,E   ,R,010,RF, , , , , ,002800,    ,    ,    ,0015,+,03200,     ,     , ,   ,    ,   ,CFBRH,K1,P,C, ,A,P,S;",
        "APPCH:060,A,R16CZ,PPAUL,EPUDE,K1,P,C,E   ,R,010,RF, , , , , ,003380,    ,    ,    ,0035,+,02200,     ,     , ,   ,    ,   ,CFBRX,K1,P,C, ,A,P,S;",
        "APPCH:070,A,R16CZ,PPAUL,SODOE,K1,P,C,EE  , ,302,TF, , , , , ,      ,    ,    ,    ,    ,+,01900,     ,     , ,   ,    ,   , , , , , ,A,P,S;",
        // Same arc repeated by the R16RZ procedure: must dedupe.
        "APPCH:020,A,R16RZ,PPAUL,HEGEM,K1,P,C,E   , ,010,TF, , , , , ,      ,    ,    ,    ,    ,+,05000,     ,     ,-,230,    ,   , , , , , ,A,P,S;",
        "APPCH:040,A,R16RZ,PPAUL,SEGAW,K1,P,C,E   ,L,010,RF, , , , , ,003150,    ,    ,    ,0040,+,03700,     ,     ,-,210,    ,   ,CFBRK,K1,P,C, ,A,P,S;",
        // STAR rows without RF legs must not produce anything.
        "STAR:010,4,HAWKZ7,BTG,BTG,K1,D, ,V  H, ,   ,IF, , , , , ,      ,    ,    ,    ,    ,+,FL240,     ,18000, ,   ,    ,   , , , , , , , , ;",
        "STAR:020,4,HAWKZ7,BTG,PTERA,K1,P,C,E   , ,   ,TF, , , , , ,      ,    ,    ,    ,    ,+,FL220,     ,     , ,   ,    ,   , , , , , , , , ;",
    };

    std::vector<RfArcRow> arcs = CifpDatabase::ParseCifpLines(lines);
    CHECK(arcs.size() == 3, "three distinct RF arcs parsed (R16RZ duplicate dropped)");

    if (arcs.size() >= 3) {
        CHECK(arcs[0].entryIdent == "HEGEM" && arcs[0].exitIdent == "SEGAW",
              "first arc entry/exit (entry is the previous row's fix)");
        CHECK(arcs[0].turn == -1, "first arc turns left");
        CHECK_NEAR(arcs[0].radiusNm, 3.150, 1e-9, "radius scaled from 0.001 NM");
        CHECK(arcs[0].centerIdent == "CFBRK", "arc centre fix ident");

        CHECK(arcs[1].entryIdent == "SEGAW" && arcs[1].exitIdent == "PINTR" &&
              arcs[1].turn == 1 && arcs[1].centerIdent == "CFBRH",
              "second arc chains from the first arc's exit");
        CHECK_NEAR(arcs[1].radiusNm, 2.800, 1e-9, "second arc radius");

        CHECK(arcs[2].entryIdent == "PINTR" && arcs[2].exitIdent == "EPUDE" &&
              arcs[2].turn == 1 && arcs[2].centerIdent == "CFBRX",
              "third arc chains from the second arc's exit");
    }

    // An RF row at the start of a transition block (no entry fix) is dropped.
    {
        std::vector<std::string> orphan = {
            "APPCH:010,A,R16CZ,PPAUL,SEGAW,K1,P,C,E   ,L,010,RF, , , , , ,003150,    ,    ,    ,0040,+,03700,     ,     ,-,210,    ,   ,CFBRK,K1,P,C, ,A,P,S;",
        };
        CHECK(CifpDatabase::ParseCifpLines(orphan).empty(),
              "RF row without a previous fix in its block is dropped");
    }

    // Malformed rows (no turn direction, no centre) are dropped.
    {
        std::vector<std::string> bad = {
            "APPCH:010,A,R16CZ,PPAUL,HEGEM,K1,P,C,E   , ,010,TF, , , , , ,      ,    ,    ,    ,    ,+,05000,     ,     ,-,230,    ,   , , , , , ,A,P,S;",
            "APPCH:020,A,R16CZ,PPAUL,SEGAW,K1,P,C,E   , ,010,RF, , , , , ,003150,    ,    ,    ,0040,+,03700,     ,     ,-,210,    ,   ,CFBRK,K1,P,C, ,A,P,S;",
            "APPCH:030,A,R16CZ,PPAUL,PINTR,K1,P,C,E   ,R,010,RF, , , , , ,002800,    ,    ,    ,0015,+,03200,     ,     , ,   ,    ,   , ,K1,P,C, ,A,P,S;",
        };
        CHECK(CifpDatabase::ParseCifpLines(bad).empty(),
              "RF rows without turn direction or centre fix are dropped");
    }

    // FF757 procedure database rows (Custom Data/STSFF/navdata/Proc).  Field 4
    // is the direction the FMS itself uses: 1 = ARC LEFT, 2 = ARC RIGHT.
    {
        std::vector<std::string> ff = {
            "SID,EO28R,28R,5",
            "",                       // real Proc files separate blocks with blanks
            "IF,RW10L,29.301139,90.888000, ,0.0,0.0,0,0,0,0,0,0,0,0",
            "TF,LS803,29.300861,90.870028,0, ,0.0,0.0,0.0,0.0,0,0,0,0,0,0,0,0",
            "RF,LS602,29.292056,90.830639,1,LS041,290.7,4.7,0,0,0,0,0,0,0,0",
            "RF,LS606,29.315278,90.758944,2,LS016,43.4,2.7,0,0,0,0,0,0,0,0",
            "RF,LS608,29.374861,90.822944,2,LS016,99.4,2.7,0,0,0,0,0,0,0,0",
            "STAR,OTHER,10L,2",
            "RF,LS999,29.1,90.1,2,LSX,10.0,5.0,0,0,0,0,0,0,0,0",
            "",                       // trailing blank / one-field junk must not crash
            "RF",
            "not-a-proc-row",
        };
        std::vector<RfArcRow> arcs = CifpDatabase::ParseFfProcLines(ff);
        CHECK(arcs.size() == 4, "FF proc: four RF rows parsed across two blocks");
        if (arcs.size() == 4) {
            CHECK(arcs[0].entryIdent == "LS803" && arcs[0].exitIdent == "LS602",
                  "FF proc: entry is the previous row's fix");
            CHECK(arcs[0].turn == -1, "FF proc: direction 1 is a left arc");
            CHECK_NEAR(arcs[0].radiusNm, 4.7, 1e-9, "FF proc: radius from field 7");
            CHECK(arcs[0].centerIdent == "LS041", "FF proc: centre fix");
            CHECK(arcs[1].entryIdent == "LS602" && arcs[1].exitIdent == "LS606" &&
                  arcs[1].turn == 1 && arcs[1].centerIdent == "LS016",
                  "FF proc: direction 2 is a right arc");
            CHECK(arcs[2].exitIdent == "LS608" && arcs[2].turn == 1 &&
                  arcs[2].centerIdent == "LS016",
                  "FF proc: consecutive arcs share the centre fix");
            CHECK(arcs[3].entryIdent.empty() && arcs[3].exitIdent == "LS999",
                  "FF proc: block change resets the entry fix");
        }
    }

    // A one-field row in the middle of a Proc file must reset the chain.
    // Reading p[1] from such a row was the 1.2.22/1.2.25 crash (0xc0000005
    // inside Trim): blank lines and a bare "RF" make SplitComma return a
    // single element, so the unconditional prevIdent assignment ran off the
    // end of the vector.
    {
        std::vector<std::string> mid = {
            "SID,TEST,10L,2",
            "TF,AAA,29.1,90.1,0, ,0.0,0.0,0.0,0.0,0,0,0,0,0,0,0,0",
            "RF",
            "RF,BBB,29.2,90.2,2,CCC,90.0,3.0,0,0,0,0,0,0,0,0",
        };
        std::vector<RfArcRow> arcs = CifpDatabase::ParseFfProcLines(mid);
        CHECK(arcs.size() == 1, "one-field row does not drop or corrupt the RF chain");
        if (!arcs.empty()) {
            CHECK(arcs[0].entryIdent.empty(),
                  "one-field row clears the entry fix before the next RF");
            CHECK(arcs[0].exitIdent == "BBB" && arcs[0].centerIdent == "CCC",
                  "RF after a one-field row keeps its own fields");
        }
    }

    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
