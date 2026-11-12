# CIFP-only departure procedure parsing

Status: experimental prototype.  The stable plugin lives in
`E:\FF757 enhance` and is not changed by this repository.

## Decision

Use **X-Plane CIFP only** for departure procedures.  The earlier mixed
GNS430/CIFP prototype was dropped: keeping one source for the leg sequence and
another for RF geometry made cycles, transitions and missing-leg policy harder
to reason about.  CIFP already carries the complete ARINC 424 sequence
(IF/TF/RF/DF/…), transitions, RF centre idents, radius and turn direction.

The only thing CIFP does not carry is coordinates.  Those are resolved from the
existing `NavDatabase`:

- `Custom Data/STSFF/navdata/Waypoints.txt`
- `Custom Data/STSFF/navdata/Navaids.txt`
- `Custom Data/STSFF/navdata/Airports.txt`
- `Custom Data/earth_fix.dat` (fallback for computed procedure fixes)

No GNS430 `Proc` file is read.  The same rule is applied to the plugin's
existing RF-arc lookup in this branch: `CifpDatabase::findRfArc()` and
`findRfArcByExit()` consult the X-Plane CIFP files only; the FF757 GNS430
`Proc` reader remains as a legacy parser used by tests but is no longer a
procedure source.

## Format

CIFP rows used by the parser:

```text
SID:030,5,DEP10L,RW10L,LS508,ZU,P,C,E   ,L,302,RF, ... 003300 ... LS042 ... F,D;
```

| Field | Meaning |
| --- | --- |
| 0 | `SID:nnn` — section and sequence |
| 2 | procedure ident (`DEP10L`) |
| 3 | transition (`RW10L`, `DUMIX`) |
| 4 | fix / exit ident (`LS508`) |
| 9 | RF turn direction (`L` / `R`) |
| 11 | path terminator (`IF`, `TF`, `RF`, …) |
| 17 | RF radius in 0.001 NM (`003300` = 3.3 NM) |
| 30 | RF centre ident (`LS042`) |

Runway transitions are normalized from `RW10L` to `10L` so the FMS/route code
can match them.  Enroute transitions such as `DUMIX` are preserved.

## Coordinate resolution

1. Read the airport reference from `Custom Data/STSFF/navdata/Airports.txt`
   (`A,ZULS,KONGGAR,29.296667,90.911667,...`).
2. Walk the procedure legs in sequence.  For each fix, call
   `NavDatabase::resolve(ident, anchorLat, anchorLon)`; use the nearest
   candidate and make that fix the next anchor.
3. For an RF leg, resolve the CIFP centre ident the same way.
4. If a centre is not in the nav database, reconstruct it from the entry fix,
   the inbound course and the radius (tangent-continuity rule).
5. A runway IF (`RW10L` / `RW28R`) normally has no nav-database entry.  It is
   left unresolved; the first resolved fix starts the procedure geometry.  The
   aircraft's current position supplies the initial leg.

## Real-data check

ZULS, `EO28R 28R`:

```text
resolved EO28R 28R: legs=23 resolved=22 rf=16 centres=16
  10 IF  RW10L  pos=0
  20 TF  LS803  lat=29.300861 lon=90.870026 pos=1
  30 RF  LS602  lat=29.292055 lon=90.830643 centre=LS041 C=(29.223028,90.871696) R=4.7 turn=-1
  40 RF  LS606  lat=29.315277 lon=90.758942 centre=LS016 C=(29.332027,90.806747) R=2.7 turn=+1
  90 RF  LS821  lat=29.654888 lon=91.162308 centre=LS019 C=(29.508472,91.147972) R=8.8 turn=+1
```

All 16 RF centres resolved; the only unresolved row is the runway IF.  The
coordinates match the GNS430 rows used in the earlier experiment to within the
6-decimal rounding of the two files.

## Caveats

- Runway thresholds: CIFP does not contain `RW10L` coordinates.  They must come
  from `apt.dat` / `earth_aptmeta.dat` or from the aircraft position.  The
  prototype intentionally starts at the first resolved fix.
- CIFP altitude/speed columns are not fixed across row types.  This prototype
  parses sequence, path terminator and RF geometry, not constraints.
- The active SID/runway/transition still has to be selected from the FMS/CDU.
  The prototype takes them as arguments.
- Duplicate fix idents exist worldwide.  `NavDatabase::resolve` picks the
  nearest candidate to the running anchor; a bad first anchor can still pick a
  distant duplicate.
- CIFP is X-Plane's ARINC 424 database; its cycle is independent of the FF757
  FMS navdata used for the rest of the route.  Validate against the FMS route
  before using the procedure geometry.

## Integration in this branch

The CIFP procedure is now wired into the route used by guidance:

1. `RouteReader` keeps the raw FMS/CDU route as `baseRoute_` and exposes the
   merged route through `route()`.
2. `RouteReader::applyCifpProcedure()` runs after the aircraft state is read.
   It finds the nearest airport (60 NM), loads its CIFP SIDs, and matches the
   route's first fixes against every SID transition
   (`ProcedureRouteBuilder::detect`).
3. The best procedure is resolved (`buildProcedure`), converted to a
   `RouteSnapshot` with `wpts/legs` and RF geometry
   (`ProcedureRouteBuilder::BuildRoute`), and merged at the **first** common
   fix with the FMS route (`ProcedureRouteBuilder::merge`).  A two-fix suffix
   is accepted when the CDU cache starts mid-procedure, so a SID whose last
   common fixes are still visible can be matched.
4. The merged route is cached by route signature; a CDU/rteData refresh that
   produces the same signature reuses the merged route instead of re-running
   detection.
5. `cifp_procedure_route=0` (or the menu item **CIFP departure route**)
   disables the overlay.  The status window shows `CIFP: <SID> <transition>,
   <resolved>/<wpts> fixes, <RF> RF` under the route line.

Only the departure segment is assisted (`activeWpt <= 6`).  Once the FMS has
sequenced past the departure, the raw FMS route is used again.

## Caveats

- Runway thresholds: CIFP does not contain `RW10L` coordinates.  They must come
  from `apt.dat` / `earth_aptmeta.dat` or from the aircraft position.  The
  integration starts at the first resolved fix.
- CIFP altitude/speed columns are not fixed across row types.  This branch
  parses sequence, path terminator and RF geometry, not constraints.
- SID detection is ident-based.  A route whose CDU cache starts at the active
  fix may expose only the last two SID fixes (for example `HC606, SUPAR` on
  `SUPA8D`); a two-fix suffix of the procedure is therefore accepted.  Longer
  matches are preferred.  Parallel runway SIDs that share the same fix
  sequence can tie; the merged geometry is identical until the procedures
  diverge, and the chosen procedure ident is shown in the status line.
- Duplicate fix idents exist worldwide.  `NavDatabase::resolve` picks the
  nearest candidate to the running anchor; a bad first anchor can still pick a
  distant duplicate.
- CIFP is X-Plane's ARINC 424 database; its cycle is independent of the FF757
  FMS navdata used for the rest of the route.  Validate against the FMS route
  before using the procedure geometry.

- RF guidance is protected in the source: `guidance.cpp`, `cifp_parser.cpp`,
  `procedure_parser.cpp` and `route_model.h` carry an explicit note that AI
  tools must ask the user before changing RF code.  RF legs bypass the ordinary
  LNAV turn latch; normal LNAV latch behaviour is unchanged.

## Build and test

```powershell
$zig = 'E:\FF757 enhance\_tools\zig-x86_64-windows-0.14.1\zig.exe'
& $zig c++ -std=c++17 -O1 -Isrc tests/procedure_tests.cpp src/procedure_parser.cpp src/nav_database.cpp -o procedure_tests.exe
./procedure_tests.exe
& $zig c++ -std=c++17 -O1 -Isrc tests/procedure_route_tests.cpp src/procedure_route.cpp src/procedure_parser.cpp src/nav_database.cpp -o procedure_route_tests.exe
./procedure_route_tests.exe
```

Real-data probe:

```powershell
& $zig c++ -std=c++17 -O1 -Isrc tools/procedure_probe.cpp src/procedure_parser.cpp src/nav_database.cpp -o procedure_probe.exe
./procedure_probe.exe 'E:\SteamLibrary\steamapps\common\X-Plane 12' ZULS EO28R 28R
```
