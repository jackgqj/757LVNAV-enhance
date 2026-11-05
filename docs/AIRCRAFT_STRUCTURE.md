# FF757 aircraft structure (observed)

This document summarizes the parts of the FlightFactor 757 that matter for
this plugin. It is not a complete aircraft manual; it records what was
inspected while designing the external guidance layer.

## Root

```
Boeing757-Full/
  *.acf                      757-200 / -300 / -F / -C32, XP11 and XP12 variants
  757-cockpit.obj            cockpit object
  airfoils/                  airfoil data
  charts/                    airport charts
  co-routes/                 user route files loaded by the FMS
  cockpit/                   legacy 2D panel assets
  cockpit_3d/                3D panel/EFIS assets
  data/                      FMS data, navdata, cockpit configuration
  DOCS/                      FCOM, FMS manual, checklists, dataref API doc
  liveries/                  liveries
  modules/                   shared SASL/cpp modules and license keys
  objects/                   3D objects
  plugins/                   XPLM plugins (avionics, SASL, effects)
  sounds/                    sound banks
  x-updater/                 updater
```

The `.acf` files contain `acf/_custom_autopilot 1`, which tells X-Plane that
the aircraft supplies its own autopilot brains instead of using the default
Plane Maker autopilot logic.

## Plugins

```
plugins/
  757767Avionics/            C++ FMS/avionics/autopilot bridge
   64/win.xpl, mac.xpl, lin.xpl
   cache/                    navigation database cache
   routes/                   legacy route folder (readme says do not use)
  FF_Effects/                visual effects
  FF_PluginManager/          aircraft variant/loading manager
  sasl/                      SASL runtime; data/ contains encrypted scripts
```

`757767Avionics` is the important plugin. Its binary publishes the
`757Avionics/*` datarefs and overrides several X-Plane GPS/nav datarefs
(`override_gps`, `override_fms_advance`, `override_nav_heading`). It also
publishes a route data tree whose base is observed as
`1-sim/for-internal-usage-only/fms`.

## Modules

```
modules/
  configuration/             widget and configuration resources
  cpp-libs/
    stsff-airports/          airport database library
    stsff-librain/           librain integration
    stsff-navigraph/         Navigraph/AIRNAV3 database access
    stsff-radar/             weather radar
  Custom Module/             aircraft systems, largely .secured SASL scripts
  data.key, main.key, serial.key
```

The `Custom Module` tree contains many `.secured` files (autopilot glue,
flight controls, EFIS data providers, systems, etc.) plus `.fnt`, `.png` and
`.txt` resources. These are encrypted and are **not** modified or decrypted by
this plugin.

## Data

```
data/
  fms/
    perfData/                752PW, 752RR, 753PW, ... performance tables
    remoteCDU/               remote CDU web app
  cycle_info.txt             AIRAC 1702 (as shipped in this installation)
  navdata.zip                navigation database
  757cockpit.ini, 764cockpit.ini, 767cockpit.ini
  plugins/TGDS/              terrain/weather display resources
```

The performance data is used by the FMS; the plugin does not read or alter it.

## SASL

`plugins/sasl/data/modules/Custom Module` is the aircraft's SASL script tree.
The scripts are `.secured` (encrypted). `plugins/sasl/data/init/*.secured`
contains the SASL runtime initialization. There is no supported way to
modify these scripts without breaking the aircraft and the vendor license.
This is why the enhancement is implemented as a separate XPLM plugin.

## Where the guidance comes from

The observed flight-guidance interface is:

- `757Avionics/ap/*` — AP mode, roll/pitch engaged/armed modes, FD states,
  warnings, autothrottle state.
- `757Avionics/fms/*` — active-leg navigation data (`AirnavDtk`,
  `AirnavDis`, `AirnavXtk`, `AirnavBrg`, `AirnavTke`, `AirnavAtd`,
  `AirnavFromTo`), performance/speed limits, VNAV phase, and route-dump
  triggers.
- `1-sim/for-internal-usage-only/fms/rteData/*` — route waypoint arrays and
  estimated altitudes used by the ND/FMC (private, undocumented; **not
  registered** on the FF757 v2 build tested here).
- `1-sim/cduL|R/display/*` — the left/right CDU character grid. This is the
  fallback route source: the plugin reads the LEGS page text and pages through
  it with `757Avionics/CDU{,2}/prev_page` / `next_page`, then restores the
  original page.
- Standard X-Plane datarefs — `sim/cockpit/radios/gps_*` (the FF757 writes
  these while `override_gps=1`), `sim/cockpit2/autopilot/*`,
  `sim/cockpit/autopilot/nav_steer_deg_mag`, and the legacy FD pitch.

The plugin reads these and writes the standard X-Plane target datarefs plus
the CDU page keys described in [`DATAREFS.md`](DATAREFS.md).

## Design consequences

- The FF757's own AP mode logic, autothrottle, bank limiter, and FMS pages
  remain in charge.
- The plugin can be removed by deleting `plugins/757LVNAVPlus/`.
- A future FF757 update can rename or remove private route datarefs; the
  plugin detects that through route-vs-CDI validation and falls back to the
  CDU LEGS page, then to the GPS CDI.
- Because SASL is encrypted, the plugin cannot add FMS pages or change the
  CDU; it can only read the LEGS display, press the existing PREV/NEXT page
  keys while scanning, and improve the targets that the existing guidance
  consumes.
