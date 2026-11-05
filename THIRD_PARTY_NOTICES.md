# Third-party notices

This repository contains only source code and documentation written for
757 LNAV/VNAV Plus. It does not vendor third-party source code, binaries, or
media assets (no images, fonts, sounds, `.secured` scripts, or aircraft files).

## X-Plane SDK

The plugin is compiled against the X-Plane SDK (XPLM headers and import
library). CMake fetches the SDK from <https://github.com/X-Plane/xpsdk> at
build time; it is **not** included in this repository or in the release zip.
The SDK is distributed under the following BSD-style license:

```
Copyright (c) 2008, Sandy Barbour and Ben Supnik
All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Neither the names of the authors nor that of X-Plane or Laminar Research
      may be used to endorse or promote products derived from this software
      without specific prior written permission from the authors or
      Laminar Research, respectively.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

The release binary links against XPLM as required by every X-Plane plugin. The
SDK implementation itself is provided by X-Plane; no SDK source is copied into
this repository.

## Compiler and C++ runtime

The Windows `64/win.xpl` convenience build is produced with Zig 0.13 and
includes the toolchain's C++ runtime (LLVM libc++ / libc++abi / libunwind).
That runtime is part of the compiler toolchain, not a source asset of this
repository. It is covered by the LLVM Project's Apache License v2.0 with LLVM
Exceptions; a copy is included at
[`licenses/libcxx-LICENSE.txt`](licenses/libcxx-LICENSE.txt).

## Aircraft, SASL and FlightFactor

No FlightFactor, StepToSky, Laminar Research, SASL, `.secured`, or aircraft
files are copied, loaded, decrypted, patched, or redistributed. The plugin only
reads the aircraft's published datarefs and the CDU display interface described
in `docs/DATAREFS.md`. Names such as FlightFactor, Boeing, 757, 767, X-Plane,
and SASL are used only to describe compatibility and are trademarks of their
respective owners.

## GitHub Actions

The CI workflow references the standard GitHub Actions
`actions/checkout`, `actions/upload-artifact`, and
`softprops/action-gh-release`. They are referenced as external services and are
not vendored into this repository.
