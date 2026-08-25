# Third-party notice: SdFat

This crate contains code adapted from **SdFat** by Bill Greiman
(<https://github.com/greiman/SdFat>), specifically the files

- `src/SdCard/PioSdio/PioSdioCard.pio`
- `src/SdCard/PioSdio/PioSdioCard.cpp`
- `src/SdCard/PioSdio/PioSdioCard.h`

as of commit `cda057318bec196183d4cc92b01bc1dd64bbfb02` (2025-08-12).

What was adapted, what was changed, and what is original to this crate is set out in
full in the `# Provenance` section of this crate's `src/lib.rs`. In particular, the PIO
programs `cmd_rsp`, `rd_clk`, `rd_data`, `wr_data` and `wr_resp` in `src/programs.rs` are
transcribed from `PioSdioCard.pio` instruction for instruction; `rd_data_1bit` and the
data CRC-16 in `src/crc.rs` are **not** from SdFat.

Note that SdFat's own top-level `LICENSE.md` carries the years `2011..2020`. The
`PioSdio` sources carry `2011-2025`, and that is the notice reproduced below, because
those are the files this crate adapts.

SdFat's licence follows verbatim.

---

MIT License

Copyright (c) 2011-2025 Bill Greiman

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
