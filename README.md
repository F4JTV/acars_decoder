# ACARS Decoder Module for SDR++

A **VHF ACARS** (Aircraft Communications Addressing and Reporting System)
decoder module for SDR++. The module demodulates the 2400-baud MSK signal
carried on an aeronautical-band AM carrier, decodes the frames, corrects errors
(odd parity + CCITT CRC) and displays the decoded messages in a **detached
window**.

---

## Features

- Fully self-contained **AM + MSK** demodulation (nothing to wire on the audio side).
- Complete ACARS frame decoding: mode, registration, ACK/NAK, label, block id,
  message number, flight id (downlink), free text.
- **Error correction**: per-character odd parity + CCITT CRC, with single- and
  double-bit recovery (syndrome tables, as in `acarsdec`).
- **Separate message window** with a table, auto-scroll, *Clear* button, *TSV*
  export and a toggle to hide messages that still contain residual errors.
- **Moving the window does not change the VFO frequency** (see below).
- File logging: pick a folder with a **native file-explorer button**; messages
  are appended to `<folder>/acars_log.tsv`.
- Multiple simultaneous instances (one VFO per instance).
- Persistent settings (bandwidth, snap, AGC, display options, log folder).

---

## How it works

VHF ACARS uses **MSK** modulation (index 0.5) centered on 1800 Hz, with tones at
1200 / 2400 Hz, at **2400 baud**, all carried by an **AM carrier** in the
aeronautical band.

The module DSP pipeline:

```
VFO (IQ, 12500 Hz)  ->  AM demodulation (envelope, carrier AGC)  ->
  real audio 12500 Hz  ->  MSK demodulator (1800 Hz VCO + matched filter + PLL)  ->
  bits  ->  frame state machine  ->  parity/CRC/correction  ->  message
```

The decoding core (MSK demodulator + frame state machine + CRC) is **ported from
`acarsdec` by Thierry Leconte** (see Credits). The audio sample rate is fixed at
**12500 Hz** because the MSK matched filter is designed for that rate.

---

## Common ACARS frequencies (VHF, AM)

| Region          | Primary frequency | Common secondaries                 |
|-----------------|-------------------|------------------------------------|
| Europe          | **131.725 MHz**   | 131.525, 131.825, 136.700, 136.750 |
| North America   | **131.550 MHz**   | 130.025, 130.450, 129.125          |
| Worldwide / misc|                   | 131.450, 131.475, 136.900, 136.925 |

> In Europe, start with **131.725 MHz**. The default tuning step (snap) is
> 1 kHz; the aeronautical channel raster is 25 kHz.

---

## Choosing the VFO bandwidth

The **Bandwidth** slider sets the width of the channel filter applied *before* AM
demodulation, i.e. how much spectrum around the carrier is kept. It runs from
4.8 kHz to 12.5 kHz (the fixed audio sample rate), with a **default of 8.4 kHz**,
which is the recommended general-purpose value.

Why 8.4 kHz is a good default:

- The ACARS MSK signal's energy is concentrated within roughly the carrier
  **± 2.4 kHz** (the highest tone is 2400 Hz), but MSK has spectral spreading, so
  the useful signal extends out to about **± 3.5–4 kHz**.
- A bandwidth of ~8 kHz captures the full signal plus a margin for small tuning
  errors and aircraft carrier offset, while still rejecting the neighbouring
  25 kHz channels.

Practical guidance:

| Situation                                   | Suggested bandwidth |
|---------------------------------------------|---------------------|
| General use / unknown conditions            | **8.4 kHz** (default) |
| Weak signal, or you are slightly mistuned   | 8.4 – 10 kHz        |
| Strong signal in a crowded band (adjacent ACARS channels) | 6 – 7.5 kHz |

What to avoid:

- **Too narrow (< ~5 kHz):** clips the 2400 Hz MSK sideband, the matched filter
  loses energy and decoding drops — especially on weaker bursts.
- **Too wide (→ 12.5 kHz):** lets in extra noise and adjacent-channel energy,
  lowering the SNR and increasing false syncs / spurious empty frames.

In short: leave it at **8.4 kHz**, widen a little if the band is quiet and you
miss messages, narrow toward ~6 kHz only if a neighbouring channel is bleeding
in. Pair this with **Carrier AGC** enabled (the default), which normalizes the
recovered audio level for the MSK demodulator.

---

## Building on Ubuntu 24.04

### 1. SDR++ build dependencies

```bash
sudo apt update
sudo apt install build-essential cmake git pkg-config \
    libfftw3-dev libglfw3-dev libvolk-dev libzstd-dev \
    mesa-common-dev libgl-dev
# (+ the dependencies of whatever SDR sources you use: librtlsdr-dev, etc.)
```

### 2. Drop the module into the SDR++ tree

```bash
git clone https://github.com/AlexandreRouma/SDRPlusPlus.git
cp -r sdrpp_acars_decoder SDRPlusPlus/decoder_modules/acars_decoder
```

### 3. Register the module in SDR++'s root `CMakeLists.txt`

Add the option next to the other `OPT_BUILD_*` lines (around line 55):

```cmake
option(OPT_BUILD_ACARS_DECODER "Build the VHF ACARS decoder module (no dependencies required)" ON)
```

Then the `add_subdirectory` block next to the other decoders (around line 292):

```cmake
if (OPT_BUILD_ACARS_DECODER)
add_subdirectory("decoder_modules/acars_decoder")
endif (OPT_BUILD_ACARS_DECODER)
```

### 4. Build

```bash
cd SDRPlusPlus
mkdir -p build && cd build
cmake .. -DOPT_BUILD_ACARS_DECODER=ON
make -j$(nproc)
```

The resulting library is `build/decoder_modules/acars_decoder/acars_decoder.so`.

### 5. Install the module

- Either `sudo make install` (installs into `lib/sdrpp/plugins`),
- or, for a quick test, copy the `.so` into your SDR++ modules folder, then in
  SDR++: **Module Manager** -> add an instance of type `acars_decoder`.

---

## Usage

1. Enable the module instance (checkbox in the left-hand menu).
2. Tune the VFO to an ACARS frequency (e.g. **131.725 MHz**). AM demodulation is
   handled internally — you do not need to put the radio in AM mode.
3. Adjust the **Bandwidth** (~8.4 kHz default — see the section above) and the
   **AGC** if needed.
4. Click **Show Messages** to open the message window.

### Table columns

`Time` · `Md` (mode) · `Lbl` (label) · `Reg` (registration) · `Flight` ·
`MsgNo` (message no.) · `Blk` (block id, `+` if an additional ETB fragment
follows) · `Ack` (ACK or **NAK**) · `Lvl` (level in dB) · `Err` (corrected
errors) · `Text` (message text).

### The window does not move the VFO

SDR++'s waterfall input handler uses the raw mouse state plus a geometric hit
test that ignores overlapping ImGui windows: without care, dragging a floating
window over the waterfall would move the VFO. While the message window is
hovered or focused, the module forces `gui::mainWindow.lockWaterfallControls =
true` for the duration of the interaction. **You can therefore move the window
freely without changing the listening frequency.**

### Logging

Tick **Log to file**, then use the folder field and its **`...` button**, which
opens the native file explorer (GTK/KDE on Linux, the standard dialog on
Windows). Messages are appended in TSV format to `<folder>/acars_log.tsv`. The
folder picker runs the dialog on a worker thread inside the widget, so it never
interacts with the waterfall and cannot retune the VFO. If the folder does not
exist the path turns red and "Invalid folder" is shown.

The **Save TSV** button in the message window writes a timestamped snapshot
(`acars_YYYYMMDD_HHMMSS.tsv`) of the current history into the same folder (or the
SDR++ root if no valid folder is set). Both the live log and the snapshot honour
the "Hide errored" filter, so the files match what you see on screen.

---

## Validation

The decoding core was tested independently of SDR++ (`test/test_acars.cpp`,
buildable under AddressSanitizer/UndefinedBehaviorSanitizer):

- CRC folding to zero verified,
- full decode of a known downlink frame (all fields),
- single-bit error recovery,
- 25 000 samples pushed through the MSK demodulator with no out-of-bounds access.

```bash
cd sdrpp_acars_decoder/src   # first move test/test_acars.cpp next to acars.cpp
g++ -std=c++17 -fsanitize=address,undefined test_acars.cpp acars.cpp -o t && ./t
```

`main.cpp` and `acars.cpp` also compile cleanly against the official SDR++
repository headers.

---

## Credits and license

- MSK demodulator, frame state machine, CRC check/correction and syndrome
  tables: ported from **[acarsdec](https://github.com/TLeconte/acarsdec)** by
  **Thierry Leconte**, under the **LGPL v2** license. The original copyright
  headers are preserved in the affected files.
- Integration patterns (detached window, waterfall locking, VFO/DSP lifecycle)
  follow the SDR++ module conventions and earlier decoder modules (POCSAG, AIS,
  ADS-B, Cospas-Sarsat).

This module is distributed under the **LGPL v2**, consistent with `acarsdec`.
