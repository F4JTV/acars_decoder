/*
 * ACARS decoder core for SDR++.
 *
 * Ported from acarsdec by Thierry Leconte (LGPL v2):
 *   - MSK demodulator           (msk.c)
 *   - frame state machine        (acars.c: decodeAcars)
 *   - CRC / parity / error fix   (acars.c: blk_thread, fixprerr, fixdberr)
 *   - block -> message fields    (output.c)
 *   - CRC / parity / syndrom data (syndrom.h)
 *
 * Original work Copyright (c) 2015-2017 Thierry Leconte.
 * SDR++ adaptation by F4JTV (ADRASEC 06).
 *
 * This file is distributed under the GNU Library General Public License v2,
 * consistent with the upstream acarsdec licensing.
 */
#include "acars.h"
#include "syndrom.h"

#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace acars {

    // MSK matched-filter geometry (acarsdec msk.c).
    static constexpr int FLEN     = (INTRATE / 1200) + 1;     // 11
    static constexpr int MFLTOVER = 12;
    static constexpr int FLENO    = (FLEN * MFLTOVER) + 1;    // 133

    // Half-wave rectified 600 Hz cosine matched filter, oversampled MFLTOVER
    // times. Built once on first use (acarsdec builds it for channel 0 only).
    static float gH[FLENO];
    static bool  gHReady = false;

    static void buildMatchedFilter() {
        if (gHReady) { return; }
        for (int i = 0; i < FLENO; i++) {
            float v = cosf(2.0f * (float)M_PI * 600.0f / INTRATE / MFLTOVER *
                           (i - (FLENO - 1) / 2));
            gH[i] = (v < 0.0f) ? 0.0f : v;
        }
        gHReady = true;
    }

    // ACARS control characters.
    static constexpr uint8_t SYN = 0x16;
    static constexpr uint8_t SOH = 0x01;
    static constexpr uint8_t STX = 0x02;
    static constexpr uint8_t ETX = 0x83;
    static constexpr uint8_t ETB = 0x97;
    static constexpr uint8_t DLE = 0x7f;
    static constexpr uint8_t NAK = 0x15;

    static constexpr int MAXPERR = 3;

    // PLL constants (acarsdec).
    static constexpr float PLLG = 38e-4f;
    static constexpr float PLLC = 0.52f;

    static inline bool isDownlinkBlock(uint8_t bid) {
        return bid >= '0' && bid <= '9';
    }

    // -----------------------------------------------------------------------
    Demod::Demod(MessageHandler handler) : onMessage(std::move(handler)) {
        buildMatchedFilter();
        std::memset(inb, 0, sizeof(inb));
        std::memset(txt, 0, sizeof(txt));
        resetFrame();
        // resetFrame() sets nbits = 1; we want a clean 8-bit hunt at startup.
        nbits = 8;
    }

    void Demod::reset() {
        resetFrame();
        MskPhi = 0.0;
        MskDf  = 0.0;
        MskClk = 0.0f;
        MskS   = 0;
        idx    = 0;
        outbits = 0;
        nbits   = 8;
        std::memset(inb, 0, sizeof(inb));
    }

    // ---- MSK demodulator (acarsdec demodMSK) ------------------------------
    void Demod::process(const float* buf, int count) {
        int idxL  = (int)idx;
        double p  = MskPhi;

        for (int n = 0; n < count; n++) {
            // VCO at 1800 Hz plus PLL correction.
            double s = 1800.0 / INTRATE * 2.0 * M_PI + MskDf;
            p += s;
            if (p >= 2.0 * M_PI) { p -= 2.0 * M_PI; }

            // Mixer: shift the 1800 Hz subcarrier down to baseband.
            float in = buf[n];
            inb[idxL] = in * std::polar(1.0f, -(float)p);
            idxL = (idxL + 1) % FLEN;

            // Bit clock: one symbol every pi (two bits per 3*pi/2? see acarsdec).
            MskClk += (float)s;
            if (MskClk >= 3.0f * (float)M_PI / 2.0f - (float)s / 2.0f) {
                MskClk -= 3.0f * (float)M_PI / 2.0f;

                // Matched filter, fractional-delay aligned to the clock phase.
                int o = (int)(MFLTOVER * (MskClk / s + 0.5));
                if (o > MFLTOVER) { o = MFLTOVER; }
                std::complex<float> v(0.0f, 0.0f);
                for (int j = 0; j < FLEN; j++, o += MFLTOVER) {
                    v += gH[o] * inb[(j + idxL) % FLEN];
                }

                // Normalize and accumulate level for SNR/level reporting.
                float lvl = std::abs(v);
                v /= (lvl + 1e-8f);
                MskLvlSum += (double)lvl * lvl / 4.0;
                MskBitCount++;

                // Differential MSK bit decision + phase error for the PLL.
                double dphi;
                float  vo;
                if (MskS & 1) {
                    vo = v.imag();
                    dphi = (vo >= 0.0f) ? -v.real() : v.real();
                } else {
                    vo = v.real();
                    dphi = (vo >= 0.0f) ? v.imag() : -v.imag();
                }
                if (MskS & 2) { putbit(-vo); } else { putbit(vo); }
                MskS++;

                // PLL loop filter.
                MskDf = PLLC * MskDf + (1.0 - PLLC) * PLLG * dphi;
            }
        }

        idx    = (unsigned int)idxL;
        MskPhi = p;
    }

    // Shift one decoded bit (LSB-first) into the byte assembler.
    void Demod::putbit(float v) {
        outbits >>= 1;
        if (v > 0.0f) { outbits |= 0x80; }
        nbits--;
        if (nbits <= 0) { decodeFrame(); }
    }

    void Demod::resetFrame() {
        state  = WSYN;
        MskDf  = 0.0;
        nbits  = 1;
    }

    // ---- ACARS frame state machine (acarsdec decodeAcars) -----------------
    void Demod::decodeFrame() {
        uint8_t r = outbits;

        switch (state) {
        case WSYN:
            if (r == SYN) { state = SYN2; nbits = 8; return; }
            if (r == (uint8_t)~SYN) { MskS ^= 2; state = SYN2; nbits = 8; return; }
            nbits = 1;
            return;

        case SYN2:
            if (r == SYN) { state = SOH1; nbits = 8; return; }
            if (r == (uint8_t)~SYN) { MskS ^= 2; nbits = 8; return; }
            resetFrame();
            return;

        case SOH1:
            if (r == SOH) {
                state = TXT;
                len = 0;
                err = 0;
                nbits = 8;
                MskLvlSum = 0.0;
                MskBitCount = 0;
                return;
            }
            resetFrame();
            return;

        case TXT:
            txt[len] = r;
            len++;
            if ((numbits[r] & 1) == 0) {
                err++;
                if (err > MAXPERR + 1) { resetFrame(); return; }
            }
            if (r == ETX || r == ETB) { state = CRC1; nbits = 8; return; }
            if (len > 20 && r == DLE) {
                // Missed text-end marker: back up and treat trailing bytes as CRC.
                len -= 3;
                crc[0] = txt[len];
                crc[1] = txt[len + 1];
                state = CRC2;
                blkLevel = (MskBitCount > 0)
                    ? 10.0f * log10f((float)(MskLvlSum / MskBitCount))
                    : 0.0f;
                emitBlock();
                state = END;
                nbits = 8;
                return;
            }
            if (len > 240) { resetFrame(); return; }
            nbits = 8;
            return;

        case CRC1:
            crc[0] = r;
            state = CRC2;
            nbits = 8;
            return;

        case CRC2:
            crc[1] = r;
            blkLevel = (MskBitCount > 0)
                ? 10.0f * log10f((float)(MskLvlSum / MskBitCount))
                : 0.0f;
            emitBlock();
            state = END;
            nbits = 8;
            return;

        case END:
        default:
            resetFrame();
            nbits = 8;
            return;
        }
    }

    // ---- CRC error correction (acarsdec) ----------------------------------
    static bool fixprerr(uint8_t* t, int blkLen, unsigned short crc, int* pr, int pn) {
        if (pn > 0) {
            for (int i = 0; i < 8; i++) {
                if (fixprerr(t, blkLen, crc ^ syndrom[i + 8 * (blkLen - *pr + 1)], pr + 1, pn - 1)) {
                    t[*pr] ^= (1 << i);
                    return true;
                }
            }
            return false;
        }
        if (crc == 0) { return true; }
        for (int i = 0; i < 2 * 8; i++) {
            if (syndrom[i] == crc) { return true; }
        }
        return false;
    }

    static bool fixdberr(uint8_t* t, int blkLen, unsigned short crc) {
        for (int i = 0; i < 2 * 8; i++) {
            if (syndrom[i] == crc) { return true; }
        }
        for (int k = 0; k < blkLen; k++) {
            int bo = 8 * (blkLen - k + 1);
            for (int i = 0; i < 8; i++) {
                for (int j = 0; j < 8; j++) {
                    if (i == j) { continue; }
                    if ((crc ^ syndrom[i + bo] ^ syndrom[j + bo]) == 0) {
                        t[k] ^= (1 << i);
                        t[k] ^= (1 << j);
                        return true;
                    }
                }
            }
        }
        return false;
    }

    // ---- block validation + field extraction (acarsdec) -------------------
    void Demod::emitBlock() {
        if (len < 13) { return; }

        // Force STX/ETX bits on the start-of-text byte.
        txt[12] &= (ETX | STX);
        txt[12] |= (ETX & STX);

        // Parity scan.
        int pn = 0;
        int pr[MAXPERR];
        for (int i = 0; i < len; i++) {
            if ((numbits[txt[i]] & 1) == 0) {
                if (pn < MAXPERR) { pr[pn] = i; }
                pn++;
            }
        }
        if (pn > MAXPERR) { return; }
        int residualErr = pn;

        // CRC over text + 2 CRC bytes.
        unsigned short c = 0;
        for (int i = 0; i < len; i++) { update_crc(c, txt[i]); }
        update_crc(c, crc[0]);
        update_crc(c, crc[1]);

        // Try to repair.
        if (pn) {
            if (!fixprerr(txt, len, c, pr, pn)) { return; }
        } else if (c) {
            if (!fixdberr(txt, len, c)) { return; }
        }

        // Re-check parity and strip the parity bit.
        pn = 0;
        for (int i = 0; i < len; i++) {
            if ((numbits[txt[i]] & 1) == 0) { pn++; }
            txt[i] &= 0x7f;
        }
        if (pn) { return; }

        // ----- field extraction (acarsdec output.c) -----
        Message msg;
        msg.timestamp = std::time(nullptr);
        msg.errors    = residualErr;
        msg.level     = blkLevel;

        int k = 0;
        msg.mode = (char)txt[k]; k++;

        // Aircraft registration (7 chars, dots dropped).
        for (int i = 0; i < 7; i++, k++) {
            if (txt[k] != '.') { msg.reg.push_back((char)txt[k]); }
        }

        // ACK / NAK.
        uint8_t ack = txt[k];
        msg.ack = (ack == NAK) ? '!' : (char)ack;
        k++;

        // Label (2 chars).
        char l0 = (char)txt[k]; k++;
        char l1 = (char)txt[k]; if ((uint8_t)l1 == 0x7f) { l1 = 'd'; } k++;
        msg.label.push_back(l0);
        msg.label.push_back(l1);

        // Block id.
        msg.block = (char)txt[k]; k++;
        msg.downlink = isDownlinkBlock((uint8_t)msg.block);

        // Start-of-text byte (0x02 normally; 0x03 means empty text).
        uint8_t bs = txt[k]; k++;
        uint8_t be = txt[len - 1];
        msg.moreFragments = (be == 0x17); // stripped ETB

        if (bs != 0x03) {
            if (msg.downlink) {
                for (int i = 0; i < 4 && k < len - 1; i++, k++) {
                    msg.msgNo.push_back((char)txt[k]);
                }
                for (int i = 0; i < 6 && k < len - 1; i++, k++) {
                    msg.flight.push_back((char)txt[k]);
                }
            }
            int txtLen = len - k - 1;
            if (txtLen > 0) {
                msg.text.assign((const char*)txt + k, (size_t)txtLen);
            }
        }

        if (onMessage) { onMessage(msg); }
    }

} // namespace acars
