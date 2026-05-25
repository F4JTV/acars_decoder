#pragma once
#include <complex>
#include <functional>
#include <string>
#include <ctime>
#include <cstdint>

// ---------------------------------------------------------------------------
// VHF ACARS decoder.
//
// The DSP core (MSK demodulator) and the frame state machine are ported from
// acarsdec by Thierry Leconte (LGPL v2), https://github.com/TLeconte/acarsdec
// They are kept together in a single object because, exactly like acarsdec's
// channel_t, the frame state machine needs to reach back into the MSK state
// (it resets MskDf, toggles the differential phase MskS, and drives the bit
// counter nbits). The CRC/parity tables live in syndrom.h (also from acarsdec).
//
// Input expected by process(): real, AM-demodulated audio sampled at exactly
// ACARS_INTRATE (12500 Hz). On a successful, CRC-validated frame the supplied
// MessageHandler is invoked with a fully-parsed ACARSMessage.
// ---------------------------------------------------------------------------

namespace acars {

    // Audio sample rate the MSK matched filter is designed for. Do NOT change
    // without regenerating the matched filter; the whole demod assumes 12500.
    static constexpr int INTRATE = 12500;

    struct Message {
        std::time_t timestamp = 0;
        char        mode = 0;          // ACARS mode character
        std::string reg;               // aircraft registration / tail (addr)
        char        ack = 0;           // ACK char, or '!' for NAK
        std::string label;             // 2-char label
        char        block = 0;         // block id (bid)
        std::string msgNo;             // message sequence number (downlink only)
        std::string flight;            // flight id / fid (downlink only)
        std::string text;              // free message text
        bool        downlink = false;  // air -> ground
        bool        moreFragments = false; // ETB seen (more blocks follow)
        int         errors = 0;        // residual / corrected parity errors
        float       level = 0.0f;      // signal level in dB
    };

    class Demod {
    public:
        using MessageHandler = std::function<void(const Message&)>;

        explicit Demod(MessageHandler handler);

        // Feed AM-demodulated audio (real, 12500 Hz). Thread: DSP worker.
        void process(const float* samples, int count);

        // Drop any in-progress frame and re-arm the bit hunter.
        void reset();

    private:
        // ---- MSK demodulator state (acarsdec msk.c) --------------------
        void  putbit(float v);
        double MskPhi   = 0.0;
        double MskDf    = 0.0;
        float  MskClk   = 0.0f;
        double MskLvlSum = 0.0;
        int    MskBitCount = 0;
        unsigned int MskS = 0;
        unsigned int idx  = 0;
        std::complex<float> inb[16]; // FLEN = INTRATE/1200 + 1 = 11, padded

        unsigned char outbits = 0;
        int           nbits   = 8;

        // ---- ACARS frame state machine (acarsdec acars.c) --------------
        enum State { WSYN, SYN2, SOH1, TXT, CRC1, CRC2, END };
        void decodeFrame();
        void resetFrame();
        void emitBlock();

        State    state = WSYN;
        uint8_t  txt[256];
        int      len = 0;
        int      err = 0;
        uint8_t  crc[2] = {0, 0};
        float    blkLevel = 0.0f;

        MessageHandler onMessage;
    };

} // namespace acars
