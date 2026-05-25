// Local validation harness for the ACARS decoder core. Not shipped.
// Build: g++ -std=c++17 -fsanitize=address,undefined test_acars.cpp acars.cpp -o t
#include <complex>
#include <functional>
#include <string>
#include <ctime>
#include <cstdint>
#include <cstdio>
#include <vector>
#include <cassert>
#include <cmath>

#define private public
#define protected public
#include "acars.h"
#undef private
#undef protected
#include "syndrom.h"

using namespace acars;

// Add odd parity in bit 7 to a 7-bit value.
static uint8_t par(uint8_t v) {
    v &= 0x7f;
    int ones = 0;
    for (int i = 0; i < 7; i++) { if (v & (1 << i)) ones++; }
    if ((ones & 1) == 0) { v |= 0x80; }
    return v;
}

// Feed one byte LSB-first into the demod's bit assembler.
static void feedByte(Demod& d, uint8_t b) {
    for (int i = 0; i < 8; i++) {
        float v = (b & (1 << i)) ? 1.0f : -1.0f;
        d.putbit(v);
    }
}

int main() {
    // ---- Build a known DOWNLINK ACARS block --------------------------------
    std::vector<uint8_t> body; // mode .. ETX (this is what ends up in txt[])
    body.push_back(par('2'));                 // mode
    const char* reg = ".F-GKXL";              // 7 chars, leading dot is fill
    for (int i = 0; i < 7; i++) body.push_back(par((uint8_t)reg[i]));
    body.push_back(0x15);                     // ACK = NAK (already odd parity)
    body.push_back(par('5'));                 // label[0]
    body.push_back(par('U'));                 // label[1]
    body.push_back(par('1'));                 // block id (digit => downlink)
    body.push_back(0x02);                     // STX
    const char* no  = "M01A";                 // message no (4)
    for (int i = 0; i < 4; i++) body.push_back(par((uint8_t)no[i]));
    const char* fid = "XL1234";               // flight id (6)
    for (int i = 0; i < 6; i++) body.push_back(par((uint8_t)fid[i]));
    const char* tx  = "HELLO WORLD/EX1";      // free text
    for (size_t i = 0; tx[i]; i++) body.push_back(par((uint8_t)tx[i]));
    body.push_back(0x83);                      // ETX (odd parity already)

    // ---- Compute the CRC the same way emitBlock will check it --------------
    unsigned short c = 0;
    for (uint8_t b : body) { update_crc(c, b); }
    uint8_t crc0 = c & 0xff;
    uint8_t crc1 = (c >> 8) & 0xff;
    // Verify the appended FCS folds the running CRC to zero.
    unsigned short chk = 0;
    for (uint8_t b : body) { update_crc(chk, b); }
    update_crc(chk, crc0);
    update_crc(chk, crc1);
    printf("CRC fold check (expect 0): %u\n", chk);
    assert(chk == 0 && "CRC FCS does not fold to zero");

    // ---- Run it through the decoder via the bit assembler ------------------
    Message got;
    bool fired = false;
    Demod d([&](const Message& m){ got = m; fired = true; });

    feedByte(d, 0x16); // SYN
    feedByte(d, 0x16); // SYN
    feedByte(d, 0x01); // SOH
    for (uint8_t b : body) { feedByte(d, b); }
    feedByte(d, crc0);
    feedByte(d, crc1);

    printf("message fired: %s\n", fired ? "yes" : "NO");
    assert(fired && "no message emitted");

    printf("  mode   = '%c'\n", got.mode);
    printf("  reg    = '%s'\n", got.reg.c_str());
    printf("  ack    = '%c'\n", got.ack);
    printf("  label  = '%s'\n", got.label.c_str());
    printf("  block  = '%c' (downlink=%d, more=%d)\n", got.block, got.downlink, got.moreFragments);
    printf("  msgNo  = '%s'\n", got.msgNo.c_str());
    printf("  flight = '%s'\n", got.flight.c_str());
    printf("  text   = '%s'\n", got.text.c_str());
    printf("  errors = %d\n", got.errors);

    assert(got.mode == '2');
    assert(got.reg == "F-GKXL");
    assert(got.ack == '!');
    assert(got.label == "5U");
    assert(got.block == '1');
    assert(got.downlink == true);
    assert(got.msgNo == "M01A");
    assert(got.flight == "XL1234");
    assert(got.text == "HELLO WORLD/EX1");
    assert(got.errors == 0);
    printf("FRAME DECODE: PASS\n");

    // ---- Single-bit error correction test ----------------------------------
    {
        Message g2; bool f2 = false;
        Demod d2([&](const Message& m){ g2 = m; f2 = true; });
        feedByte(d2, 0x16); feedByte(d2, 0x16); feedByte(d2, 0x01);
        auto corrupt = body;
        corrupt[14] ^= 0x04;          // flip one bit in the text region
        for (uint8_t b : corrupt) { feedByte(d2, b); }
        feedByte(d2, crc0); feedByte(d2, crc1);
        printf("single-bit-error recovery fired: %s, text='%s'\n",
               f2 ? "yes" : "no", f2 ? g2.text.c_str() : "");
        assert(f2 && g2.text == "HELLO WORLD/EX1");
        printf("ERROR CORRECTION: PASS\n");
    }

    // ---- MSK demod safety / smoke test (ASan/UBSan bounds) -----------------
    {
        Demod d3([](const Message&){});
        std::vector<float> buf(12500 * 2, 0.0f);
        // Sweep an MSK-like 1200..2400 Hz audio so the matched filter and
        // syndrom indexing exercise their full range without UB.
        double ph = 0.0;
        for (size_t n = 0; n < buf.size(); n++) {
            double f = 1800.0 + 600.0 * sin(2.0 * M_PI * 50.0 * n / 12500.0);
            ph += 2.0 * M_PI * f / 12500.0;
            buf[n] = (float)sin(ph);
        }
        d3.process(buf.data(), (int)buf.size());
        printf("MSK demod smoke test: ran %zu samples, no crash\n", buf.size());
    }

    printf("\nALL TESTS PASSED\n");
    return 0;
}
