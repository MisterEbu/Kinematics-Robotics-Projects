// arduino_shim.h
// Just enough of the Arduino API to compile the arm sketches on a normal PC
// so the kinematics can be tested without the board. This is a test aid only.
// It is NOT the real Arduino core, so it cannot catch AVR-specific problems.
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>

#ifndef PI
#define PI 3.14159265358979323846
#endif

// On AVR these are plain functions in avr-libc, on a PC they live in std::.
using std::isnan;
using std::isinf;

static inline double radians(double deg) { return deg * PI / 180.0; }
static inline double degrees(double rad) { return rad * 180.0 / PI; }

// Same integer-only map() the Arduino core uses (this is why fractional values get truncated).
static inline long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define sq(x) ((x) * (x))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))
#define F(s) (s)

// A fake serial port. Output is counted so tests can see how much was sent,
// and input can be queued with feed() so loop() can be exercised.
struct FakeStream
{
    unsigned long bytes  = 0;   // characters printed since the last reset
    unsigned long writes = 0;   // number of print/println calls since the last reset
    std::string   last;         // most recent complete thing printed
    std::string   log;          // everything printed (cleared by resetCounters)
    std::string   in;           // queued input
    size_t        inPos = 0;
    bool          echo  = false;

    void begin(long) {}
    void resetCounters() { bytes = 0; writes = 0; last.clear(); log.clear(); }

    void record(const std::string &s)
    {
        bytes += s.size();
        writes++;
        last = s;
        log += s;
        if (echo) fputs(s.c_str(), stdout);
    }

    void print(const char *s)  { record(s); }
    void print(int v)          { record(std::to_string(v)); }
    void print(long v)         { record(std::to_string(v)); }
    void print(unsigned v)     { record(std::to_string(v)); }
    void print(unsigned long v){ record(std::to_string(v)); }
    void print(double v, int digits = 2)
    {
        char b[40];
        snprintf(b, sizeof b, "%.*f", digits, v);
        record(b);
    }
    void println(const char *s)         { record(std::string(s) + "\r\n"); }
    void println(int v)                 { record(std::to_string(v) + "\r\n"); }
    void println(double v, int digits = 2)
    {
        char b[40];
        snprintf(b, sizeof b, "%.*f\r\n", digits, v);
        record(b);
    }
    void println()                      { record("\r\n"); }

    void feed(const std::string &s) { in += s; }
    int  available() { return (int)(in.size() - inPos); }
    int  read()      { return inPos < in.size() ? (unsigned char)in[inPos++] : -1; }

    // The old sketches call these in loop(). The tests never run those loops.
    long   parseInt()   { return 0; }
    double parseFloat() { return 0.0; }
};

struct SoftwareSerial : FakeStream
{
    SoftwareSerial(int, int) {}
};

inline FakeStream Serial;

static inline void delay(unsigned long) {}
