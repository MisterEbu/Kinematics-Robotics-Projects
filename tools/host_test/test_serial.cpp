// test_serial.cpp
// Exercises the serial command handling of Robot_Arms_Final.ino (setup() and loop())
// with a fake Serial port, so the parser can be checked without the board.
//
// Build/run with:  bash run_tests.sh ../../Robot_Arms_Final/Robot_Arms_Final.ino --serial

#include <vector>
#include <algorithm>
#include <string>
#include "arduino_shim.h"

#ifdef EMULATE_AVR_FLOAT
#define double float
#endif

#include SKETCH

#ifdef EMULATE_AVR_FLOAT
#undef double
#endif

static int failures = 0;

static void check(bool cond, const char *what)
{
    printf("  [%s] %s\n", cond ? "PASS" : "FAIL", what);
    if (!cond) failures++;
}

static bool contains(const std::string &hay, const char *needle)
{
    return hay.find(needle) != std::string::npos;
}

// Feeds text, runs loop() enough times to consume it, and reports what came out.
static void send(const std::string &text, std::string &serialOut, std::string &sscOut)
{
    Serial.resetCounters();
    SSC.resetCounters();
    Serial.feed(text);
    for (int i = 0; i < 200 && Serial.available(); i++) loop();
    loop();
    serialOut = Serial.log;
    sscOut = SSC.log;
}

int main()
{
    std::string so, ss;

    setup();
    printf("Start-up\n");
    check(contains(Serial.log, "Ready"), "prints a ready message");

    printf("Valid commands\n");
    Arm1.servos[Shoulder].power = 1500;
    send("1 20 20 45\n", so, ss);
    check(contains(so, "OK arm 1"), "reports success for \"1 20 20 45\\n\"");
    check(!ss.empty() && ss[0] == '#' && ss.back() == '\r', "sends one SSC-32U line ending in \\r");
    check(contains(ss, "#2 P") && contains(ss, "#9 P") && contains(ss, "T1000"), "line has Arm 1 channels 2..9 and T1000");
    check(!contains(ss, "#16"), "does not touch Arm 2's channels");

    send("2 15 30 90\r\n", so, ss);
    check(contains(so, "OK arm 2"), "accepts Windows line endings (\\r\\n)");
    check(contains(ss, "#16 P") && contains(ss, "T500") && !contains(ss, "#2 P"), "Arm 2 line uses channels 16..25 and T500");

    send("1 20 20 45", so, ss);
    check(ss.empty() && so.empty(), "waits (does not block or act) until the line ends");
    send("\n", so, ss);
    check(contains(so, "OK arm 1"), "completes the command when the newline arrives");

    send("1 2", so, ss);
    send("0 20 4", so, ss);
    send("5\n", so, ss);
    check(contains(so, "OK arm 1"), "handles a command that arrives in pieces");

    send("\n\n  \n", so, ss);
    check(ss.empty(), "blank lines do nothing");

    send("  1   20.5   20.25   45.0  \n", so, ss);
    check(contains(so, "OK arm 1"), "tolerates extra spaces and decimals");

    send("1 30 5 -20\n", so, ss);
    check(contains(so, "OK arm 1"), "accepts a negative gamma");

    printf("Bad input (nothing may be sent to the servos)\n");
    send("hello\n", so, ss);
    check(contains(so, "Usage") && ss.empty(), "text instead of numbers");
    send("1 20 20\n", so, ss);
    check(contains(so, "Usage") && ss.empty(), "too few numbers");
    send("1 20 20 45 7\n", so, ss);
    check(contains(so, "Usage") && ss.empty(), "too many numbers");
    send("3 20 20 45\n", so, ss);
    check(contains(so, "Arm must be 1 or 2") && ss.empty(), "arm number 3");
    send("0 20 20 45\n", so, ss);
    check(contains(so, "Arm must be 1 or 2") && ss.empty(), "arm number 0");
    send("1 60 60 0\n", so, ss);
    check(contains(so, "not reachable") && ss.empty(), "target out of reach");
    send("1 30 5 -30\n", so, ss);
    check(contains(so, "not reachable") && ss.empty(), "target blocked by the wrist servo limit");
    send("1 nan 20 45\n", so, ss);
    check(ss.empty(), "nan is rejected");
    send("1 20 20 45abc\n", so, ss);
    check(contains(so, "Usage") && ss.empty(), "junk after the numbers");

    std::string longLine(200, '1');
    send(longLine + "\n", so, ss);
    check(contains(so, "too long") && ss.empty(), "a very long line is dropped");
    send("1 20 20 45\n", so, ss);
    check(contains(so, "OK arm 1"), "works normally again after a long line");

    printf("\n%s (%d failure%s)\n", failures ? "SOME TESTS FAILED" : "ALL SERIAL TESTS PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
