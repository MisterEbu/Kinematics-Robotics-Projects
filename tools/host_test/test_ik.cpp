// test_ik.cpp
// Desktop test for the dual-arm kinematics sketches (no Arduino board needed).
//
// What it does:
//   1. Picks random CONTINUOUS joint angles and runs an independent copy of the FK
//      math (below, no code from the sketch) to get a target (radius, height, gamma)
//      that is known to be reachable. The angles are not rounded to whole servo
//      pulses, so the sketch cannot hit the target "by luck" on the pulse grid.
//   2. Resets the arm to the home pose and calls the sketch's invKine().
//   3. Checks where the arm really ended up and how much serial data was sent.
//
// If the sketch has the closed-form seed + Jacobian refinement functions
// (Robot_Arms_Final), it also tests the Jacobian refinement on its own.
//
// Build/run with run_tests.sh. See the README for details.

// Standard headers first: the shim defines Arduino-style min()/max() macros that would break them.
#include <vector>
#include <algorithm>
#include "arduino_shim.h"

// Compile the sketch with 4-byte doubles, like an AVR board (Uno/Nano/Mega) does.
#ifdef EMULATE_AVR_FLOAT
#define double float
#endif

#include SKETCH

#ifdef EMULATE_AVR_FLOAT
#undef double
#endif

// ---------------------------------------------------------------------------
// Independent ground truth (does not call any code from the sketch)
// ---------------------------------------------------------------------------
static double mapD(double x, double inMin, double inMax, double outMin, double outMax)
{
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

struct Pose { double r, h, g; };

// Forward kinematics from servo angles in degrees (shoulder, elbow SERVO angle, wrist tilt).
static Pose truthFKdeg(double sDeg, double eServoDeg, double wDeg)
{
    const double l1 = 14.5, l2 = 18.25, l3 = 10.0; // cm, same link lengths as the sketch
    double S = radians(sDeg);
    double E = radians(180.0 - eServoDeg);
    double W = radians(wDeg);
    double t1 = S - PI / 2, t2 = t1 + E, t3 = t2 + W - PI / 2;
    return { l3 * cos(t3) + l2 * cos(t2) + l1 * cos(t1),
             l3 * sin(t3) + l2 * sin(t2) + l1 * sin(t1),
             degrees(t3) };
}

// Same, but reading the angles the arm model currently holds.
static Pose truthFK(const RobotArm &a)
{
    return truthFKdeg(mapD(a.servos[Shoulder].power,  500, 2500, 0, 180),
                      mapD(a.servos[Elbow].power,     500, 2500, 0, 180),
                      mapD(a.servos[Wrist_Tlt].power, 500, 2500, 0, 180));
}

static void setHome(RobotArm &a)
{
    for (int i = 0; i < 6; i++) a.servos[i].power = 1500;
}

static double wrap180(double d)
{
    while (d > 180) d -= 360;
    while (d < -180) d += 360;
    return d;
}

static double pct(std::vector<double> v, double p)
{
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    return v[(size_t)((v.size() - 1) * p)];
}

static double mean(const std::vector<double> &v)
{
    double s = 0;
    for (double x : v) s += x;
    return v.empty() ? 0 : s / v.size();
}

static double randIn(double lo, double hi) { return lo + (hi - lo) * rand() / RAND_MAX; }

int main(int argc, char **argv)
{
    int N = argc > 1 ? atoi(argv[1]) : 500;
    const double POS_TOL = 0.5;   // cm   (what we CHECK here; the sketch's own tolerance is tighter)
    const double ANG_TOL = 2.0;   // deg

    // ---- 1. random reachable targets, starting from the home pose -----------
    srand(12345);
    std::vector<double> posErr, angErr, bytes, writes;
    int ok = 0;

    for (int t = 0; t < N; t++)
    {
        Pose tgt = truthFKdeg(randIn(15, 165), randIn(15, 165), randIn(15, 165));
        setHome(Arm1);

        SSC.resetCounters();
        invKine(Arm1, tgt.r, tgt.h, tgt.g);
        bytes.push_back((double)SSC.bytes);
        writes.push_back((double)SSC.writes);

        Pose got = truthFK(Arm1);
        double pe = hypot(got.r - tgt.r, got.h - tgt.h);
        double ae = fabs(wrap180(got.g - tgt.g));
        posErr.push_back(pe);
        angErr.push_back(ae);
        if (pe <= POS_TOL && ae <= ANG_TOL) ok++;
    }

    printf("Random reachable targets: %d (continuous angles, start pose = all servos at 90 deg)\n", N);
    printf("  reached within %.1f cm and %.1f deg : %d / %d (%.1f%%)\n",
           POS_TOL, ANG_TOL, ok, N, 100.0 * ok / N);
    printf("  position error (cm)  median %.3f   95th %.3f   max %.3f\n",
           pct(posErr, 0.5), pct(posErr, 0.95), pct(posErr, 1.0));
    printf("  gamma error (deg)    median %.3f   95th %.3f   max %.3f\n",
           pct(angErr, 0.5), pct(angErr, 0.95), pct(angErr, 1.0));
    printf("  to the SSC-32U, per invKine() call: %.0f bytes in %.1f writes on average\n",
           mean(bytes), mean(writes));

    // ---- 2. a few named targets ---------------------------------------------
    printf("\nNamed targets (radius cm, height cm, gamma deg):\n");
    const Pose named[] = { {25, 10, 0}, {20, 20, 45}, {15, 30, 90}, {30, 5, -20}, {10, 35, 90} };
    for (const Pose &n : named)
    {
        setHome(Arm1);
        invKine(Arm1, n.r, n.h, n.g);
        Pose got = truthFK(Arm1);
        printf("  target (%5.1f, %5.1f, %6.1f) -> reached (%5.1f, %5.1f, %6.1f)  servos S/E/W = %.1f / %.1f / %.1f deg\n",
               n.r, n.h, n.g, got.r, got.h, got.g,
               mapD(Arm1.servos[Shoulder].power, 500, 2500, 0, 180),
               mapD(Arm1.servos[Elbow].power, 500, 2500, 0, 180),
               mapD(Arm1.servos[Wrist_Tlt].power, 500, 2500, 0, 180));
    }

    // ---- 3. targets the arm cannot reach --------------------------------------
    printf("\nTargets that cannot be reached (the arm must not move and nothing may be sent):\n");
    const Pose bad[] = { {60, 60, 0},       // far outside the reach circle
                         {30, 5, -30},      // wrist point is reachable but needs the wrist tilt servo below 0 deg
                         {0, 0, 0} };       // wrist point too close to the shoulder (inside the minimum reach)
    for (const Pose &b : bad)
    {
        setHome(Arm1);
        uint16_t before[6];
        for (int i = 0; i < 6; i++) before[i] = Arm1.servos[i].power;
        SSC.resetCounters();
        invKine(Arm1, b.r, b.h, b.g);   // old sketches return void, the final one returns bool
        bool moved = false;
        for (int i = 0; i < 6; i++) if (Arm1.servos[i].power != before[i]) moved = true;
        printf("  (%5.1f, %5.1f, %6.1f): arm %s, %lu bytes sent\n", b.r, b.h, b.g,
               moved ? "MOVED" : "stayed put", SSC.bytes);
    }

#ifdef HAVE_SEED_AND_REFINE
    // ---- 4. Jacobian refinement on its own (no closed-form seed) ---------------
    {
        int solved = 0, fromRand = 0, fromRandSolved = 0;
        std::vector<double> iters;
        for (int t = 0; t < N; t++)
        {
            Pose tgt = truthFKdeg(randIn(15, 165), randIn(15, 165), randIn(15, 165));

            // (a) from the home pose
            setHome(Arm1);
            int it = 0;
            bool done = false;
            for (; it < IK_MAX_ITERATIONS; it++)
            {
                if (poseReached(fwdKine(Arm1), tgt.r, tgt.h, tgt.g)) { done = true; break; }
                jacobianStep(Arm1, tgt.r, tgt.h, tgt.g);
            }
            if (done) { solved++; iters.push_back(it); }

            // (b) from a random starting pose (like a later move, not from home)
            setHome(Arm1);
            Arm1.servos[Shoulder].power  = (uint16_t)lround(mapD(randIn(10, 170), 0, 180, 500, 2500));
            Arm1.servos[Elbow].power     = (uint16_t)lround(mapD(randIn(10, 170), 0, 180, 500, 2500));
            Arm1.servos[Wrist_Tlt].power = (uint16_t)lround(mapD(randIn(10, 170), 0, 180, 500, 2500));
            fromRand++;
            if (refineWithJacobian(Arm1, tgt.r, tgt.h, tgt.g)) fromRandSolved++;
        }
        printf("\nJacobian refinement ALONE (no closed-form seed), tolerance %.2f cm / %.2f deg:\n",
               IK_POS_TOL_CM, IK_ANGLE_TOL_DEG);
        printf("  from the home pose   : %d / %d solved, %.1f iterations on average, 95th percentile %.0f\n",
               solved, N, mean(iters), pct(iters, 0.95));
        printf("  from a random pose   : %d / %d solved\n", fromRandSolved, fromRand);
        printf("  (the full invKine() above adds the closed-form seed and a fallback, which is why it does better)\n");
    }
#endif

    return 0;
}
