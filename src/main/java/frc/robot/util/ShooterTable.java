package frc.robot.util;

/**
 * ShooterTable
 *
 * Maps shot distance (meters) → flywheel RPM using quadratic interpolation.
 *
 * ── HOW QUADRATIC INTERPOLATION WORKS HERE ───────────────────────────────────
 *
 *  For any query distance, we find the three nearest data points and fit a
 *  unique quadratic (parabola) through them using Lagrange's formula.
 *  This gives a smooth curve that passes exactly through your measurements
 *  and curves naturally between them — better than a straight line between
 *  adjacent points, especially when RPM vs distance has a curve to it.
 *
 *  With only 1-2 real data points, the curve is extrapolated from those plus
 *  the placeholder estimates. Replace placeholders as you measure them and
 *  the curve automatically improves.
 *
 *  Edge cases:
 *    - Below the first point: clamps to first RPM (don't spin faster than needed)
 *    - Above the last point:  clamps to last RPM  (don't exceed measured range)
 *    - Exactly on a point:    returns that RPM directly
 *
 * ── TUNING SESSION CHECKLIST ─────────────────────────────────────────────────
 *
 *  For each distance row:
 *    1. Place the robot exactly that distance from the speaker opening.
 *       Use a tape measure — the Localizer pose is not accurate enough for
 *       initial calibration (it depends on the camera being tuned already).
 *    2. Run AimAndShoot. Watch "Shooter/CurrentRPM" on SmartDashboard.
 *    3. Adjust the RPM value in the table until the note lands consistently.
 *    4. Repeat at each distance. You need at least 3 confirmed points for
 *       the quadratic to be meaningful.
 *
 *  Recommended order: start at 1.0m (your existing data point), then 3.0m,
 *  then fill in 2.0m and 4.0m last since they'll be interpolated naturally.
 *
 * ── DATA POINTS ──────────────────────────────────────────────────────────────
 *  Format: { distanceMeters, targetRPM }
 *  Must be in strictly ascending order of distance.
 *  Minimum 3 rows for quadratic interpolation to be meaningful.
 */
public class ShooterTable {

    /**
     * Lookup table: { distanceMeters, targetRPM }
     *
     * TODO [TUNING REQUIRED]: Replace ALL RPM values with real measurements.
     *
     * Only the 1.0m row has a real data point — update it with your actual
     * measured RPM, then measure and fill in every other row.
     *
     * The placeholder values (2800, 3300, 3900, 4500, 5100) are rough estimates
     * based on a linear extrapolation from close range. They will NOT be
     * accurate — treat them as "safe starting points that probably undershoot"
     * rather than real values.
     */
    private static final double[][] TABLE = {
        // { distanceMeters,  targetRPM  }
        { 1.0,  2800 },   // TODO [MEASURE]: confirmed close-range point — update RPM
        { 2.0,  3300 },   // TODO [MEASURE]: placeholder — measure and replace
        { 3.0,  3900 },   // TODO [MEASURE]: placeholder — measure and replace
        { 4.0,  4500 },   // TODO [MEASURE]: placeholder — measure and replace
        { 5.0,  5100 },   // TODO [MEASURE]: placeholder — measure and replace
    };

    /**
     * Returns the target flywheel RPM for the given shot distance using
     * quadratic (Lagrange) interpolation over the three nearest data points.
     *
     * @param distanceMeters Distance from robot to speaker opening (meters).
     *                       Compute this from localizer.getPose() and the
     *                       known speaker position — do NOT use Limelight distance.
     * @return Target RPM for the flywheel at this distance.
     */
    public static double getRPM(double distanceMeters) {
        int n = TABLE.length;

        // Clamp to table bounds
        if (distanceMeters <= TABLE[0][0])     return TABLE[0][1];
        if (distanceMeters >= TABLE[n - 1][0]) return TABLE[n - 1][1];

        // Find the index of the closest point
        int closestIdx = 0;
        double minDist = Math.abs(distanceMeters - TABLE[0][0]);
        for (int i = 1; i < n; i++) {
            double d = Math.abs(distanceMeters - TABLE[i][0]);
            if (d < minDist) {
                minDist    = d;
                closestIdx = i;
            }
        }

        // Select the three nearest points for the quadratic fit.
        // Prefer the point below the query, center, and above when possible.
        int i0, i1, i2;
        if (closestIdx == 0) {
            i0 = 0; i1 = 1; i2 = 2;
        } else if (closestIdx == n - 1) {
            i0 = n - 3; i1 = n - 2; i2 = n - 1;
        } else {
            i0 = closestIdx - 1;
            i1 = closestIdx;
            i2 = closestIdx + 1;
        }

        double x0 = TABLE[i0][0], y0 = TABLE[i0][1];
        double x1 = TABLE[i1][0], y1 = TABLE[i1][1];
        double x2 = TABLE[i2][0], y2 = TABLE[i2][1];
        double x  = distanceMeters;

        // Lagrange quadratic interpolation:
        // L(x) = y0 * [(x-x1)(x-x2)] / [(x0-x1)(x0-x2)]
        //      + y1 * [(x-x0)(x-x2)] / [(x1-x0)(x1-x2)]
        //      + y2 * [(x-x0)(x-x1)] / [(x2-x0)(x2-x1)]
        double l0 = y0 * ((x - x1) * (x - x2)) / ((x0 - x1) * (x0 - x2));
        double l1 = y1 * ((x - x0) * (x - x2)) / ((x1 - x0) * (x1 - x2));
        double l2 = y2 * ((x - x0) * (x - x1)) / ((x2 - x0) * (x2 - x1));

        return l0 + l1 + l2;
    }

    // Static utility class — not instantiated
    private ShooterTable() {}
}
