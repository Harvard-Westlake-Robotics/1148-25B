package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.wrist.WristConstants;

/**
 * Inverse kinematics for a telescoping single-link arm with a wrist.
 * All angles are Rotation2d (use .fromRotations / .getRotations).
 * Distances are WPILib Distance measures (meters).
 *
 * Conventions:
 * - θ (shoulder): 0 = vertical up, + toward +x (rotations from vertical).
 * - β (wrist RELATIVE TO ARM): 0 = vertical when arm vertical (rotations).
 * - α (outtake, GLOBAL): rotations above horizontal (0 = level, + up).
 *
 * Relationship: α = 0.25 - θ - β ⇒ β = 0.25 - θ - α (all in rotations).
 */
public final class ArmKinematics {

    // ---------------- Limits & constants ----------------

    // Wrist pivot -> intake center along α
    public static final Distance WRIST_TO_CENTER = Meters.of(0.10);

    private static final double TAU = 2.0 * Math.PI;
    private static final double EPS = 1e-9;

    private ArmKinematics() {
    }

    // ---------------- Types ----------------
    /** Current joint state (angles as Rotation2d, length as Distance). */
    public static record JointPose(Rotation2d theta, Distance L, Rotation2d beta) {
    }

    /** Desired end-effector state. α is rotations above the horizontal. */
    public static record Target(Distance x, Distance y, Rotation2d alpha) {
    }

    /** IK result. */
    public static record Solution(
            Rotation2d theta,
            Distance L,
            Rotation2d beta,
            boolean reachable, // exact (x,y,α) achieved
            boolean clampedLength,
            boolean clampedShoulder,
            boolean clampedWrist,
            String note) {
    }

    // ---------------- Public API ----------------

    /**
     * Solve inverse kinematics.
     *
     * If (x,y,α) is reachable, returns exact (θ,L,β).
     * If not, projects to the nearest feasible pose and minimally adjusts α so that
     * β
     * stays within limits.
     */
    public static Solution solve(JointPose current, Target target) {
        // Pick α near the current α to avoid wrap jumps
        double currentAlpha = 0.25 - rot(current.theta) - rot(current.beta);
        double desiredAlpha = nearestEquivalentRot(rot(target.alpha), currentAlpha);

        // 1) Wrist pivot Pw = Pc - r * u(α)
        double r = WRIST_TO_CENTER.in(Meters);
        double alphaRad = rot2rad(desiredAlpha);
        double ux = Math.cos(alphaRad), uy = Math.sin(alphaRad);

        double wx = target.x.in(Meters) - r * ux;
        double wy = target.y.in(Meters) - r * uy;

        // Raw (unconstrained) θ, L from wrist pivot
        double Lraw = hypot(wx, wy);
        double thetaRaw = rad2rot(angleFromVertical(wx, wy)); // rotations from vertical

        boolean withinY = wy >= -1e-12; // restrict to half-plane the shoulder can reach
        boolean withinLen = Lraw >= ElevatorConstants.ARM_MIN_LEN.in(Meters) - 1e-12
                && Lraw <= ElevatorConstants.ARM_MAX_LEN.in(Meters) + 1e-12;
        boolean withinTheta = thetaRaw >= rot(WristConstants.ShoulderWrist.WRIST_MIN_DEG) - 1e-12
                && thetaRaw <= rot(WristConstants.ShoulderWrist.WRIST_MAX_DEG) + 1e-12;

        if (withinY && withinLen && withinTheta) {
            // β from α, θ
            double betaRaw = 0.25 - thetaRaw - desiredAlpha;
            boolean withinWrist = betaRaw >= rot(WristConstants.IntakeWrist.WRIST_MIN_DEG) - 1e-12
                    && betaRaw <= rot(WristConstants.IntakeWrist.WRIST_MAX_DEG) + 1e-12;
            if (withinWrist) {
                // Exact target
                return new Solution(
                        Rotation2d.fromRotations(thetaRaw),
                        Meters.of(clamp(Lraw, ElevatorConstants.ARM_MIN_LEN.in(Meters),
                                ElevatorConstants.ARM_MAX_LEN.in(Meters))),
                        Rotation2d.fromRotations(betaRaw),
                        true, false, false, false, "Exact target achieved.");
            }

            // Wrist out of range: adjust α minimally to satisfy wrist limits, then
            // recompute (θ,L) to keep (x,y).
            double alphaMin = 0.25 - thetaRaw - rot(WristConstants.IntakeWrist.WRIST_MAX_DEG);
            double alphaMax = 0.25 - thetaRaw - rot(WristConstants.IntakeWrist.WRIST_MIN_DEG);
            double alphaFeasible = clampPeriodic(desiredAlpha, alphaMin, alphaMax, currentAlpha);

            // Recompute wrist pivot with α_feasible to preserve (x,y)
            double a2 = rot2rad(alphaFeasible);
            double ux2 = Math.cos(a2), uy2 = Math.sin(a2);
            double wx2 = target.x.in(Meters) - r * ux2;
            double wy2 = target.y.in(Meters) - r * uy2;

            double L2 = hypot(wx2, wy2);
            double theta2 = rad2rot(angleFromVertical(wx2, wy2));
            boolean okY2 = wy2 >= -1e-12;
            boolean okLen2 = L2 >= ElevatorConstants.ARM_MIN_LEN.in(Meters) - 1e-12
                    && L2 <= ElevatorConstants.ARM_MAX_LEN.in(Meters) + 1e-12;
            boolean okTh2 = theta2 >= rot(WristConstants.ShoulderWrist.WRIST_MIN_DEG) - 1e-12
                    && theta2 <= rot(WristConstants.ShoulderWrist.WRIST_MAX_DEG) + 1e-12;

            if (okY2 && okLen2 && okTh2) {
                double beta2 = 0.25 - theta2 - alphaFeasible; // guaranteed wrist-legal by construction
                return new Solution(
                        Rotation2d.fromRotations(theta2),
                        Meters.of(L2),
                        Rotation2d.fromRotations(beta2),
                        false, false, false, true, "a adjusted to wrist limits; (x,y) preserved.");
            }

            // If the recomputed pose violates other limits, fall through to projection path
            // below using (wx2, wy2)
            wx = wx2;
            wy = wy2;
            Lraw = L2;
            thetaRaw = theta2;
            desiredAlpha = alphaFeasible; // lock α to feasible value for the projection step
        }

        // 2) Project wrist pivot to feasible region (y >= 0) and clamp length to
        // [min,max]
        boolean clampedLen = false, clampedShoulder = false;

        double px = wx;
        double py = Math.max(wy, 0.0);

        double rr = hypot(px, py);
        if (rr < EPS) {
            // Degenerate: place at min length on +x axis
            px = ElevatorConstants.ARM_MIN_LEN.in(Meters);
            py = 0.0;
            rr = ElevatorConstants.ARM_MIN_LEN.in(Meters);
            clampedLen = true;
            clampedShoulder = true;
        }

        double scale = 1.0;
        if (rr < ElevatorConstants.ARM_MIN_LEN.in(Meters)) {
            scale = ElevatorConstants.ARM_MIN_LEN.in(Meters) / rr;
            clampedLen = true;
        }
        if (rr > ElevatorConstants.ARM_MAX_LEN.in(Meters)) {
            scale = ElevatorConstants.ARM_MAX_LEN.in(Meters) / rr;
            clampedLen = true;
        }
        px *= scale;
        py *= scale;

        double Lp = hypot(px, py);
        double thetaProj = rad2rot(angleFromVertical(px, py));
        if (thetaProj < rot(WristConstants.ShoulderWrist.WRIST_MIN_DEG)) {
            thetaProj = rot(WristConstants.ShoulderWrist.WRIST_MIN_DEG);
            clampedShoulder = true;
        }
        if (thetaProj > rot(WristConstants.ShoulderWrist.WRIST_MAX_DEG)) {
            thetaProj = rot(WristConstants.ShoulderWrist.WRIST_MAX_DEG);
            clampedShoulder = true;
        }

        // 3) Choose α' (near desired) to keep β within limits at θ_proj
        double desiredAlphaNear = nearestEquivalentRot(desiredAlpha, currentAlpha);
        double alphaMin = 0.25 - thetaProj - rot(WristConstants.IntakeWrist.WRIST_MAX_DEG);
        double alphaMax = 0.25 - thetaProj - rot(WristConstants.IntakeWrist.WRIST_MIN_DEG);
        double alphaFeasible = clampPeriodic(desiredAlphaNear, alphaMin, alphaMax, currentAlpha);
        boolean clampedWrist = !approxEqual(alphaFeasible, desiredAlphaNear);
        double betaFeasible = 0.25 - thetaProj - alphaFeasible;

        StringBuilder sb = new StringBuilder("Projected to nearest feasible pose:");
        if (py <= 0.0 + 1e-12)
            sb.append(" y>=0 boundary;");
        if (clampedLen)
            sb.append(" length clamped;");
        if (clampedShoulder)
            sb.append(" shoulder clamped;");
        if (clampedWrist)
            sb.append(" wrist clamped (a adjusted);");

        if (clampedLen && clampedShoulder && clampedWrist)
            sb = new StringBuilder("Illegal Immigrants Found, deploying ICE");
        System.out.println(sb.toString());

        return new Solution(
                Rotation2d.fromRotations(thetaProj),
                Meters.of(Lp),
                Rotation2d.fromRotations(betaFeasible),
                false, clampedLen, clampedShoulder, clampedWrist, sb.toString());
    }

    // ---------------- Optional: forward kinematics (for testing) ----------------
    /** FK: returns target (x,y,α) for a given joint pose. */
    public static Target forward(JointPose pose) {
        double theta = rot(pose.theta);
        double alpha = 0.25 - theta - rot(pose.beta);
        double thetaRad = rot2rad(theta);
        double alphaRad = rot2rad(alpha);

        double xw = pose.L.in(Meters) * Math.sin(thetaRad);
        double yw = pose.L.in(Meters) * Math.cos(thetaRad);
        double xc = xw + WRIST_TO_CENTER.in(Meters) * Math.cos(alphaRad);
        double yc = yw + WRIST_TO_CENTER.in(Meters) * Math.sin(alphaRad);

        return new Target(Meters.of(xc), Meters.of(yc), Rotation2d.fromRotations(alpha));
    }

    // ---------------- Utilities ----------------
    private static double rot(Rotation2d r) {
        return r.getRotations();
    } // rotations

    private static double rot2rad(double rot) {
        return rot * TAU;
    }

    private static double rad2rot(double rad) {
        return rad / TAU;
    }

    private static double hypot(double x, double y) {
        return Math.hypot(x, y);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /** Angle from +Y (vertical) to vector (x,y), radians in (-π, π]. */
    private static double angleFromVertical(double x, double y) {
        return Math.atan2(x, y);
    }

    private static boolean approxEqual(double a, double b) {
        return Math.abs(a - b) <= 1e-9;
    }

    /** Wrap to (-0.5, 0.5] rotations. */
    private static double wrapRot(double rot) {
        double w = ((rot + 0.5) % 1.0 + 1.0) % 1.0 - 0.5;
        return (w <= -0.5 + 1e-12) ? 0.5 : w;
    }

    /** Pick the representative of rot that’s nearest to ref (mod 1 rotation). */
    private static double nearestEquivalentRot(double rot, double ref) {
        double r = wrapRot(rot);
        double k = Math.round(ref - r);
        return r + k;
    }

    /** Clamp a periodic value into [min,max] near ref (all rotations). */
    private static double clampPeriodic(double value, double min, double max, double ref) {
        double v = nearestEquivalentRot(value, ref);
        double lo = nearestEquivalentRot(min, ref);
        double hi = nearestEquivalentRot(max, ref);
        if (lo > hi) {
            double t = lo;
            lo = hi;
            hi = t;
        }
        return clamp(v, lo, hi);
    }
}
