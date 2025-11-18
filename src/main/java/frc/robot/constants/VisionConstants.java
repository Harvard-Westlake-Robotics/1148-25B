package frc.robot.constants;

public class VisionConstants {
    // Limelight Standard Deviation Coefficients
    public static double xyStdDevCoeff = 6.85;
    public static double rStdDevCoeff = 6.85;

    // ================================= Limelight Standard Dev =================================

    private static double sdMultiplier = 1;

    public static double getSdMultiplier() {
        return sdMultiplier;
    }

    public static void setSdMultiplier(double multiplier) {
        sdMultiplier = multiplier;
    }
}
