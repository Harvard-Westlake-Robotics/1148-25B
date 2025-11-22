package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class VisionConstants {
  // Limelight Standard Deviation Coefficients
  public static final double xyStdDev = 6.85;
  public static final double rStdDev = 6.85;

  public static final double maxAmbiguity = 0.3;
  public static final Distance maxDistToCamera = Meters.of(6.5);

  public static final String[] limelightNames = {
    "limelight-a", "limelight-b", "limelight-c", "limelight-d"
  };
}
