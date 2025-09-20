package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.ArmWrist;
import frc.robot.subsystems.wrist.IntakeWrist;
import frc.robot.util.ArmKinematics;

public class ArmCommand extends Command {
  /*
   * Stores values needed for scoring in an array. The list is as follows:
   * [0] : Elevator Angle (deg)
   * [1] : Elevator Height (m)
   * [2] : Intake Wrist Angle (deg)
   */
  private double[] targetPos;

  public enum ScoringLevel {
    NEUTRAL,
    GROUND_CORAL,
    GROUND_ALGAE,
    L1,
    L2,
    L3,
    L4,
    TOP_REMOVE,
    BOTTOM_REMOVE,
    NET
  }

  // For accessing the set ScoringLevel by other commands
  public static ScoringLevel level;

  public ArmCommand() {
    this.addRequirements(Elevator.getInstance());
    level = ScoringLevel.NEUTRAL;
    setHeight(level);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    ArmWrist.getInstance().goToAngle(targetPos[0]);
    Elevator.getInstance().goToHeight(targetPos[1]);
    IntakeWrist.getInstance().goToAngle(targetPos[2]);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(targetPos[1] - Elevator.getInstance().getHeight()) < 2;
  }

  public void setHeight(ScoringLevel level) {
    ArmCommand.level = level;

    // Current joint pose (rotations + meters)
    var current =
        new ArmKinematics.JointPose(
            Rotation2d.fromRotations(ArmWrist.getInstance().getWristPosition()),
            Meters.of(Elevator.getInstance().getHeight()), // L
            Rotation2d.fromRotations(IntakeWrist.getInstance().getWristPosition()));

    Distance y = Meters.of(0);
    Rotation2d wristAngle = Rotation2d.fromDegrees(0);

    switch (level) {
      case BOTTOM_REMOVE:
        break;
      case GROUND_ALGAE:
        break;
      case GROUND_CORAL:
        break;
      case L1:
        break;
      case L2:
        break;
      case L3:
        break;
      case L4:
        break;
      case NET:
        break;
      case NEUTRAL:
        break;
      case TOP_REMOVE:
        break;
      default:
        break;
    }

    var target =
        new ArmKinematics.Target(
            Meters.of(Drive.getInstance().distanceFromReefEdge()), // X
            y, // Y
            wristAngle);

    var sol = ArmKinematics.solve(current, target);

    targetPos =
        new double[] {sol.theta().getRotations(), sol.L().abs(Meters), sol.beta().getRotations()};
  }
}
