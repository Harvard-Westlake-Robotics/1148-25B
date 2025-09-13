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

public class ElevatorCommand extends Command {

  private double[] targetPos;
  public boolean straightIntakePosition;

  /*
   * An enum that stores y and wrist values that are fed into our ArmKinematics
   * class.
   * The x value is taken from our distance from the reef
   */
  public enum ScoringLevel {
    NEUTRAL,
    SOURCE_CORAL,
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

  public ElevatorCommand() {
    this.addRequirements(Elevator.getInstance());
    level = ScoringLevel.NEUTRAL;
    straightIntakePosition = false;
    setHeight(level);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    ArmWrist.getInstance().goToAngle(targetPos[0]);
    Elevator.getInstance().goToHeight(targetPos[1]);
    IntakeWrist.getInstance().goToAngle(targetPos[2]);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return Math.abs(targetPos[1] - Elevator.getInstance().getHeight()) < 2;
  }

  public void setHeight(ScoringLevel level) {
    ElevatorCommand.level = level;

    // Current joint pose (rotations + meters)
    var current = new ArmKinematics.JointPose(
        Rotation2d.fromRotations(ArmWrist.getInstance().getWristPosition()),
        Meters.of(Elevator.getInstance().getHeight()), // L
        Rotation2d.fromRotations(IntakeWrist.getInstance().getWristPosition()));

    Distance y = Meters.of(0);
    Rotation2d wristAngle = Rotation2d.fromRotations(0);

    switch (level) {
      case BOTTOM_REMOVE:
        straightIntakePosition = false;
        break;
      case GROUND_ALGAE:
        straightIntakePosition = false;
        break;
      case GROUND_CORAL:
        straightIntakePosition = false;
        break;
      case SOURCE_CORAL:
        straightIntakePosition = false;
        break;
      case L1:
        straightIntakePosition = false;
        break;
      case L2:
        straightIntakePosition = true;
        break;
      case L3:
        straightIntakePosition = true;
        break;
      case L4:
        straightIntakePosition = true;
        wristAngle = Rotation2d.fromRotations(0);
      case NET:
        straightIntakePosition = false;
        break;
      case NEUTRAL:
        straightIntakePosition = false;
        break;
      case TOP_REMOVE:
        straightIntakePosition = false;
        break;
      default:
        straightIntakePosition = false;
        break;
    }

    var target = new ArmKinematics.Target(
        Meters.of(Drive.getInstance().distanceFromReefEdge()), // X
        y, // Y
        wristAngle);

    var sol = ArmKinematics.solve(current, target);

    targetPos = new double[] {
        sol.theta().getRotations(),
        sol.L().abs(Meters),
        sol.beta().getRotations()
    };
  }
}