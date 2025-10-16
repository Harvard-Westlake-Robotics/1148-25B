package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.AlgaeIntake;
import frc.robot.subsystems.intake.CoralIntake;
import frc.robot.subsystems.wrists.Wrist;
import frc.robot.subsystems.wrists.Pivot;
import frc.robot.util.ArmKinematics;

public class ArmCommand extends Command {
  // Tag ids corresponding to the reef ids
  private static double[] targetPos;
  public boolean outtakePosition;

  public static double[] getTargetPos() {
    return targetPos;
  }

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
    NET,
    PROCESSOR,
    HANG
  }

  // For accessing the set ScoringLevel by other commands
  public static ScoringLevel level;

  public ArmCommand() {
    this.addRequirements(Elevator.getInstance(), Pivot.getInstance(), Wrist.getInstance());
    level = ScoringLevel.NEUTRAL;
    outtakePosition = false;
    setHeight(level);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Pivot.getInstance().goToAngle(targetPos[0]);
    Elevator.getInstance().goToHeight(targetPos[1]);
    Wrist.getInstance().goToAngle(targetPos[2]);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return Math.abs(targetPos[1] - Elevator.getInstance().getExtension()) < 2;
  }

  public void setHeight(ScoringLevel level) {
    ArmCommand.level = level;

    // Current joint pose (rotations + meters)
    var current = new ArmKinematics.JointPose(
        Rotation2d.fromRotations(Pivot.getInstance().getWristPosition()), // Theta
        Meters.of(Elevator.getInstance().getExtension()), // L
        Rotation2d.fromRotations(Wrist.getInstance().getWristPosition())); // Beta

    var target = new ArmKinematics.Target(
        Meters.of(getTargetPos(level)[0]), // X
        Meters.of(getTargetPos(level)[1]), // Y
        Rotation2d.fromRotations(getTargetPos(level)[2])); // Intake angle

    var sol = ArmKinematics.solve(current, target);

    targetPos = new double[] { sol.theta().getRotations(), sol.L().abs(Meters), sol.beta().getRotations() };
  }

  public boolean facingForward() {
    Pose2d pose = Drive.getInstance().getPose();
    Translation2d robotTranslation = Drive.getInstance().getPose()
        .getTranslation()
        .plus(
            new Translation2d(
                FieldConstants.ROBOT_REEF_OFFSET_METERS, Drive.getInstance().getPose().getRotation()));

    // Determine reef center based on alliance
    double reefCenterX = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? FieldConstants.BLUE_REEF_CENTER_X
        : FieldConstants.RED_REEF_CENTER_X;
    // Find vector from robot to reef for finding angle
    Translation2d vectorReef = new Translation2d(reefCenterX - robotTranslation.getX(),
      FieldConstants.REEF_CENTER_Y - robotTranslation.getY());
    // Find theta by subtracting angle to reef from robot angle
    double theta = Math.abs(vectorReef.getAngle().minus(pose.getRotation()).getRadians());
    return theta >= 0 && theta <= Math.PI;
  }

  public double[] getTargetPos(ScoringLevel level) {
    // TODO: Add real values
    switch (level) {
      case SOURCE_CORAL:
        outtakePosition = false;
        return new double[] { 0, 0, 0 };
      case GROUND_CORAL:
        outtakePosition = false;
        return new double[] { 0, 0, 0 };
      case GROUND_ALGAE:
        outtakePosition = false;
        return new double[] { 0, 0, 0 };
      case L1:
        outtakePosition = true;
        // Might want to make the boolean false here depending on if L1 outtakes
        // differently
        // If front of robot is looking at reef:
        if (facingForward()) {
          return new double[] { 0, 0, 0 };
        } else {
          // if back of robot is looking at reef
          return new double[] { 0, 0, 0 };
        }
      case L2:
        outtakePosition = true;
        // If front of robot is looking at reef:
        if (facingForward()) {
          return new double[] { 0, 0, 0 };
        } else {
          // if back of robot is looking at reef
          return new double[] { 0, 0, 0 };
        }
      case L3:
        outtakePosition = true;
        // If front of robot is looking at reef:
        if (facingForward()) {
          return new double[] { 0, 0, 0 };
        } else {
          // if back of robot is looking at reef
          return new double[] { 0, 0, 0 };
        }
      case L4:
        outtakePosition = true;
        // If front of robot is looking at reef:
        if (facingForward()) {
          return new double[] { 0, 0, 0 };
        } else {
          // if back of robot is looking at reef
          return new double[] { 0, 0, 0 };
        }
      case TOP_REMOVE:
        outtakePosition = false;
        return new double[] { 0, 0, 0 };
      case BOTTOM_REMOVE:
        outtakePosition = false;
        return new double[] { 0, 0, 0 };
      case NET:
        outtakePosition = false;
        return new double[] { 0, 0, 0 };
      case PROCESSOR:
        outtakePosition = false;
        return new double[] { 0, 0, 0 };
      case HANG:
        outtakePosition = false;
        return new double[] { 0, 0, 0 };
      case NEUTRAL:
        outtakePosition = false;
        // Different position based on what is happening
        if (CoralIntake.getInstance().hasCoralHotDog()) {
          // "Keeps it slightly up, ready to go up and score"
          return new double[] { 0, 0, 0 };
        } else if (CoralIntake.getInstance().hasCoralBurger()) {
          // "L1 scoring position"
          return new double[] { 0, 0, 0 };
        } else if (AlgaeIntake.getInstance().hasAlgae()) {
          // "Keeps elevator straight up but contracted, like lolipop
          return new double[] { 0, 0, 0 };
        } else { // Nothing in intake
          // Elevator fully down and intake stowed in.
          return new double[] { 0, 0, 0 };
        }
      default:
        return new double[] { 0, 0, 0 };
    }
  }
}
