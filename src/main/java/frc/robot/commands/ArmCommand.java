package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.AlgaeIntake;
import frc.robot.subsystems.intake.CoralIntake;
import frc.robot.subsystems.wrist.ArmWrist;
import frc.robot.subsystems.wrist.IntakeWrist;
import frc.robot.util.ArmKinematics;

public class ArmCommand extends Command {
  // Tag ids corresponding to the reef ids
  private final int[] tagIds = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
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
    this.addRequirements(Elevator.getInstance(), ArmWrist.getInstance(), IntakeWrist.getInstance());
    level = ScoringLevel.NEUTRAL;
    outtakePosition = false;
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
    return Math.abs(targetPos[1] - Elevator.getInstance().getExtention()) < 2;
  }

  public void setHeight(ScoringLevel level) {
    ArmCommand.level = level;

    // Current joint pose (rotations + meters)
    var current =
        new ArmKinematics.JointPose(
            Rotation2d.fromRotations(ArmWrist.getInstance().getWristPosition()),
            Meters.of(Elevator.getInstance().getExtention()), // L
            Rotation2d.fromRotations(IntakeWrist.getInstance().getWristPosition()));

    Rotation2d wristAngle = Rotation2d.fromRotations(0);

    switch (level) {
      case NEUTRAL:
        outtakePosition = false;
        // Different position based on what is happening
        if (CoralIntake.getInstance().hasCoralHotDog()) {
          // "Keeps it slightly up, ready to go up and score"
          wristAngle = Rotation2d.fromRotations(0);
        } else if (CoralIntake.getInstance().hasCoralBurger()) {
          // "L1 scoring position"
          wristAngle = Rotation2d.fromRotations(0);
        } else if (AlgaeIntake.getInstance().hasAlgae()) {
          // "Keeps elevator straight up but contracted, like lolipop
          wristAngle = Rotation2d.fromRotations(0);
        } else { // Nothing in intake
          // Elevator fully down and intake stowed in.
          wristAngle = Rotation2d.fromRotations(0);
        }
        break;
      case GROUND_ALGAE:
        wristAngle = Rotation2d.fromRotations(0);
        outtakePosition = false;
        break;
      case GROUND_CORAL:
        wristAngle = Rotation2d.fromRotations(0);
        outtakePosition = false;
        break;
      case SOURCE_CORAL:
        wristAngle = Rotation2d.fromRotations(0);
        outtakePosition = false;
        break;
      case L1:
        // Might want to make the boolean false here depending on if L1 outtakes
        // differently
        // If front of robot is looking at reef:
        if (facingForward()) {
          wristAngle = Rotation2d.fromRotations(0);
        } else if (facingBackward()) {
          // if back of robot is looking at reef
          wristAngle = Rotation2d.fromRotations(0);
        }
        outtakePosition = true;
        break;
      case L2:
        // If front of robot is looking at reef:
        if (facingForward()) {
          wristAngle = Rotation2d.fromRotations(0);
        } else if (facingBackward()) {
          // if back of robot is looking at reef
          wristAngle = Rotation2d.fromRotations(0);
        }
        outtakePosition = true;
        break;
      case L3:
        // If front of robot is looking at reef:
        if (facingForward()) {
          wristAngle = Rotation2d.fromRotations(0);
        } else if (facingBackward()) {
          // if back of robot is looking at reef
          wristAngle = Rotation2d.fromRotations(0);
        }
        outtakePosition = true;
        break;
      case L4:
        // If front of robot is looking at reef:
        if (facingForward()) {
          wristAngle = Rotation2d.fromRotations(0);
        } else if (facingBackward()) {
          // if back of robot is looking at reef
          wristAngle = Rotation2d.fromRotations(0);
        }
        outtakePosition = true;
      case NET:
        wristAngle = Rotation2d.fromRotations(0);
        outtakePosition = false;
        break;
      case BOTTOM_REMOVE:
        wristAngle = Rotation2d.fromRotations(0);
        outtakePosition = false;
        break;
      case TOP_REMOVE:
        wristAngle = Rotation2d.fromRotations(0);
        outtakePosition = false;
        break;
      case PROCESSOR:
        wristAngle = Rotation2d.fromRotations(0);
        outtakePosition = false;
        break;
      case HANG:
        wristAngle = Rotation2d.fromRotations(0);
        outtakePosition = false;
        break;
      default:
        wristAngle = Rotation2d.fromRotations(0);
        outtakePosition = false;
        break;
    }

    var target =
        new ArmKinematics.Target(
            Meters.of(
                (Elevator.getInstance().getExtention())
                    * Math.cos(ArmWrist.getInstance().getWristPosition())), // X
            WristConstants.getTargetY(level), // Y
            wristAngle);

    var sol = ArmKinematics.solve(current, target);

    targetPos =
        new double[] {sol.theta().getRotations(), sol.L().abs(Meters), sol.beta().getRotations()};
  }

  public boolean facingForward() {
    return false;
  }

  public boolean facingBackward() {
    return false;
  }
}
