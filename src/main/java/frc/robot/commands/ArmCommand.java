package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.AlgaeIntake;
import frc.robot.subsystems.intake.CoralIntake;
import frc.robot.subsystems.wrist.ArmWrist;
import frc.robot.subsystems.wrist.IntakeWrist;
import frc.robot.util.ArmKinematics;

public class ArmCommand extends Command {

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

  public static Distance getTargetY(ScoringLevel level) {
    switch (level){
      case SOURCE_CORAL:
        return Meters.of(0);
      case GROUND_CORAL:
        return Meters.of(0);
      case GROUND_ALGAE:
        return Meters.of(0);
      case L1:
        return Meters.of(0);
      case L2:
        return Meters.of(0);
      case L3:
        return Meters.of(0);
      case L4:
        return Meters.of(0);
      case TOP_REMOVE:
        return Meters.of(0);
      case BOTTOM_REMOVE:
        return Meters.of(0);
      case NET:
        return Meters.of(0);
      case PROCESSOR:
        return Meters.of(0);
      case HANG:
        return Meters.of(0);
      case NEUTRAL:
      default:
      // Different position based on what is happening
      if (CoralIntake.getInstance().hasCoralHotDog()) {
        // "Keeps it slightly up, ready to go up and score"
        return Meters.of(0);
      } else if (CoralIntake.getInstance().hasCoralBurger()) {
        // "L1 scoring position"
        return Meters.of(0);
      } else if (AlgaeIntake.getInstance().hasAlgae()) {
        // "Keeps elevator straight up but contracted, like lolipop
        return Meters.of(0);
      } else { // Nothing in intake
        // Elevator fully down and intake stowed in.
        return Meters.of(0);
      }
    }
  }

  public void setHeight(ScoringLevel level) {
    ArmCommand.level = level;

    // Current joint pose (rotations + meters)
    var current = new ArmKinematics.JointPose(
        Rotation2d.fromRotations(ArmWrist.getInstance().getWristPosition()),
        Meters.of(Elevator.getInstance().getHeight()), // L
        Rotation2d.fromRotations(IntakeWrist.getInstance().getWristPosition()));

    Rotation2d wristAngle = Rotation2d.fromRotations(0);

    switch (level) {
      case BOTTOM_REMOVE:
        wristAngle = Rotation2d.fromRotations(0);
        outtakePosition = false;
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
        wristAngle = Rotation2d.fromRotations(0);
        outtakePosition = true;
        break;
      case L2:
        wristAngle = Rotation2d.fromRotations(0);
        outtakePosition = true;
        break;
      case L3:
        wristAngle = Rotation2d.fromRotations(0);
        outtakePosition = true;
        break;
      case L4:
        wristAngle = Rotation2d.fromRotations(0);
        outtakePosition = true;
      case NET:
        wristAngle = Rotation2d.fromRotations(0);
        outtakePosition = false;
        break;
      case NEUTRAL:
        outtakePosition = false;
        // Different position based on what is happening
        if (CoralIntake.getInstance().hasCoralHotDog()) {
          // "Keeps it slightly up, ready to go up and score"
        } else if (CoralIntake.getInstance().hasCoralBurger()) {
          // "L1 scoring position"
        } else if (AlgaeIntake.getInstance().hasAlgae()) {
          // "Keeps elevator straight up but contracted, like lolipop
        } else { // Nothing in intake
          // Elevator fully down and intake stowed in.
        }
        break;
      case TOP_REMOVE:
        outtakePosition = false;
        break;
      case PROCESSOR:
        outtakePosition = false;
        break;
      case HANG:
        outtakePosition = false;
        break;
      default:
        outtakePosition = false;
        break;
    }

    var target = new ArmKinematics.Target(
        Meters.of(Drive.getInstance().distanceFromReefEdge()), // X
        getTargetY(level), // Y
        wristAngle);

    var sol = ArmKinematics.solve(current, target);

    targetPos = new double[] { sol.theta().getRotations(), sol.L().abs(Meters), sol.beta().getRotations() };
  }
}
