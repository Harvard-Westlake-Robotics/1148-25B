package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.AlgaeIntake;
import frc.robot.subsystems.intake.CoralIntake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;
import org.littletonrobotics.junction.Logger;

public class ArmCommand extends Command {
  private static double pivotAngle;
  private static double elevatorLength;
  private static double wristAngle;

  private static final double PIVOT_TOLERANCE = 1;
  private static final double ELEVATOR_TOLERANCE = 0.5;
  private static final double WRIST_TOLERANCE = 1;

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
    HANG_DEPLOY,
    HANG_CLIMB
  }

  // For accessing the set ScoringLevel by other commands
  public static ScoringLevel level;

  public ArmCommand() {
    this.addRequirements(Elevator.getInstance(), Pivot.getInstance(), Wrist.getInstance());
    pivotAngle = Units.degreesToRotations(70);
    elevatorLength = 0;
    wristAngle = Units.degreesToRotations(0);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Pivot.getInstance().goToAngleClosedLoop(pivotAngle);
    Elevator.getInstance().goToHeightClosedLoop(elevatorLength);
    Wrist.getInstance().goToAngleClosedLoop(wristAngle);
    logState();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  // Logs the current state of the arm command in both degrees and rotations
  public void logState() {
    Logger.recordOutput(
        "RealOutputs/Arm Command State Degrees/Target Pivot Angle (degrees)", getStateDegrees()[0]);
    Logger.recordOutput(
        "RealOutputs/Arm Command State Degrees/Target Elevator Length (meters)",
        getStateDegrees()[1]);
    Logger.recordOutput(
        "RealOutputs/Arm Command State Degrees/Target Wrist Angle (degrees)", getStateDegrees()[2]);
    Logger.recordOutput(
        "RealOutputs/Arm Command State Rotations/Target Pivot Angle (rotations)",
        getStateRotations()[0]);
    Logger.recordOutput(
        "RealOutputs/Arm Command State Rotations/Target Elevator Length (meters)",
        getStateRotations()[1]);
    Logger.recordOutput(
        "RealOutputs/Arm Command State Rotations/Target Wrist Angle (rotations)",
        getStateRotations()[2]);
  }

  // For manual position, no arm kinematics, just straight going to position
  public void setHeight(ScoringLevel level) {
    // All of these need to be changed based on actual positions
    if (level == ScoringLevel.NEUTRAL) {
      // All that is needed is for the elevator to extend to score at any level (except maybe L4)
      // TODO:
      if (CoralIntake.getInstance().hasCoralHotDog()) {
        // setPivotAngle(52);
        // setWristAngle(-27);
      }

      // When intake has algae in it, the pivot moves to a 90 degree angle and the wrist holds the
      // algae straight up
      // So it's easier to go up and score on the barge later
      // (Given energy chain broken, idk if we even want this but i'm keeping it in here anyway)
      // TODO
      else if (AlgaeIntake.getInstance().hasAlgae()) {
        setPivotAngle(90);
        setWristAngle(-20);
      }
      // When intake has coral in it burger style, the arm just goes to L1 scoring position and I
      // don't wanna
      // figure that out as of October 28th at 6:52 PM
      // TODO
      else if (CoralIntake.getInstance().hasCoralBurger()) {

      }

      // If the intake has nothing in it, it fully stows itself (arm down, elevator in)
      else {
        setPivotAngle(15);
        setElevatorLength(0);
        setWristAngle(80);
      }
    }
    if (level == ScoringLevel.SOURCE_CORAL) {
      setPivotAngle(0);
      setElevatorLength(0);
      setWristAngle(0);
      return;
    }
    if (level == ScoringLevel.GROUND_CORAL) {
      setPivotAngle(20);
      setElevatorLength(0);
      setWristAngle(-60);
      return;
    }
    if (level == ScoringLevel.GROUND_ALGAE) {
      setPivotAngle(21.5);
      setElevatorLength(0);
      setWristAngle(-51.5);
      return;
    }
    if (level == ScoringLevel.L1) {
      setPivotAngle(0);
      setElevatorLength(0);
      setWristAngle(0);
      return;
    }
    if (level == ScoringLevel.L2) {
      setPivotAngle(0);
      setElevatorLength(0);
      setWristAngle(0);
      return;
    }
    if (level == ScoringLevel.L3) {
      setPivotAngle(0);
      setElevatorLength(0);
      setWristAngle(0);
      return;
    }
    if (level == ScoringLevel.L4) {
      setPivotAngle(0);
      setElevatorLength(0);
      setWristAngle(0);
      return;
    }
    if (level == ScoringLevel.TOP_REMOVE) {
      setPivotAngle(0);
      setElevatorLength(0);
      setWristAngle(0);
      return;
    }
    if (level == ScoringLevel.BOTTOM_REMOVE) {
      setPivotAngle(0);
      setElevatorLength(0);
      setWristAngle(0);
      return;
    }
    if (level == ScoringLevel.NET) {
      setPivotAngle(0);
      setElevatorLength(0);
      setWristAngle(0);
      return;
    }
    if (level == ScoringLevel.PROCESSOR) {
      setPivotAngle(25);
      setElevatorLength(0);
      setWristAngle(-51);
      return;
    }

    // Position will be changed when we bring in encoder code
    if (level == ScoringLevel.HANG_DEPLOY) {
      setPivotAngle(102);
      setElevatorLength(0);
      setWristAngle(-25);
      return;
    }

    if (level == ScoringLevel.HANG_CLIMB) {
      setPivotAngle(15);
      setElevatorLength(0);
      setWristAngle(-25);
      return;
    }
  }

  public void setPivotAngle(double angle) {
    pivotAngle = Units.degreesToRotations(angle);
  }

  public void setElevatorLength(double length) {
    elevatorLength = length;
  }

  public void setWristAngle(double angle) {
    wristAngle = Units.degreesToRotations(angle);
  }

  // Returns the state of the arm command in degrees
  public static double[] getStateDegrees() {
    return new double[] {
      Units.rotationsToDegrees(pivotAngle), elevatorLength, Units.rotationsToDegrees(wristAngle)
    };
  }

  // Returns the state of the arm command in rotations
  public static double[] getStateRotations() {
    return new double[] {pivotAngle, elevatorLength, wristAngle};
  }

  public boolean withinTargetRange() {
    return Math.abs(Pivot.getInstance().getAngleDeg() - pivotAngle) <= PIVOT_TOLERANCE
        && Math.abs(Elevator.getInstance().getCurrentHeight() - elevatorLength)
            <= ELEVATOR_TOLERANCE
        && Math.abs(Wrist.getInstance().getAngleDeg()) <= WRIST_TOLERANCE;
  }
}
