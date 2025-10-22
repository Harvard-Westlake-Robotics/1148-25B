package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrists.Pivot;
import frc.robot.subsystems.wrists.Wrist;

public class ArmCommand extends Command {
  public boolean outtakePosition;
  private static double pivotAngle;
  private static double elevatorLength;
  private static double wristAngle;

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
    // level = ScoringLevel.NEUTRAL;
    outtakePosition = false;
    pivotAngle = 0;
    elevatorLength = 0;
    wristAngle = 0;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Pivot.getInstance().goToAngleClosedLoop(pivotAngle);
    Elevator.getInstance().goToHeightClosedLoop(elevatorLength);
    Wrist.getInstance().goToAngleClosedLoop2(wristAngle);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // For manual position, no arm kinematics, just straight going to position
  public void setHeight(ScoringLevel level) {
    // All of these need to be changed based on actual positions
    if (level == ScoringLevel.NEUTRAL) {
      // Changes based on stuff
      setPivotAngle(0);
      setElevatorLength(0);
      setWristAngle(0);
      return;
    }
    if (level == ScoringLevel.SOURCE_CORAL) {
      setPivotAngle(0);
      setElevatorLength(0);
      setWristAngle(0);
      return;
    }
    if (level == ScoringLevel.GROUND_CORAL) {
      setPivotAngle(0);
      setElevatorLength(0);
      setWristAngle(0);
      return;
    }
    if (level == ScoringLevel.GROUND_ALGAE) {
      setPivotAngle(0);
      setElevatorLength(0);
      setWristAngle(0);
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
      setPivotAngle(0);
      setElevatorLength(0);
      setWristAngle(0);
      return;
    }

    // Position will be changed when we bring in encoder code
    if (level == ScoringLevel.HANG) {
      setPivotAngle(12);
      setElevatorLength(0);
      setWristAngle(-14);
      return;
    }
  }

  public void setPivotAngle(double angle) {
    pivotAngle = angle;
  }

  public void setElevatorLength(double length) {
    elevatorLength = length;
  }

  public void setWristAngle(double angle) {
    wristAngle = angle;
  }

  public static double[] getState() {
    return new double[] { pivotAngle, elevatorLength, wristAngle };
  }
}
