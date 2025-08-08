package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.ArmWrist;
import frc.robot.subsystems.wrist.IntakeWrist;

public class ElevatorCommand extends Command {
  /* Stores values needed for scoring in an array. The list is as follows:
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

  public ElevatorCommand() {
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
    ElevatorCommand.level = level;
    if (level == ScoringLevel.GROUND_CORAL) {
      targetPos = new double[] {0, 0, 0};
    } else if (level == ScoringLevel.GROUND_ALGAE) {
      targetPos = new double[] {0, 0, 0};
    } else if (level == ScoringLevel.L1) {
      targetPos = new double[] {0, 0, 0};
    } else if (level == ScoringLevel.L2) {
      targetPos = new double[] {0, 0, 0};
    } else if (level == ScoringLevel.L3) {
      targetPos = new double[] {0, 0, 0};
    } else if (level == ScoringLevel.L4) {
      targetPos = new double[] {0, 0, 0};
    } else if (level == ScoringLevel.TOP_REMOVE) {
      targetPos = new double[] {0, 0, 0};
    } else if (level == ScoringLevel.BOTTOM_REMOVE) {
      targetPos = new double[] {0, 0, 0};
    } else if (level == ScoringLevel.NET) {
      targetPos = new double[] {0, 0, 0};
      // Neutral
    } else if (level == ScoringLevel.NEUTRAL) {
      targetPos = new double[] {0, 0, 0};
    }
  }
}
