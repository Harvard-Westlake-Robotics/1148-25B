package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.CoralIntake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.commands.ElevatorCommand.ScoringLevel;

public class CoralIntakeCommand extends Command {
  /*
   * Notes for intake
   * Two styles of intaking: Intake hot dog and intake straight
   * At Coral source, intake straight ALWAYS
   * when intaking on ground, keep two choices:
   * 1. Intake hot dog
   * 2. Intake straight
   * 
   * Different shift modes for the two choices
   */
  private LinearVelocity velocity;
  // intake straight or intake hot dog
  private Boolean intakingStraight;

  public void setVelocity(LinearVelocity velocity) {
    this.velocity = velocity;
  }

  public Boolean getIntakingStraight() {
    return intakingStraight;
  }

  public void setIntakingStraight(Boolean intakingStraight) {
    this.intakingStraight = intakingStraight;
  }

  public CoralIntakeCommand() {
    addRequirements(CoralIntake.getInstance());
    this.intakingStraight = true;
  }

  @Override
  public void initialize() {
    CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(4, MetersPerSecond));
  }

  @Override
  public void execute() {

    // Code if we want the intake to run automatically
    // Dont kill quinns hard work please
    // if (eject) {
    // CoralIntake.getInstance()
    // .setVelocity(
    // LinearVelocity.ofBaseUnits(
    // IntakeConstants.CoralIntake.outtakeVelocity, MetersPerSecond));
    // } else {
    // // If Coral is in the "Jammed" position- covering all three top sensors
    // if (CoralIntake.getInstance().getSensor1()
    // && CoralIntake.getInstance().getSensor2()
    // && CoralIntake.getInstance().getSensor3()) {
    // CoralIntake.getInstance().setVelocityShift(LinearVelocity.ofBaseUnits(6,
    // MetersPerSecond));
    // // If Coral is in intake-ready position
    // } else if ((CoralIntake.getInstance().getSensor1() ||
    // CoralIntake.getInstance().getSensor3())
    // && CoralIntake.getInstance().getSensor2()) {
    // CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(8,
    // MetersPerSecond));
    // // Deafult intake velocity
    // } else {
    // CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(4,
    // MetersPerSecond));
    // }
    // }

    // Code for manual intaking
    if (ElevatorCommand.level == ScoringLevel.SOURCE_CORAL) {
      intakingStraight = true;
      intake();
    } else {
    }
  }

  public void intake() {
    if (intakingStraight) {
      // Intake straight --> Shift when sensors 1, 2, 3 are triggered
      if (CoralIntake.getInstance().getSensor1() && CoralIntake.getInstance().getSensor2()
          && CoralIntake.getInstance().getSensor3()) {
        CoralIntake.getInstance().setVelocityShift(LinearVelocity.ofBaseUnits(6, MetersPerSecond));
      } else {
        CoralIntake.getInstance()
            .setVelocity(LinearVelocity.ofBaseUnits(IntakeConstants.CoralIntake.intakeVelocity, MetersPerSecond));
      }
    } else {
      // Intake hotdog --> shift when sensors 1 and 3 are triggered BUT NOT WHEN BOTH
      if ((CoralIntake.getInstance().getSensor1() || CoralIntake.getInstance().getSensor3())
          && CoralIntake.getInstance().getSensor2()) {
        // We're good, keep intaking
      } else {
        // Bad things, gotta shift depending on situation
        if (CoralIntake.getInstance().getSensor1()) {
          // Shift right
        } else if (CoralIntake.getInstance().getSensor3()) {
          // Shift left
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
  }

  @Override
  public boolean isFinished() {
    if (intakingStraight) {
      // returns when the Coral has been scored
      return !CoralIntake.getInstance().getSensor4();
    } else {
      // returns when the Coral is in position
      return CoralIntake.getInstance().hasCoral();
    }
  }
}
