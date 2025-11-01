package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.AlgaeIntake;
import frc.robot.subsystems.intake.CoralIntake;

public class CoralIntakeCommand extends Command {
  /*
   * Notes for intake
   * Two styles of intaking: Intake straight and intake wide
   * At Coral source, intake hotdog ALWAYS
   * when intaking on ground, keep two choices:
   * 1. Intake straight (hotdog)
   * 2. Intake wide (burger)
   *
   * Different shift modes for the two choices
   */

  public LinearVelocity velocity;

  public CoralIntakeCommand() {
    addRequirements(CoralIntake.getInstance());
    velocity = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    CoralIntake.getInstance().runVelocity(velocity);
  }

  // Runs the intake based on whether we are intaking straight or hot dog
  // Sensor requirements may need to change based on how the intake actually runs
  public void intakeSource() {
    // Intake straight --> Shift when sensors 1, 2, 3 are triggered
    if (CoralIntake.getInstance().getSensor1()
        && CoralIntake.getInstance().getSensor2()
        && CoralIntake.getInstance().getSensor3()) {
      // Coral stuck, shift
      // Doesn't matter which way we shift since the goal is just to put it in the middle
      CoralIntake.getInstance().shift(true, IntakeConstants.CoralIntake.shiftVelocity);
    } else {
      // We're good to keep intaking
      velocity = IntakeConstants.CoralIntake.intakeVelocity;
    }
  }

  public void intakeGround() {
    // Intake hamburger --> shift when sensors 1 and 3 are triggered BUT NOT WHEN
    // BOTH
    // Sensor 2 should be trigerred if sensors 1 and 3 are both triggered
    AlgaeIntake.getInstance().runVelocity(IntakeConstants.CoralIntake.hamburgerIntakeVelocity);
    if ((CoralIntake.getInstance().getSensor1() || CoralIntake.getInstance().getSensor3())
        && CoralIntake.getInstance().getSensor2()) {
      // We're good, keep intaking
      velocity = IntakeConstants.CoralIntake.intakeVelocity;
    } else {
      // Bad things, gotta shift depending on situation
      if (CoralIntake.getInstance().getSensor1()) {
        CoralIntake.getInstance().shift(true, IntakeConstants.CoralIntake.shiftVelocity);
      } else if (CoralIntake.getInstance().getSensor3()) {
        // Shift left
        CoralIntake.getInstance().shift(false, IntakeConstants.CoralIntake.shiftVelocity);
      }
    }
  }

  public void outtakeReef() {
    velocity = IntakeConstants.CoralIntake.outtakeVelocity;
  }

  public void outtakeGround() {
    velocity = IntakeConstants.CoralIntake.hamburgerOuttakeVelocity;
  }

  public void stop() {
    this.velocity = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
    AlgaeIntake.getInstance().runVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
  }

  public void manualIntake() {
    this.velocity = IntakeConstants.CoralIntake.intakeVelocity;
  }

  public void manualOuttake() {
    this.velocity = IntakeConstants.CoralIntake.outtakeVelocity;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
