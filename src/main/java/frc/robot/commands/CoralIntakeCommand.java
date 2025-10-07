package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArmCommand.ScoringLevel;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.AlgaeIntake;
import frc.robot.subsystems.intake.CoralIntake;

public class CoralIntakeCommand extends Command {
  /*
   * Notes for intake
   * Two styles of intaking: Intake straight and intake wide
   * At Coral source, intake hotdog ALWAYS
   * when intaking on ground, keep two choices:
   * 1. Intake straight
   * 2. Intake wide
   *
   * Different shift modes for the two choices
   */
  // intake straight or intake wide
  private Boolean intakingHotdog;
  private boolean outtaking;
  // boolean on if we're running the intake
  private boolean running;

  // For ignoring intake logic
  public Boolean manual;
  public LinearVelocity velocity;

  public Boolean getIntakingHotdog() {
    return intakingHotdog;
  }

  public void setIntakingHotdog(Boolean intakingHotdog) {
    this.intakingHotdog = intakingHotdog;
  }

  public CoralIntakeCommand() {
    addRequirements(CoralIntake.getInstance());
    this.intakingHotdog = true;
    this.outtaking = false;
    this.running = false;
    velocity = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (!manual) {
      // Code for automatic intaking
      if (ArmCommand.level == ScoringLevel.SOURCE_CORAL) {
        // If we're at the source, always intake straight
        intakingHotdog = true;
        running = true;
        outtaking = false;
      }
      if (running) {
        if (!outtaking) {
          intake();
        } else {
          // We outtake
          outtake();
        }
      } else {
        CoralIntake.getInstance().setVelocityMPS(0);
      }
    } else {
      CoralIntake.getInstance().setVelocity(velocity);
    }
  }

  // Runs the intake based on whether we are intaking straight or hot dog
  // Sensor requirements may need to change based on how the intake actually runs
  public void intake() {
    if (intakingHotdog) {
      // Intake straight --> Shift when sensors 1, 2, 3 are triggered
      if (CoralIntake.getInstance().getSensor1()
          && CoralIntake.getInstance().getSensor2()
          && CoralIntake.getInstance().getSensor3()) {
        // Coral stuck, shift
        // Doesn't matter which way we shift since the goal is just to put it in the middle
        CoralIntake.getInstance().shift(true, IntakeConstants.CoralIntake.shiftVelocity);
      } else {
        // We're good to keep intaking
        CoralIntake.getInstance().setVelocityMPS(IntakeConstants.CoralIntake.intakeVelocity);
      }
    } else {
      // Intake hamburger --> shift when sensors 1 and 3 are triggered BUT NOT WHEN
      // BOTH
      // Sensor 2 should be trigerred if sensors 1 and 3 are both triggered
      AlgaeIntake.getInstance().setVelocityMPS(4);
      if ((CoralIntake.getInstance().getSensor1() || CoralIntake.getInstance().getSensor3())
          && CoralIntake.getInstance().getSensor2()) {
        // We're good, keep intaking
        CoralIntake.getInstance().setVelocityMPS(IntakeConstants.CoralIntake.intakeVelocity);
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
  }

  public void outtake() {
    CoralIntake.getInstance().setVelocityMPS(IntakeConstants.CoralIntake.outtakeVelocity);
  }

  @Override
  public void end(boolean interrupted) {
    CoralIntake.getInstance().setVelocityMPS(0);
  }

  public void runIntake(boolean intakingStraight) {
    running = true;
    outtaking = false;
    this.intakingHotdog = intakingStraight;
  }

  public void runOuttake() {
    running = true;
    outtaking = true;
  }

  public void stopIntake() {
    running = false;
    AlgaeIntake.getInstance().setVelocityMPS(0);
  }

  @Override
  public boolean isFinished() {
    // Deprecated code but idk if we want this in or not
    // if (intakingStraight) {
    // // returns when the Coral has been scored
    // return !CoralIntake.getInstance().getSensor4();
    // } else {
    // // returns when the Coral is in position
    // return CoralIntake.getInstance().hasCoral();
    // }

    return false;
  }
}
