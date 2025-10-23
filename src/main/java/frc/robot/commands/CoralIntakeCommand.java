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
  public Boolean manual = true;
  public LinearVelocity velocity;

  public Boolean isIntakingHotdog() {
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
    // Change this later
    this.manual = true;
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
          outtake();
        }
      } else {
        CoralIntake.getInstance().runVelocity(MetersPerSecond.of(0));
      }
    } else {
      CoralIntake.getInstance().runVelocity(velocity);
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
        CoralIntake.getInstance().runVelocity(IntakeConstants.CoralIntake.intakeVelocity);
      }
    } else {
      // Intake hamburger --> shift when sensors 1 and 3 are triggered BUT NOT WHEN
      // BOTH
      // Sensor 2 should be trigerred if sensors 1 and 3 are both triggered
      AlgaeIntake.getInstance().runVelocity(IntakeConstants.CoralIntake.hamburgerIntakeVelocity);
      if ((CoralIntake.getInstance().getSensor1() || CoralIntake.getInstance().getSensor3())
          && CoralIntake.getInstance().getSensor2()) {
        // We're good, keep intaking
        CoralIntake.getInstance().runVelocity(IntakeConstants.CoralIntake.intakeVelocity);
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
    CoralIntake.getInstance().runVelocity(IntakeConstants.CoralIntake.outtakeVelocity);
  }

  @Override
  public void end(boolean interrupted) {
    CoralIntake.getInstance().runVelocity(MetersPerSecond.of(0));
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
  public boolean isFinished() {
    return false;
  }
}
