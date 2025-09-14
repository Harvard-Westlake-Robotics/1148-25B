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
  private boolean outtaking;
  // boolean on if we're running the intake
  private boolean running;

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
    this.outtaking = false;
    this.running = false;
  }

  @Override
  public void initialize() {
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
      // If we're at the source, always intake straight
      intakingStraight = true;
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
      CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
    }
  }

  // Runs the intake based on whether we are intaking straight or hot dog
  // Sensor requirements may need to change based on how the intake actually runs
  public void intake() {
    if (intakingStraight) {
      // Intake straight --> Shift when sensors 1, 2, 3 are triggered
      if (CoralIntake.getInstance().getSensor1() && CoralIntake.getInstance().getSensor2()
          && CoralIntake.getInstance().getSensor3()) {
        // Coral stuck, shift
        CoralIntake.getInstance().shift(true,
            LinearVelocity.ofBaseUnits(IntakeConstants.CoralIntake.shiftVelocity, MetersPerSecond));
      } else {
        // We're good to keep intaking
        CoralIntake.getInstance()
            .setVelocity(LinearVelocity.ofBaseUnits(IntakeConstants.CoralIntake.intakeVelocity, MetersPerSecond));
      }
    } else {
      // Intake hotdog --> shift when sensors 1 and 3 are triggered BUT NOT WHEN BOTH
      // Sensor 2 should be trigerred if sensors 1 and 3 are both triggered
      if ((CoralIntake.getInstance().getSensor1() || CoralIntake.getInstance().getSensor3())
          && CoralIntake.getInstance().getSensor2()) {
        // We're good, keep intaking
        CoralIntake.getInstance()
            .setVelocity(LinearVelocity.ofBaseUnits(IntakeConstants.CoralIntake.intakeVelocity, MetersPerSecond));
      } else {
        // Bad things, gotta shift depending on situation
        if (CoralIntake.getInstance().getSensor1()) {
          CoralIntake.getInstance().shift(true,
              LinearVelocity.ofBaseUnits(IntakeConstants.CoralIntake.shiftVelocity, MetersPerSecond));
        } else if (CoralIntake.getInstance().getSensor3()) {
          // Shift left
          CoralIntake.getInstance().shift(false,
              LinearVelocity.ofBaseUnits(IntakeConstants.CoralIntake.shiftVelocity, MetersPerSecond));
        }
      }
    }
  }

  public void outtake() {
    CoralIntake.getInstance()
        .setVelocity(LinearVelocity.ofBaseUnits(IntakeConstants.CoralIntake.outtakeVelocity, MetersPerSecond));
  }

  @Override
  public void end(boolean interrupted) {
    CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
  }

  public void runIntake(boolean intakingStraight) {
    running = true;
    outtaking = false;
    this.intakingStraight = intakingStraight;
  }

  public void runOuttake() {
    running = true;
    outtaking = true;
  }

  public void stopIntake() {
    running = false;
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
