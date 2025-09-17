package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.CoralIntake;
import frc.robot.subsystems.intake.IntakeConstants;

public class CoralIntakeCommand extends Command {
  private Boolean eject;
  private LinearVelocity velocity;
  public Boolean getEject() {
    return eject;
  }

  public void setEject(Boolean eject) {
    this.eject = eject;
  }

  public CoralIntakeCommand() {
    addRequirements(CoralIntake.getInstance());
    this.eject = false;
  }

  @Override
  public void initialize() {
    CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(4, MetersPerSecond));
  }

  @Override
  public void execute() {
    //Quinn I'm killing your code- Daniel
    // if (eject) {
    //   CoralIntake.getInstance()
    //       .setVelocity(
    //           LinearVelocity.ofBaseUnits(
    //               IntakeConstants.CoralIntake.outtakeVelocity, MetersPerSecond));
    // } else {
    //   // If Coral is in the "Jammed" position- covering all three top sensors
    //   if (CoralIntake.getInstance().getSensor1()
    //       && CoralIntake.getInstance().getSensor2()
    //       && CoralIntake.getInstance().getSensor3()) {
    //     CoralIntake.getInstance().setVelocityShift(LinearVelocity.ofBaseUnits(6, MetersPerSecond));
    //     // If Coral is in intake-ready position
    //   } else if ((CoralIntake.getInstance().getSensor1() || CoralIntake.getInstance().getSensor3())
    //       && CoralIntake.getInstance().getSensor2()) {
    //     CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(8, MetersPerSecond));
    //     // Deafult intake velocity
    //   } else {
    //     CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(4, MetersPerSecond));
    //   }
    // }
    driver.R1().whileTrue(new InstantCommand())
  }

  @Override
  public void end(boolean interrupted) {
    CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
  }

  @Override
  public boolean isFinished() {
    if (eject) {
      // returns when the Coral has been scored
      return !CoralIntake.getInstance().getSensor4();
    } else {
      // returns when the Coral is in position
      return CoralIntake.getInstance().hasCoral();
    }
  }
}
