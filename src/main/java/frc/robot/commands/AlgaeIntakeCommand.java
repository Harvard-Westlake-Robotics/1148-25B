package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.AlgaeIntake;

public class AlgaeIntakeCommand extends Command {
  // Run velocity
  private LinearVelocity velocity;
  private boolean algaeLastStop;

  public AlgaeIntakeCommand() {
    addRequirements(AlgaeIntake.getInstance());
    this.algaeLastStop = false;
  }

  @Override
  public void initialize() {
    this.velocity = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
  }

  @Override
  public void execute() {
    AlgaeIntake.getInstance().setVelocity(velocity);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  public void intake() {
    if (!algaeLastStop) {
      this.velocity =
          LinearVelocity.ofBaseUnits(IntakeConstants.AlgaeIntake.intakeVelocity, MetersPerSecond);
    } else {
      this.velocity =
          LinearVelocity.ofBaseUnits(IntakeConstants.AlgaeIntake.outtakeVelocity, MetersPerSecond);
    }
  }

  public void stop() {
    if (AlgaeIntake.getInstance().hasAlgae()) {
      this.velocity = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
      // TODO: Why is this here?
      AlgaeIntake.getInstance().runVoltage(1.5);
      algaeLastStop = true;
    } else {
      this.velocity = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
      AlgaeIntake.getInstance().runVoltage(0);
      algaeLastStop = false;
    }
  }

  public void setVelocity(double velocity) {
    LinearVelocity v = LinearVelocity.ofBaseUnits(velocity, MetersPerSecond);
    this.velocity = v;
  }
}
