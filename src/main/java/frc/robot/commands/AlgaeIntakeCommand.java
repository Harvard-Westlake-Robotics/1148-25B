package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.AlgaeIntake;

public class AlgaeIntakeCommand extends Command {
  // Run velocity
  private LinearVelocity velocity;

  public AlgaeIntakeCommand() {
    addRequirements(AlgaeIntake.getInstance());
  }

  @Override
  public void initialize() {
    this.velocity = MetersPerSecond.of(0);
  }

  @Override
  public void execute() {
    AlgaeIntake.getInstance().runVelocity(velocity);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  public void intake() {
    this.velocity = IntakeConstants.AlgaeIntake.intakeVelocity;
  }

  public void outtake() {
    this.velocity = IntakeConstants.AlgaeIntake.outtakeVelocity;
  }

  public void stop() {
    if (AlgaeIntake.getInstance().hasAlgae()) {
      this.velocity = MetersPerSecond.of(0);
      AlgaeIntake.getInstance().runVoltage(IntakeConstants.AlgaeIntake.algaeHoldVoltage);
    } else {
      this.velocity = MetersPerSecond.of(0);
      AlgaeIntake.getInstance().runVoltage(0);
    }
  }

  public void runVelocity(double velocity) {
    this.velocity = MetersPerSecond.of(velocity);
  }
}
