package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.AlgaeIntake;
import frc.robot.subsystems.intake.IntakeConstants;

public class AlgaeIntakeCommand extends Command {
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
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public void intake() {
    if (!algaeLastStop) {
      this.velocity = LinearVelocity.ofBaseUnits(IntakeConstants.AlgaeIntake.intakeVelocity, MetersPerSecond);
    } else {
      this.velocity = LinearVelocity.ofBaseUnits(IntakeConstants.AlgaeIntake.outtakeVelocity, MetersPerSecond);
    }
  }

  public void stop() {
    if (AlgaeIntake.getInstance().hasAlgae()) {
      this.velocity = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
      AlgaeIntake.getInstance().runVoltage(1.5);
      algaeLastStop = true;
    } else {
      this.velocity = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
      algaeLastStop = false;
    }
  }

  public void setVelocity(LinearVelocity velocity) {
    this.velocity = velocity;
  }
}
