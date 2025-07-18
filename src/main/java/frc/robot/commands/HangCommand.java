package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.hang.Hang;

public class HangCommand extends Command {
  private LinearVelocity velocity;

  public HangCommand() {
    addRequirements(Hang.getInstance());
  }

  @Override
  public void initialize() {
    this.velocity = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
  }

  @Override
  public void execute() {
    Hang.getInstance().setVelocity(velocity);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  public void run() {
    this.velocity = LinearVelocity.ofBaseUnits(Constants.Hang.hangVelocity, MetersPerSecond);
  }

  public void stop() {
    this.velocity = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
  }
}
