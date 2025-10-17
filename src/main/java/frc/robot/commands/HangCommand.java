package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.HangConstants;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.wrists.Pivot;

public class HangCommand extends Command {
  private LinearVelocity velocity;

  public HangCommand() {
    addRequirements(Hang.getInstance());
    addRequirements(Pivot.getInstance());
  }

  @Override
  public void initialize() {
    this.velocity = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
  }

  @Override
  public void execute() {
    Hang.getInstance().setVelocity(velocity);
    if (Hang.getInstance().hasBar()) {
      Hang.getInstance().servo.setAngle(160);
      stop();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  public void run() {
    this.velocity = LinearVelocity.ofBaseUnits(HangConstants.hangVelocity, MetersPerSecond);
  }

  public void stop() {
    this.velocity = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
  }
}
