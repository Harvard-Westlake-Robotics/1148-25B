package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmCommand.ScoringLevel;
import frc.robot.constants.HangConstants;
import frc.robot.subsystems.hang.Hang;

public class HangCommand extends Command {
  private LinearVelocity velocity;
  private boolean lock = false;

  public HangCommand() {
    addRequirements(Hang.getInstance());
    this.velocity = MetersPerSecond.of(0);
  }

  @Override
  public void initialize() {
    this.velocity = MetersPerSecond.of(0);
  }

  @Override
  public void execute() {
    Hang.getInstance().runVelocityClosedLoop(velocity);
    if (lock) {
      if (RobotContainer.armCommand.withinTargetRange()) {
        Hang.getInstance().servo.setAngle(100);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  public void deploy() {
    RobotContainer.armCommand.setHeight(ScoringLevel.HANG_DEPLOY);
    this.velocity = HangConstants.hangVelocity;
  }

  public void climb() {
    RobotContainer.armCommand.setHeight(ScoringLevel.HANG_CLIMB);
    this.velocity = MetersPerSecond.of(0);
    lock = true;
  }

  public void lock() {
    lock = true;
  }
}
