package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmCommand.ScoringLevel;
import frc.robot.constants.HangConstants;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.intake.CoralIntake;

public class HangCommand extends Command {
  private LinearVelocity velocity;

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
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  public void deploy() {
    CoralIntake.getInstance().yeah();
    RobotContainer.armCommand.setHeight(ScoringLevel.HANG_DEPLOY);
    this.velocity = HangConstants.hangVelocity;
  }

  public void climb() {
    Hang.getInstance().servo.setPosition(10);
    this.velocity = MetersPerSecond.of(0);
    RobotContainer.armCommand.setHeight(ScoringLevel.HANG_CLIMB);
  }
}
