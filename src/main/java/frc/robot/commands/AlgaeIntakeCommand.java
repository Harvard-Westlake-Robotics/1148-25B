package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.AlgaeIntake;
import frc.robot.subsystems.intake.CoralIntake;
import org.littletonrobotics.junction.Logger;

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
    Logger.recordOutput("Algae Command State/Velocity", velocity);
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

  public void intakeGround() {
    this.velocity = IntakeConstants.AlgaeIntake.hamburgerIntakeVelocity;
  }

  public void outtakeGround() {
    this.velocity = IntakeConstants.AlgaeIntake.hamburgerOuttakeVelocity;
  }

  public void stop() {
    if (AlgaeIntake.getInstance().hasAlgae()) {
      this.velocity = MetersPerSecond.of(500);
    } else if (CoralIntake.getInstance().hasCoralBurger()) {
      this.velocity = MetersPerSecond.of(-500);
    } else {
      this.velocity = MetersPerSecond.of(0);
    }
  }

  public void runVelocity(double velocity) {
    this.velocity = MetersPerSecond.of(velocity);
  }
}
