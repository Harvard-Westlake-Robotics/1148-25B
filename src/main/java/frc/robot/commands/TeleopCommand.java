package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmCommand.ScoringLevel;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.NetworkCommunicator;
import frc.robot.subsystems.intake.CoralIntake;

public class TeleopCommand extends Command {

  public TeleopCommand() {
    this.addRequirements(Drive.getInstance());
  }

  private Command reefCommand =
      AutoBuilder.pathfindThenFollowPath(
              NetworkCommunicator.getInstance().getSelectedReefPath(),
              DriveConstants.PP_CONSTRAINTS)
          .andThen(
              new AutoScoreCommand(
                  NetworkCommunicator.getInstance().getSelectedHeight(),
                  NetworkCommunicator.getInstance().getSelectedReefPath()));
  private Command sourceCommand =
      AutoBuilder.pathfindThenFollowPath(
              NetworkCommunicator.getInstance().getSelectedSourcePath(),
              DriveConstants.PP_CONSTRAINTS)
          .andThen(
              () -> {
                RobotContainer.coralIntakeCommand.manual = true;
                RobotContainer.coralIntakeCommand.velocity =
                    LinearVelocity.ofBaseUnits(6, MetersPerSecond);
              });

  @Override
  public void initialize() {
    updateCommands();
    if (CoralIntake.getInstance().hasCoralHotDog()) {
      reefCommand.addRequirements(getRequirements());
      sourceCommand.cancel();
      reefCommand.schedule();
    } else {
      RobotContainer.coralIntakeCommand.velocity = LinearVelocity.ofBaseUnits(6, MetersPerSecond);
      sourceCommand.addRequirements(getRequirements());
      reefCommand.cancel();
      sourceCommand.schedule();
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.coralIntakeCommand.manual = false;
    sourceCommand.cancel();
    reefCommand.cancel();
  }

  public void updateCommands() {
    reefCommand =
        new ParallelCommandGroup(
                AutoBuilder.pathfindThenFollowPath(
                    NetworkCommunicator.getInstance().getSelectedReefPath(),
                    DriveConstants.PP_CONSTRAINTS),
                new InstantCommand(() -> RobotContainer.armCommand.setHeight(ScoringLevel.L1)))
            .andThen(
                new AutoScoreCommand(
                    NetworkCommunicator.getInstance().getSelectedHeight(),
                    NetworkCommunicator.getInstance().getSelectedReefPath()));
    sourceCommand =
        AutoBuilder.pathfindThenFollowPath(
                NetworkCommunicator.getInstance().getSelectedSourcePath(),
                DriveConstants.PP_CONSTRAINTS)
            .andThen(
                () -> {
                  RobotContainer.coralIntakeCommand.velocity =
                      LinearVelocity.ofBaseUnits(6, MetersPerSecond);
                });
  }
}
