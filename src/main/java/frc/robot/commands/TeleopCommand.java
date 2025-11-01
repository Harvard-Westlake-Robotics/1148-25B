package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
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
                RobotContainer.coralIntakeCommand.velocity = MetersPerSecond.of(6);
              });

  @Override
  public void initialize() {
    updateCommands();
    if (CoralIntake.getInstance().hasCoralHotDog()) {
      reefCommand.addRequirements(getRequirements());
      sourceCommand.cancel();
      reefCommand.schedule();
    } else {
      RobotContainer.coralIntakeCommand.velocity = MetersPerSecond.of(6);
      sourceCommand.addRequirements(getRequirements());
      reefCommand.cancel();
      sourceCommand.schedule();
    }
  }

  @Override
  public void end(boolean interrupted) {
    sourceCommand.cancel();
    reefCommand.cancel();
  }

  public void updateCommands() {
    reefCommand =
        AutoBuilder.pathfindThenFollowPath(
                NetworkCommunicator.getInstance().getSelectedReefPath(),
                DriveConstants.PP_CONSTRAINTS)
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
                  RobotContainer.coralIntakeCommand.velocity = MetersPerSecond.of(6);
                });
  }
}
