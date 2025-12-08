package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.NetworkCommunicator;

public class TeleopCommand extends Command {

  public TeleopCommand() {
    this.addRequirements(Drive.getInstance());
  }

  private Command reefCommand =
      AutoBuilder.pathfindThenFollowPath(
              NetworkCommunicator.getInstance().getSelectedReefPath(),
              DriveConstants.PP_CONSTRAINTS)
          .andThen(new AutoScoreCommand(NetworkCommunicator.getInstance().getSelectedReefPath()));
  private Command sourceCommand =
      AutoBuilder.pathfindThenFollowPath(
          NetworkCommunicator.getInstance().getSelectedSourcePath(), DriveConstants.PP_CONSTRAINTS);

  @Override
  public void initialize() {
    updateCommands();
    reefCommand.addRequirements(getRequirements());
    sourceCommand.cancel();
    reefCommand.schedule();
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
            .andThen(new AutoScoreCommand(NetworkCommunicator.getInstance().getSelectedReefPath()));

    sourceCommand =
        AutoBuilder.pathfindThenFollowPath(
            NetworkCommunicator.getInstance().getSelectedSourcePath(),
            DriveConstants.PP_CONSTRAINTS);
  }
}
