package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.ScoreCommand.ScoringLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.NetworkCommunicator;
import frc.robot.subsystems.intake.CoralIntake;

public class TeleopCommand extends Command {

  public TeleopCommand() {
    // this.addRequirements(Elevator.getInstance(), Drive.getInstance(),
    // CoralIntake.getInstance());
  }

  private Command reefCommand =
      AutoBuilder.pathfindThenFollowPath(
              NetworkCommunicator.getInstance().getSelectedReefPath(), Drive.PP_CONSTRAINTS)
          .andThen(
              new AutoScoreCommand(
                  NetworkCommunicator.getInstance().getSelectedHeight(),
                  NetworkCommunicator.getInstance().getSelectedReefPath()));
  private Command sourceCommand =
      AutoBuilder.pathfindThenFollowPath(
              NetworkCommunicator.getInstance().getSelectedSourcePath(), Drive.PP_CONSTRAINTS)
          .andThen(new CoralIntakeCommand(6));

  @Override
  public void initialize() {
    updateCommands();
    if (CoralIntake.getInstance().hasCoral()) {
      reefCommand.addRequirements(getRequirements());
      sourceCommand.cancel();
      reefCommand.schedule();
    } else {
      RobotContainer.coralIntakeCommand.setVelocity(LinearVelocity.ofBaseUnits(6, MetersPerSecond));
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
        new ParallelCommandGroup(
                AutoBuilder.pathfindThenFollowPath(
                    NetworkCommunicator.getInstance().getSelectedReefPath(), Drive.PP_CONSTRAINTS),
                new RaiseElevatorCommand(ScoringLevel.L1))
            .andThen(
                new AutoScoreCommand(
                    NetworkCommunicator.getInstance().getSelectedHeight(),
                    NetworkCommunicator.getInstance().getSelectedReefPath()));
    sourceCommand =
        AutoBuilder.pathfindThenFollowPath(
                NetworkCommunicator.getInstance().getSelectedSourcePath(), Drive.PP_CONSTRAINTS)
            .andThen(new CoralIntakeCommand(6));
  }
}
