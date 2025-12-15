package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer.Mode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.NetworkCommunicator;

public class ControlMap {
  private static ControlMap instance;

  public static ControlMap getInstance() {
    if (instance == null) {
      instance = new ControlMap();
    }
    return instance;
  }

  private ControlMap() {}

  public void configurePreset1(CommandXboxController operator, CommandPS5Controller driver) {
    // Reset gyro to 0° when both center buttons are pressed
    driver
        .circle()
        .onTrue(
            Commands.runOnce(
                    RobotContainer.currentMode == Mode.SIM
                        ? () ->
                            // simulation
                            Drive.getInstance()
                                .setPose(
                                    RobotContainer.driveSimulation.getSimulatedDriveTrainPose())
                        : () ->
                            // real/test
                            Drive.getInstance()
                                .setPose(
                                    new Pose2d(
                                        Drive.getInstance().getPose().getTranslation(),
                                        new Rotation2d())),
                    Drive.getInstance())
                .ignoringDisable(true));

    driver
        .L2()
        .whileTrue(
            new Command() {
              @Override
              public void initialize() {
                // this.addRequirements(
                // CoralIntake.getInstance(), Drive.getInstance(), Elevator.getInstance());
                if (Drive.getInstance().getCurrentCommand() != null) {
                  Drive.getInstance().getCurrentCommand().cancel();
                }
                NetworkCommunicator.getInstance().getTeleopCommand().updateCommands();
                NetworkCommunicator.getInstance().getTeleopCommand().schedule();
              }

              @Override
              public void end(boolean interrupted) {
                NetworkCommunicator.getInstance().getTeleopCommand().cancel();
                Drive.getInstance().stop();
                if (Drive.getInstance().getCurrentCommand() != null) {
                  Drive.getInstance().getCurrentCommand().cancel();
                }
              }
            });

    // ORCHESTRA CRAP II
    operator
        .up()
        .OnTrue(
            new InstantCommand(
                () -> {
                  if (RobotContainer.isPlaying) {
                    if (RobotContainer.songPlaying == RobotContainer.songSelected) {
                      SmartDashboard.PutString("DB/String 0", "Paused: " + RobotContainer.allSongs.at(RobotContainer.songSelected));
                      RobotContainer.orchestra.pause();
                      RobotContainer.isPlaying = false;
                    } else {
                      RobotContainer.orchestra.stop();
                      SmartDashboard.PutString("DB/String 0", "Playing: " + RobotContainer.allSongs.at(RobotContainer.songSelected));
                      var status = RobotContainer.orchestra.loadMusic(RobotContainer.allSongs[RobotContainer.songSelected]);
                      RobotContainer.songPlaying = RobotContainer.songSelected;
                      RobotContainer.orchestra.play();
                    }
                  } else {
                    if (RobotContainer.songPlaying != RobotContainer.songSelected) {
                      var status = RobotContainer.orchestra.loadMusic(RobotContainer.allSongs[RobotContainer.songSelected]);
                      RobotContainer.songPlaying = RobotContainer.songSelected;
                    }
                    SmartDashboard.PutString("DB/String 0", "Playing: " + RobotContainer.allSongs.at(RobotContainer.songSelected));
                    RobotContainer.isPlaying = true;
                    RobotContainer.orchestra.play();
                  }
                }));
    operator
        .down()
        .OnTrue(
            new InstantCommand(
                () -> {
                  if (RobotContainer.isPlaying) {
                    RobotContainer.orchestra.stop();
                    RobotContainer.isPlaying = false;
                  }
                  SmartDashboard.PutString("DB/String 0", "Playing: ");
                  RobotContainer.songPlaying = -1;
                }));
    operator
        .left()
        .OnTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.songSelected = (RobotContainer.songSelected - 1 + RobotContainer.allSongs.length) % RobotContainer.allSongs.length;
                  SmartDashboard.PutString("DB/String 1", "Selected: " + RobotContainer.allSongs.at(RobotContainer.songSelected));
                }));
    operator
        .right()
        .OnTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.songSelected = (RobotContainer.songSelected + 1) % RobotContainer.allSongs.length;
                  SmartDashboard.PutString("DB/String 1", "Selected: " + RobotContainer.allSongs.at(RobotContainer.songSelected));
                }));
  }
}
