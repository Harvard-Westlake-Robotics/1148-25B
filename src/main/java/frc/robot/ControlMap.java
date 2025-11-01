package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer.Mode;
import frc.robot.commands.ArmCommand.ScoringLevel;
import frc.robot.subsystems.drive.Drive;

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
        .povCenter()
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

    // Intake and outtake (L2 and R2)
    // driver
    //     .R1()
    //     .whileTrue(
    //         new InstantCommand(() -> {
    //             RobotContainer.coralIntakeCommand.intakeGround();
    //             RobotContainer.armCommand.setHeight(ScoringLevel.GROUND_CORAL);
    //         }))
    //     .onFalse(
    //         new InstantCommand(() -> {
    //             RobotContainer.coralIntakeCommand.stop();
    //             RobotContainer.armCommand.setHeight(ScoringLevel.NEUTRAL);
    //         }));
    // driver
    //     .R2()
    //     .whileTrue(new InstantCommand(() -> {
    //         RobotContainer.coralIntakeCommand.manualOuttake();
    //     }))
    //     .onFalse(
    //         new InstantCommand(() -> RobotContainer.coralIntakeCommand.stop())
    //             .andThen(
    //                 new InstantCommand(
    //                     () -> RobotContainer.armCommand.setHeight(ScoringLevel.NEUTRAL))));

    driver
        .L1()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.armCommand.setHeight(ScoringLevel.GROUND_ALGAE);
                  RobotContainer.algaeIntakeCommand.intake();
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  RobotContainer.algaeIntakeCommand.stop();
                  RobotContainer.armCommand.setHeight(ScoringLevel.NEUTRAL);
                }));

    driver
        .L2()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.armCommand.setHeight(ScoringLevel.PROCESSOR);
                  if (RobotContainer.armCommand.withinTargetRange()) {
                    RobotContainer.algaeIntakeCommand.outtake();
                  }
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  RobotContainer.algaeIntakeCommand.stop();
                  RobotContainer.armCommand.setHeight(ScoringLevel.NEUTRAL);
                }));

    // MANUAL COMMANDS --> Operator:
    // Elevator to Neutral
    operator
        .a()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.armCommand.setHeight(ScoringLevel.NEUTRAL);
                }));

    operator
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.armCommand.setHeight(ScoringLevel.HANG_DEPLOY);
                }));
  }
}
