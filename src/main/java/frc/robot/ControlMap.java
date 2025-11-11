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

    // Intake and outtake (L2 and R2)
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
        .R2()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.armCommand.setHeight(ScoringLevel.L1);
                  RobotContainer.algaeIntakeCommand.outtakeGround();
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  RobotContainer.armCommand.setHeight(ScoringLevel.NEUTRAL);
                  RobotContainer.algaeIntakeCommand.stop();
                }));

    driver
        .R1()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.armCommand.setHeight(ScoringLevel.GROUND_CORAL);
                  RobotContainer.algaeIntakeCommand.intakeGround();
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
                  RobotContainer.algaeIntakeCommand.outtake();
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  RobotContainer.algaeIntakeCommand.runVelocity(0);
                }));

    // MANUAL COMMANDS --> Operator:
    // Elevator to Neutral
    operator
        .povDown()
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
                  RobotContainer.hangCommand.deploy();
                }));

    operator
        .a()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.hangCommand.climb();
                }));

    operator
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.armCommand.setPivotAngle(75);
                }));

    operator
        .leftTrigger()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.armCommand.setHeight(ScoringLevel.PROCESSOR);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  RobotContainer.armCommand.setHeight(ScoringLevel.NEUTRAL);
                }));
    operator
        .rightBumper()
        .onTrue(new InstantCommand(() -> RobotContainer.armCommand.incrementElevator()));
    operator
        .leftBumper()
        .onTrue(new InstantCommand(() -> RobotContainer.armCommand.reduceElevator()));
  }
}
