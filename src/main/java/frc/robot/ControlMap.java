package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmCommand.ScoringLevel;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.drive.Drive;

public class ControlMap {
  private static ControlMap instance;

  public static ControlMap getInstance() {
    if (instance == null) {
      instance = new ControlMap();
    }
    return instance;
  }

  private ControlMap() {
  }

  public void configurePreset1(CommandXboxController operator, CommandPS5Controller driver) {
    // Reset gyro to 0° when B button is pressed
    operator
        .back()
        .onTrue(
            Commands.runOnce(
                () -> Drive.getInstance()
                    .setPose(
                        new Pose2d(
                            Drive.getInstance().getPose().getTranslation(),
                            new Rotation2d())),
                Drive.getInstance())
                .ignoringDisable(true));

    // Intake commands

    // Coral Intake
    // Coral intake is intaking automatically when elevator at coral source
    driver
        .R1()
        .whileTrue(
            new InstantCommand(
                () -> {
                  if (ArmCommand.level == ScoringLevel.GROUND_ALGAE) {
                    // If at algae level, intake algae
                    RobotContainer.algaeIntakeCommand.setVelocity(
                        LinearVelocity.ofBaseUnits(
                            IntakeConstants.AlgaeIntake.intakeVelocity, MetersPerSecond));
                  } else {
                    // Intake straight
                    RobotContainer.coralIntakeCommand.runIntake(true);
                  }
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  if (ArmCommand.level == ScoringLevel.GROUND_ALGAE) {
                    // If at algae level, intake algae
                    RobotContainer.algaeIntakeCommand.setVelocity(
                        LinearVelocity.ofBaseUnits(0, MetersPerSecond));
                  } else {
                    // Intake straight
                    RobotContainer.coralIntakeCommand.stopIntake();
                  }
                }));
    driver
        .L1()
        .whileTrue(
            new InstantCommand(
                () -> {
                  if (ArmCommand.level == ScoringLevel.GROUND_ALGAE) {
                    // If at algae level, intake algae
                    RobotContainer.algaeIntakeCommand.setVelocity(
                        LinearVelocity.ofBaseUnits(
                            IntakeConstants.AlgaeIntake.intakeVelocity, MetersPerSecond));
                  } else {
                    // Intake straight
                    RobotContainer.coralIntakeCommand.runIntake(false);
                  }
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  if (ArmCommand.level == ScoringLevel.GROUND_ALGAE) {
                    // If at algae level, intake algae
                    RobotContainer.algaeIntakeCommand.setVelocity(
                        LinearVelocity.ofBaseUnits(0, MetersPerSecond));
                  } else {
                    // Intake straight
                    RobotContainer.coralIntakeCommand.stopIntake();
                  }
                }));
    // Shared Outtake
    driver
        .R2()
        .whileTrue(
            new InstantCommand(
                () -> {
                  if (RobotContainer.elevatorCommand.outtakePosition) {
                    RobotContainer.coralIntakeCommand.runOuttake();
                  } else if (ArmCommand.level == ScoringLevel.GROUND_ALGAE) {
                    RobotContainer.algaeIntakeCommand.setVelocity(
                        LinearVelocity.ofBaseUnits(
                            IntakeConstants.AlgaeIntake.outtakeVelocity, MetersPerSecond));
                  }
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  RobotContainer.coralIntakeCommand.stopIntake();
                  RobotContainer.algaeIntakeCommand.setVelocity(
                      LinearVelocity.ofBaseUnits(0, MetersPerSecond));
                }));

    // Elevator
    operator
        .a()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand.setHeight(ScoringLevel.SOURCE_CORAL);
                  RobotContainer.algaeIntakeCommand.setVelocity(
                      LinearVelocity.ofBaseUnits(0, MetersPerSecond));
                }));
    operator
        .x()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand.setHeight(ScoringLevel.GROUND_CORAL);
                  RobotContainer.algaeIntakeCommand.setVelocity(
                      LinearVelocity.ofBaseUnits(0, MetersPerSecond));
                }));
    operator
        .y()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand.setHeight(ScoringLevel.NET);
                  RobotContainer.algaeIntakeCommand.setVelocity(
                      LinearVelocity.ofBaseUnits(0, MetersPerSecond));
                  RobotContainer.coralIntakeCommand.stopIntake();
                }));
    operator
        .b()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand.setHeight(ScoringLevel.PROCESSOR);
                  RobotContainer.algaeIntakeCommand.setVelocity(
                      LinearVelocity.ofBaseUnits(0, MetersPerSecond));
                  RobotContainer.coralIntakeCommand.stopIntake();
                }));
    operator
        .leftBumper()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand.setHeight(ScoringLevel.L2);
                  RobotContainer.algaeIntakeCommand.setVelocity(
                      LinearVelocity.ofBaseUnits(0, MetersPerSecond));
                }));
    operator
        .rightBumper()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand.setHeight(ScoringLevel.L3);
                  RobotContainer.algaeIntakeCommand.setVelocity(
                      LinearVelocity.ofBaseUnits(0, MetersPerSecond));
                }));
    operator
        .rightTrigger()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand.setHeight(ScoringLevel.L4);
                  RobotContainer.algaeIntakeCommand.setVelocity(
                      LinearVelocity.ofBaseUnits(0, MetersPerSecond));
                }));
    operator
        .povUp()
        .whileTrue(
            new InstantCommand(
                () -> RobotContainer.elevatorCommand.setHeight(ScoringLevel.BOTTOM_REMOVE)));
    operator
        .povDown()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand.setHeight(ScoringLevel.GROUND_ALGAE);
                  RobotContainer.coralIntakeCommand.stopIntake();
                }));
    operator
        .povLeft()
        .whileTrue(
            new InstantCommand(
                () -> RobotContainer.elevatorCommand.setHeight(ScoringLevel.TOP_REMOVE)));
    RobotContainer.algaeIntakeCommand.setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
    RobotContainer.coralIntakeCommand.stopIntake();
    operator
        .povRight()
        .whileTrue(
            new InstantCommand(() -> RobotContainer.elevatorCommand.setHeight(ScoringLevel.HANG)));
    RobotContainer.algaeIntakeCommand.setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
    RobotContainer.coralIntakeCommand.stopIntake();

    // Commands not done: Hang, auto-align, whatever the "rest position" thing is on
    // the document

  }
}
