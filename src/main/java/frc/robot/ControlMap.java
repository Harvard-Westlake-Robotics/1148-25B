package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.logging.Level;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorCommand.ScoringLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeConstants;

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
    // L1 has a different intakking way --> Done on L1 button press
    driver.R1().whileTrue(new InstantCommand(
        () -> {
          RobotContainer.coralIntakeCommand
              .setVelocity(LinearVelocity.ofBaseUnits(IntakeConstants.CoralIntake.intakeVelocity, MetersPerSecond));
        })).onFalse(new InstantCommand(
            () -> {
              RobotContainer.coralIntakeCommand
                  .setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
            }));
    // Shared Outtake
    driver.cross().whileTrue(new InstantCommand(
        () -> {
          // To Do: L1 outtakes differently
          if (RobotContainer.elevatorCommand.straightIntakePosition) {
            RobotContainer.coralIntakeCommand
                .setVelocity(LinearVelocity.ofBaseUnits(IntakeConstants.CoralIntake.outtakeVelocity, MetersPerSecond));
          } else if (ElevatorCommand.level == ScoringLevel.GROUND_ALGAE) {
            RobotContainer.algaeIntakeCommand
                .setVelocity(LinearVelocity.ofBaseUnits(IntakeConstants.AlgaeIntake.outtakeVelocity, MetersPerSecond));
          }
        }));
    driver.cross().onFalse(new InstantCommand(
        () -> {
          RobotContainer.coralIntakeCommand
              .setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
          RobotContainer.algaeIntakeCommand
              .setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
        }));

    // Elevator
    operator.povDown().whileTrue(new InstantCommand(
        () -> {
          RobotContainer.elevatorCommand.setHeight(ScoringLevel.GROUND_ALGAE);
          RobotContainer.coralIntakeCommand
              .setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
        }));
    operator.a().whileTrue(new InstantCommand(
        () -> {
          RobotContainer.elevatorCommand.setHeight(ScoringLevel.GROUND_CORAL);
          RobotContainer.algaeIntakeCommand
              .setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
          RobotContainer.coralIntakeCommand
              .setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
        }));
    // Go to coral source
    operator.x().whileTrue(new InstantCommand(
        () -> {
          RobotContainer.elevatorCommand.setHeight(ScoringLevel.SOURCE_CORAL);
          RobotContainer.algaeIntakeCommand
              .setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
          RobotContainer.coralIntakeCommand
              .setVelocity(LinearVelocity.ofBaseUnits(IntakeConstants.CoralIntake.intakeVelocity, MetersPerSecond));
        }));
    operator.leftBumper().whileTrue(new InstantCommand(
        () -> {
          RobotContainer.elevatorCommand.setHeight(ScoringLevel.L2);
          RobotContainer.algaeIntakeCommand
              .setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
        }));
    operator.rightBumper().whileTrue(new InstantCommand(
        () -> {
          RobotContainer.elevatorCommand.setHeight(ScoringLevel.L3);
          RobotContainer.algaeIntakeCommand
              .setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
        }));
    operator.rightTrigger().whileTrue(new InstantCommand(
        () -> {
          RobotContainer.elevatorCommand.setHeight(ScoringLevel.L4);
          RobotContainer.algaeIntakeCommand
              .setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
        }));

  }
}
