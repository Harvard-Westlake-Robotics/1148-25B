package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer.Mode;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmCommand.ScoringLevel;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.wrists.Pivot;

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

    // Intake commands

    // Coral intake commands
    // Coral intake is intaking automatically when elevator is at coral source (ground)
    driver
        .R1()
        .whileTrue(
            new InstantCommand(
                () -> {
                  if (ArmCommand.level == ScoringLevel.GROUND_ALGAE) {
                    // If at algae level, intake algae
                    RobotContainer.algaeIntakeCommand.runVelocity(
                        IntakeConstants.AlgaeIntake.intakeVelocity.in(MetersPerSecond));
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
                    RobotContainer.algaeIntakeCommand.runVelocity(0);
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
                    RobotContainer.algaeIntakeCommand.runVelocity(
                        IntakeConstants.AlgaeIntake.intakeVelocity.in(MetersPerSecond));
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
                    RobotContainer.algaeIntakeCommand.runVelocity(0);
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
                  if (RobotContainer.armCommand.outtakePosition) {
                    RobotContainer.coralIntakeCommand.runOuttake();
                  } else if (ArmCommand.level == ScoringLevel.GROUND_ALGAE) {
                    RobotContainer.algaeIntakeCommand.runVelocity(
                        IntakeConstants.AlgaeIntake.intakeVelocity.in(MetersPerSecond));
                  } else if (ArmCommand.level == ScoringLevel.NET) {
                    RobotContainer.algaeIntakeCommand.runVelocity(
                        IntakeConstants.AlgaeIntake.outtakeVelocity.in(MetersPerSecond));
                  }
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  RobotContainer.coralIntakeCommand.stopIntake();
                  RobotContainer.algaeIntakeCommand.runVelocity(0);
                }));

    // driver
    //     .L2()
    //     .whileTrue(
    //         new Command() {
    //           @Override
    //           public void initialize() {
    //             if (Drive.getInstance().getCurrentCommand() != null) {
    //               Drive.getInstance().getCurrentCommand().cancel();
    //             }
    //             if (Elevator.getInstance().getCurrentCommand() != null) {
    //               Elevator.getInstance().getCurrentCommand().cancel();
    //             }
    //           }

    //           @Override
    //           public void end(boolean interrupted) {
    //             NetworkCommunicator.getInstance().getTeleopCommand().cancel();
    //             RobotContainer.armCommand.setHeight(ScoringLevel.NEUTRAL);
    //             Drive.getInstance().stop();
    //             if (Drive.getInstance().getCurrentCommand() != null) {
    //               Drive.getInstance().getCurrentCommand().cancel();
    //             }
    //             if (Elevator.getInstance().getCurrentCommand() != null) {
    //               Elevator.getInstance().getCurrentCommand().cancel();
    //             }
    //           }
    //         });

    // Elevator
    // operator
    //     .x()
    //     .whileTrue(
    //         new InstantCommand(
    //             () -> {
    //               RobotContainer.armCommand.setHeight(ScoringLevel.GROUND_CORAL);
    //               RobotContainer.algaeIntakeCommand.runVelocity(0);
    //             }));
    // operator
    //     .y()
    //     .whileTrue(
    //         new InstantCommand(
    //             () -> {
    //               RobotContainer.armCommand.setHeight(ScoringLevel.NET);
    //               RobotContainer.algaeIntakeCommand.runVelocity(0);
    //               RobotContainer.coralIntakeCommand.stopIntake();
    // }));
    // operator
    //     .b()
    //     .whileTrue(
    //         new InstantCommand(
    //             () -> {
    //               RobotContainer.armCommand.setHeight(ScoringLevel.PROCESSOR);
    //               RobotContainer.algaeIntakeCommand.runVelocity(0);
    //               RobotContainer.coralIntakeCommand.stopIntake();
    //             }));
    // operator
    //     .leftBumper()
    //     .whileTrue(
    //         new InstantCommand(
    //             () -> {
    //               RobotContainer.armCommand.setHeight(ScoringLevel.L2);
    //               RobotContainer.algaeIntakeCommand.runVelocity(0);
    //             }));
    // operator
    //     .rightBumper()
    //     .whileTrue(
    //         new InstantCommand(
    //             () -> {
    //               RobotContainer.armCommand.setHeight(ScoringLevel.L3);
    //               RobotContainer.algaeIntakeCommand.runVelocity(0);
    //             }));
    // operator
    //     .rightTrigger()
    //     .whileTrue(
    //         new InstantCommand(
    //             () -> {
    //               RobotContainer.armCommand.setHeight(ScoringLevel.L4);
    //               RobotContainer.algaeIntakeCommand.runVelocity(0);
    //             }));
    // operator
    //     .povUp()
    //     .whileTrue(
    //         new InstantCommand(
    //             () -> RobotContainer.armCommand.setHeight(ScoringLevel.BOTTOM_REMOVE)));
    // driver
    //     .povDown()
    //     .whileTrue(
    //         new InstantCommand(
    //             () -> {
    //               Pivot.getInstance().goToAngleClosedLoop(30);
    //               ;
    //               RobotContainer.coralIntakeCommand.stopIntake();
    //             }));

    // Hang

    operator
        .b()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.hangCommand.run();
                }))
        .onFalse(new InstantCommand(() -> RobotContainer.hangCommand.stop()));

    driver
        .square()
        .whileTrue(
            new InstantCommand(
                () -> {
                  Pivot.getInstance().goToAngleClosedLoop(0);
                }))
        .whileFalse(
            new InstantCommand(
                () -> {
                  RobotContainer.hangCommand.lock();
                }));

    // Commands not done: whatever the "rest position" thing is on
    // the document

  }
}
