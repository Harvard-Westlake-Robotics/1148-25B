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

    // Intake commands

    // Coral intake commands
    // Coral intake is intaking automatically when elevator is at coral source (ground)
    // driver
    //     .R1()
    //     .whileTrue(
    //         new InstantCommand(
    //             () -> {
    //               if (ArmCommand.level == ScoringLevel.GROUND_ALGAE) {
    //                 // If at algae level, intake algae
    //                 RobotContainer.algaeIntakeCommand.runVelocity(
    //                     IntakeConstants.AlgaeIntake.intakeVelocity.in(MetersPerSecond));
    //               } else {
    //                 // Intake straight
    //                 RobotContainer.coralIntakeCommand.runIntake(true);
    //               }
    //             }))
    //     .onFalse(
    //         new InstantCommand(
    //             () -> {
    //               if (ArmCommand.level == ScoringLevel.GROUND_ALGAE) {
    //                 // If at algae level, intake algae
    //                 RobotContainer.algaeIntakeCommand.runVelocity(0);
    //               } else {
    //                 // Intake straight
    //                 RobotContainer.coralIntakeCommand.stopIntake();
    //               }
    //             }));
    // driver
    //     .L1()
    //     .whileTrue(
    //         new InstantCommand(
    //             () -> {
    //               if (ArmCommand.level == ScoringLevel.GROUND_ALGAE) {
    //                 // If at algae level, intake algae
    //                 RobotContainer.algaeIntakeCommand.runVelocity(
    //                     IntakeConstants.AlgaeIntake.intakeVelocity.in(MetersPerSecond));
    //               } else {
    //                 // Intake straight
    //                 RobotContainer.coralIntakeCommand.runIntake(false);
    //               }
    //             }))
    //     .onFalse(
    //         new InstantCommand(
    //             () -> {
    //               if (ArmCommand.level == ScoringLevel.GROUND_ALGAE) {
    //                 // If at algae level, intake algae
    //                 RobotContainer.algaeIntakeCommand.runVelocity(0);
    //               } else {
    //                 // Intake straight
    //                 RobotContainer.coralIntakeCommand.stopIntake();
    //               }
    //             }));
    // Shared Outtake
    // driver
    //     .R2()
    //     .whileTrue(
    //         new InstantCommand(
    //             () -> {
    //               if (RobotContainer.armCommand.outtakePosition) {
    //                 RobotContainer.coralIntakeCommand.runOuttake();
    //               } else if (ArmCommand.level == ScoringLevel.GROUND_ALGAE) {
    //                 RobotContainer.algaeIntakeCommand.runVelocity(
    //                     IntakeConstants.AlgaeIntake.intakeVelocity.in(MetersPerSecond));
    //               } else if (ArmCommand.level == ScoringLevel.NET) {
    //                 RobotContainer.algaeIntakeCommand.runVelocity(
    //                     IntakeConstants.AlgaeIntake.outtakeVelocity.in(MetersPerSecond));
    //               }
    //             }))
    //     .onFalse(
    //         new InstantCommand(
    //             () -> {
    //               RobotContainer.coralIntakeCommand.stopIntake();
    //               RobotContainer.algaeIntakeCommand.runVelocity(0);
    //             }));

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
    // RobotContainer.algaeIntakeCommand.runVelocity(0);
    // RobotContainer.coralIntakeCommand.stopIntake();
    // operator
    //     .povRight()
    //     .whileTrue(
    //         new InstantCommand(() -> Pivot.getInstance().goToAngleClosedLoop(-15)));
    // RobotContainer.algaeIntakeCommand.runVelocity(0);
    // RobotContainer.coralIntakeCommand.stopIntake();

    // Commands not done: whatever the "rest position" thing is on
    // the document

    // MANUAL COMMANDS --> Operator:
    // Pivot down to limit
    operator
        .a()
        .whileTrue(
            new InstantCommand(
                () -> {
                  // Pivot.getInstance().goToAngleClosedLoop(-19);
                  RobotContainer.armCommand.setPivotAngle(-15);
                  RobotContainer.algaeIntakeCommand.runVelocity(0);
                }));

    // Pivot up to hang position
    operator
        .y()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.armCommand.setPivotAngle(9);
                  // Pivot.getInstance().goToAngleClosedLoop(9);
                }));
    // hanging after we have clamped the barge
    operator
        .x()
        .whileTrue(
            new InstantCommand(() -> RobotContainer.hangCommand.hang())
                .andThen(
                    new InstantCommand(
                        () -> {
                          RobotContainer.armCommand.setPivotAngle(-19);
                          // Pivot.getInstance().goToAngleClosedLoop(-20);
                        })));
    // Algae intake
    operator
        .leftBumper()
        .whileTrue(new InstantCommand(() -> RobotContainer.algaeIntakeCommand.intake()));
    // .onFalse(new InstantCommand(() -> RobotContainer.algaeIntakeCommand.stop()));
    // Algae outtake
    operator
        .leftTrigger()
        .whileTrue(new InstantCommand(() -> RobotContainer.algaeIntakeCommand.outtake()));
    // .onFalse(new InstantCommand(() -> RobotContainer.algaeIntakeCommand.stop()));
    operator
        .rightBumper()
        .whileTrue(new InstantCommand(() -> RobotContainer.algaeIntakeCommand.stop()));

    // For hang + other stuff -> flip out the wrist
    operator
        .povRight()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.armCommand.setWristAngle(-15);
                  // RobotContainer.hangCommand.flipOut()
                }));

    // Flip in the wrist to angle 0
    operator
        .povLeft()
        .whileTrue(new InstantCommand(() -> RobotContainer.armCommand.setWristAngle(0)));
    // RobotContainer.hangCommand.flipIn()
    // run the hang motors
    operator
        .b()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.hangCommand.run();
                }))
        .onFalse(new InstantCommand(() -> RobotContainer.hangCommand.stop()));
    operator
        .povUp()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.armCommand.setWristAngle(-11);
                }));

    // MANUAL CONTROLS --> DRIVER:
    // Intake and outtake (L2 and R2)
    driver
        .L2()
        .whileTrue(new InstantCommand(() -> RobotContainer.coralIntakeCommand.manualIntake()))
        .onFalse(new InstantCommand(() -> RobotContainer.coralIntakeCommand.stopIntake()));
    driver
        .R2()
        .whileTrue(new InstantCommand(() -> RobotContainer.coralIntakeCommand.manualOuttake()))
        .onFalse(new InstantCommand(() -> RobotContainer.coralIntakeCommand.stopIntake()));
    // Go to hang position with BOTH arm and pivot. Just a test, can be moved later.
    driver
        .L1()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.armCommand.setHeight(ScoringLevel.HANG);
                }));
    operator
        .povDown()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.armCommand.setElevatorLength(2);
                }));
  }
}
