package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    // Reset gyro to 0° when B button is pressed
    operator
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        Drive.getInstance()
                            .setPose(
                                new Pose2d(
                                    Drive.getInstance().getPose().getTranslation(),
                                    new Rotation2d())),
                    Drive.getInstance())
                .ignoringDisable(true));

    // Intake commands

    // Coral Intake
    driver.R1()whileTrue(new InstantCommand(
      () ->{
        RobotContainer.CoralIntakeCommand.setVelocity(LinearVelocity.ofBaseUnits(IntakeConstants.CoralIntake.intakeVelocity, MetersPerSecond))
    }))
    driver.R1()onFalse(new InstantCommand(
      () -> {
        RobotContainer.CoralIntake.setVelocity(0)
    }))
    // Algae Intake

    // Elevator

  }
}
