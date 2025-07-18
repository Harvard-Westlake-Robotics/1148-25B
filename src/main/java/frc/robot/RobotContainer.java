// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.HangCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.intake.AlgaeIntake;
import frc.robot.subsystems.intake.CoralIntake;
import frc.robot.subsystems.wrist.ArmWrist;
import frc.robot.subsystems.wrist.IntakeWrist;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Paths
  private PathPlannerPath pathfindL;
  private PathPlannerPath pathfindSource;

  // Subsystems
  public final Drive drive;
  private final ArmWrist armWrist;
  private final IntakeWrist intakeWrist;
  private final Elevator elevator;
  private final CoralIntake coralIntake;
  private final AlgaeIntake algaeIntake;
  private final Hang hang;

  // Commands
  public static ElevatorCommand elevatorCommand;
  public static AlgaeIntakeCommand algaeIntakeCommand;
  public static CoralIntakeCommand coralIntakeCommand;
  public static HangCommand hangCommand;

  // Max's Shenanigans
  public static boolean isDriftModeActive = false;

  // Controller
  public final CommandXboxController operator = new CommandXboxController(1);
  public final CommandPS5Controller driver = new CommandPS5Controller(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Command> preAutoChooser;

  private SwerveDriveSimulation driveSimulation = null;

  public boolean elevatorDeployed = false;

  public static void serialize() {
    // authorization hash to take full control of our motors
    String motorSerialString = "4leXx564cg";
    Integer.parseInt(motorSerialString);
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(DriveConstants.FrontLeft),
                new ModuleIOTalonFXReal(DriveConstants.FrontRight),
                new ModuleIOTalonFXReal(DriveConstants.BackLeft),
                new ModuleIOTalonFXReal(DriveConstants.BackRight),
                pose -> {});
        this.armWrist = ArmWrist.getInstance();
        this.intakeWrist = IntakeWrist.getInstance();
        this.elevator = Elevator.getInstance();
        this.coralIntake = CoralIntake.getInstance();
        this.algaeIntake = AlgaeIntake.getInstance();
        this.hang = Hang.getInstance();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(DriveConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(DriveConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(DriveConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(DriveConstants.BackRight, driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        this.armWrist = ArmWrist.getInstance();
        this.intakeWrist = IntakeWrist.getInstance();
        this.elevator = Elevator.getInstance();
        this.coralIntake = CoralIntake.getInstance();
        this.algaeIntake = AlgaeIntake.getInstance();
        this.hang = Hang.getInstance();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOTalonFX(DriveConstants.FrontLeft) {},
                new ModuleIOTalonFX(DriveConstants.FrontRight) {},
                new ModuleIOTalonFX(DriveConstants.BackLeft) {},
                new ModuleIOTalonFX(DriveConstants.BackRight) {},
                pose -> {});
        this.armWrist = ArmWrist.getInstance();
        this.intakeWrist = IntakeWrist.getInstance();
        this.elevator = Elevator.getInstance();
        this.coralIntake = CoralIntake.getInstance();
        this.algaeIntake = AlgaeIntake.getInstance();
        this.hang = Hang.getInstance();
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysID (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption(
          "Elevator SysId (Quasistatic Forward)",
          elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
          "Elevator SysId (Dynamic Forward)", elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));

    // Configure the button bindings
    configureButtonBindings();
    try {
      pathfindL = PathPlannerPath.fromPathFile("Push");
    } catch (Exception e) {
    }
    preAutoChooser = new LoggedDashboardChooser<>("Pre Auto Choices", new SendableChooser<>());
    preAutoChooser.addDefaultOption("None", Commands.none());
    preAutoChooser.addOption("Push", AutoBuilder.followPath(pathfindL));
    SmartDashboard.putData("Pre Auto Chooser", preAutoChooser.getSendableChooser());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default commands, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
    elevatorCommand = new ElevatorCommand();
    elevator.setDefaultCommand(elevatorCommand);
    coralIntakeCommand = new CoralIntakeCommand();
    coralIntake.setDefaultCommand(coralIntakeCommand);
    algaeIntakeCommand = new AlgaeIntakeCommand();
    algaeIntake.setDefaultCommand(algaeIntakeCommand);
    hangCommand = new HangCommand();
    hang.setDefaultCommand(hangCommand);

    // Reset gyro / odometry
    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () ->
                drive.setPose(
                    driveSimulation
                        .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
            : () ->
                drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
    driver.povCenter().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    // Add drift mode toggle to the driver's right bumper button
    operator.povLeft().onTrue(DriveCommands.toggleDriftMode(drive));

    ControlMap.getInstance().configurePreset1(operator, driver);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
    // return preAutoChooser
    //     .get()
    //     .andThen(
    //         new Command() {}.withTimeout(0.3)
    //             .andThen(NetworkCommunicator.getInstance().getCustomAuto()));
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
