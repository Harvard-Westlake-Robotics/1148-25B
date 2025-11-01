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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HangCommand;
import frc.robot.constants.DriveConstants;
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
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;
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
  /**
   * This defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a
   * roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay" (log
   * replay from a file).
   */
  public static final Mode simMode = Mode.SIM;

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // Paths
  private PathPlannerPath pathfindL;
  private PathPlannerPath pathfindSource;
  private PathPlannerPath crossLine;

  // Subsystems
  public final Drive drive;
  private final Pivot pivot;
  private final Wrist wrist;
  private final Elevator elevator;
  private final CoralIntake coralIntake;
  private final AlgaeIntake algaeIntake;
  private final Hang hang;

  // Commands
  public static ArmCommand armCommand;
  public static AlgaeIntakeCommand algaeIntakeCommand;
  public static CoralIntakeCommand coralIntakeCommand;
  public static HangCommand hangCommand;

  // Controllers
  public final CommandXboxController operator = new CommandXboxController(1);
  public final CommandPS5Controller driver = new CommandPS5Controller(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Command> preAutoChooser;

  public static SwerveDriveSimulation driveSimulation = null;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    switch (currentMode) {
      case REAL:
        // Real robot: instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(DriveConstants.FrontLeft),
                new ModuleIOTalonFXReal(DriveConstants.FrontRight),
                new ModuleIOTalonFXReal(DriveConstants.BackLeft),
                new ModuleIOTalonFXReal(DriveConstants.BackRight),
                pose -> {});
        break;

      case SIM:
        // Sim robot: instantiate physics sim IO implementations
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
        break;

      default:
        // Replayed robot: disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOTalonFX(DriveConstants.FrontLeft) {},
                new ModuleIOTalonFX(DriveConstants.FrontRight) {},
                new ModuleIOTalonFX(DriveConstants.BackLeft) {},
                new ModuleIOTalonFX(DriveConstants.BackRight) {},
                pose -> {});
        break;
    }

    // Instantiate subsystems
    this.pivot = Pivot.getInstance();
    this.wrist = Wrist.getInstance();
    this.elevator = Elevator.getInstance();
    this.coralIntake = CoralIntake.getInstance();
    this.algaeIntake = AlgaeIntake.getInstance();
    this.hang = Hang.getInstance();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // try {
    //   PathPlannerPath crossLine = PathPlannerPath.fromPathFile("Push");
    // } catch (Exception e) {
    // }

    // autoChooser.addOption("Cross the Line", AutoBuilder.followPath(crossLine));

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
        "Elevator SysId (Quasistatic Reverse)",
        elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Elevator SysId (Dynamic Forward)", elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Elevator SysId (Dynamic Reverse)", elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption(
        "Hang SysId (Quasistatic Forward)", hang.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Hang SysId (Quasistatic Reverse)", hang.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Hang SysId (Dynamic Forward)", hang.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Hang SysId (Dynamic Reverse)", hang.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption(
        "CoralIntake SysId (Quasistatic Forward)",
        coralIntake.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "CoralIntake SysId (Quasistatic Reverse)",
        coralIntake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "CoralIntake SysId (Dynamic Forward)",
        coralIntake.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "CoralIntake SysId (Dynamic Reverse)",
        coralIntake.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption(
        "AlgaeIntake SysId (Quasistatic Forward)",
        algaeIntake.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "AlgaeIntake SysId (Quasistatic Reverse)",
        algaeIntake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "AlgaeIntake SysId (Dynamic Forward)",
        algaeIntake.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "AlgaeIntake SysId (Dynamic Reverse)",
        algaeIntake.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption(
        "Pivot SysID (Quasistatic Forward)",
        pivot.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Pivot SysID (Quasistatic Reverse)",
        pivot.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Pivot SysID (Dynamic Forward)", pivot.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Pivot SysID (Dynamic Reverse)", pivot.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption(
        "Wrist SysId (Quasistatic Forward)",
        wrist.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Wrist SysId (Quasistatic Reverse)",
        wrist.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Wrist SysId (Dynamic Forward)", wrist.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Wrist SysId (Dynamic Reverse)", wrist.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
    // Set default commands, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> driver.getRightX()));
    // Instantiate and set default commands
    armCommand = new ArmCommand();
    elevator.setDefaultCommand(armCommand);
    coralIntakeCommand = new CoralIntakeCommand();
    coralIntake.setDefaultCommand(coralIntakeCommand);
    algaeIntakeCommand = new AlgaeIntakeCommand();
    algaeIntake.setDefaultCommand(algaeIntakeCommand);
    hangCommand = new HangCommand();
    hang.setDefaultCommand(hangCommand);

    // Assign controls in ControlMap
    ControlMap.getInstance().configurePreset1(operator, driver);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (currentMode != Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (currentMode != Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
