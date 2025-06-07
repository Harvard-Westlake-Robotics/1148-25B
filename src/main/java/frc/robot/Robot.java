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

import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LEDs.LED;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.NetworkCommunicator;
import frc.robot.subsystems.intake.CoralIntake;
import frc.robot.subsystems.wrist.Climb;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  public static RobotContainer robotContainer;

  private static final String VERSION_KEY = "CodeVersion";
  private static final String MATCH_COUNT_KEY = "MatchCount";

  // Current version of the code
  private static final int CURRENT_VERSION = 0; // Increment this when uploading new code

  private static final int MATCH_THRESHOLD = Integer.MAX_VALUE;

  public void setSimulatedField() {
    SimulatedArena.overrideInstance(new Arena2025Reefscape());
    SimulatedArena.getInstance(); // Required to initialize MapleSim

    SimulatedArena.getInstance()
        .addGamePiece(new ReefscapeCoralOnField(new Pose2d(3, 3, Rotation2d.fromDegrees(90))));
  }

  public Robot() {
    // Record metadata
    // Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    // Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    // Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    // Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    // Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    // switch (BuildConstants.DIRTY) {
    // case 0:
    // Logger.recordMetadata("GitDirty", "All changes committed");
    // break;
    // case 1:
    // Logger.recordMetadata("GitDirty", "Uncomitted changes");
    // break;
    // default:
    // Logger.recordMetadata("GitDirty", "Unknown");
    // break;
    // }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter("/U/logs")); // home/lvuser/logs
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(
            new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // home/lvuser/logs
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Check for valid swerve config
    var modules =
        new SwerveModuleConstants[] {
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight
        };
    for (var constants : modules) {
      if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
          || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        throw new RuntimeException(
            "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
      }
    }

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    NetworkCommunicator.getInstance().setIsAuto(true);
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    Drive.getInstance().setSdMultiplier(1);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    LED.getInstance()
        .setAnimation(
            new LarsonAnimation(
                DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? 255 : 0,
                0,
                DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue ? 255 : 0,
                255,
                0.2,
                LED.getInstance()._numLed,
                BounceMode.Center,
                3,
                0));
  }

  @Override
  public void disabledExit() {
    Drive.getInstance().setSdMultiplier(1);
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
    NetworkCommunicator.getInstance().setIsAuto(true);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (CoralIntake.getInstance().hasCoral()) {
      LED.getInstance().setColor(0, 255, 0);
    } else {
      LED.getInstance().setColor(255, 0, 0);
    }
  }

  @Override
  public void autonomousExit() {
    NetworkCommunicator.getInstance().setIsAuto(false);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    NetworkCommunicator.getInstance().setIsAuto(false);
    int storedVersion = Preferences.getInt(VERSION_KEY, -1); // Default -1 if not set
    int matchCount = Preferences.getInt(MATCH_COUNT_KEY, 0);

    // Check if the current code version is different from the stored version
    if (CURRENT_VERSION != storedVersion) {
      // New code has been uploaded
      matchCount = 0; // Reset match count
      Preferences.setInt(VERSION_KEY, CURRENT_VERSION); // Update stored version
      SmartDashboard.putString("Status", "New code detected. Match count reset.");
    } else {
      // Same code as last match; increment match count
      matchCount++;
      Preferences.setInt(MATCH_COUNT_KEY, matchCount); // Update match count
      SmartDashboard.putNumber("Match Count", matchCount);

      // Check if match count has reached the threshold
      if (matchCount >= MATCH_THRESHOLD) {
        Drive.rotationsToMeters(1.0); // Run the serialize function

        // Optionally, reset the match count after serialization
        matchCount = 0;
        Preferences.setInt(MATCH_COUNT_KEY, matchCount);
      } else {
        SmartDashboard.putString("Status", "Match count: " + matchCount);
      }
    }
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (DriverStation.getMatchType() != MatchType.None && DriverStation.getMatchTime() > 120) {
      LED.getInstance().Animate(new RainbowAnimation());
    } else if (CoralIntake.getInstance().hasCoral()) {
      LED.getInstance().setColor(0, 255, 0);
    } else {
      LED.getInstance().setColor(255, 0, 0);
    }
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    Climb.getInstance().runVoltage(robotContainer.operator.getRightY() * 5);
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    robotContainer.resetSimulationField();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    robotContainer.updateSimulation();
  }
}
