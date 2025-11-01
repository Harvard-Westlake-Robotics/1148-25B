package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Camera.BaseCam.AprilTagResult;
import frc.robot.subsystems.drive.Drive;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class DriveConstants {
  // The steer motor uses any SwerveModule.SteerRequestType control request with
  // the
  // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput

  // ================================= PID Tuning =================================

  // Swerve Steer PID Values
  // TODO: Tune drive/steer PIDSVA constants
  public static final double kSteerP = 65;
  public static final double kSteerI = 0;
  public static final double kSteerD = 0;
  public static final double kSteerS = 0;
  public static final double kSteerV = 0;
  public static final double kSteerA = 0;
  public static final StaticFeedforwardSignValue kStaticFeedforwardSign =
      StaticFeedforwardSignValue.UseClosedLoopSign;

  // Swerve Drive PID Values
  public static final double kDriveP = 0.032762;
  public static final double kDriveI = 0;
  public static final double kDriveD = 0;
  public static final double kDriveS = 0.047782;
  public static final double kDriveV = 0.10756;
  public static final double kDriveA = 0.023363;

  // The closed-loop output type to use for the motors;
  // This affects PID/FF gains
  public static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  public static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  // PathPlanner PIDs
  public static double PP_ROTATION_P = 5.05;
  public static double PP_ROTATION_I = 0.00;
  public static double PP_ROTATION_D = 0.00;
  public static double PP_TRANSLATION_P = 4.45;
  public static double PP_TRANSLATION_I = 0.00;
  public static double PP_TRANSLATION_D = 0.0;

  // Limelight Standard Deviation Coefficients

  public static double xyStdDevCoeff = 6.85;
  public static double rStdDevCoeff = 6.85;

  // ================================= Hardware Tuning =================================

  public static final double ROBOT_MASS_KG = 54.088;
  public static final double ROBOT_MOI = 6.883;
  public static final double WHEEL_COF = 1.2;

  // The type of motor used for the drive motor
  public static final DriveMotorArrangement kDriveMotorType =
      DriveMotorArrangement.TalonFX_Integrated;
  // The type of motor used for the steer motor
  public static final SteerMotorArrangement kSteerMotorType =
      SteerMotorArrangement.TalonFX_Integrated;

  // The remote sensor feedback type to use for the steer motors;
  // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to
  // RemoteCANcoder
  public static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

  // The stator current at which the wheels start to slip;
  // This needs to be tuned to your individual robot
  // TODO: Sysid this
  public static final Current kSlipCurrent = Amps.of(130.0);

  // Theoretical free speed (m/s) at 12 V applied output;
  // This needs to be tuned to your individual robot
  // TODO: Sysid this
  public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(7.50);

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
  // This may need to be tuned to your individual robot
  public static final double kCoupleRatio = 5.4;

  public static final double kDriveGearRatio = 5.68;
  public static final double kSteerGearRatio = 12.1;
  public static final Distance kWheelRadius = Inches.of(2.15);

  public static final boolean kInvertLeftSide = false;
  public static final boolean kInvertRightSide = true;

  public static final int kPigeonId = 28;

  // Simulated moment of inertia for the steer and drive motors;
  public static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
  public static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
  // Simulated voltage necessary to overcome friction
  public static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
  public static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

  // Front Left
  public static final int kFrontLeftDriveMotorId = 1;
  public static final int kFrontLeftSteerMotorId = 2;
  public static final int kFrontLeftEncoderId = 11;
  public static final Angle kFrontLeftEncoderOffset = Rotations.of(0.010742);
  public static final boolean kFrontLeftSteerMotorInverted = true;
  public static final boolean kFrontLeftEncoderInverted = false;
  public static final Distance kFrontLeftXPos = Inches.of(11.5);
  public static final Distance kFrontLeftYPos = Inches.of(11.5);

  // Front Right
  public static final int kFrontRightDriveMotorId = 3;
  public static final int kFrontRightSteerMotorId = 4;
  public static final int kFrontRightEncoderId = 12;
  public static final Angle kFrontRightEncoderOffset = Rotations.of(-0.452881);
  public static final boolean kFrontRightSteerMotorInverted = true;
  public static final boolean kFrontRightEncoderInverted = false;

  public static final Distance kFrontRightXPos = Inches.of(11.5);
  public static final Distance kFrontRightYPos = Inches.of(-11.5);

  // Back Left
  public static final int kBackLeftDriveMotorId = 5;
  public static final int kBackLeftSteerMotorId = 6;
  public static final int kBackLeftEncoderId = 13;
  public static final Angle kBackLeftEncoderOffset = Rotations.of(-0.299316 + 0.5);
  public static final boolean kBackLeftSteerMotorInverted = true;
  public static final boolean kBackLeftEncoderInverted = false;

  public static final Distance kBackLeftXPos = Inches.of(-11.5);
  public static final Distance kBackLeftYPos = Inches.of(11.5);

  // Back Right
  public static final int kBackRightDriveMotorId = 7;
  public static final int kBackRightSteerMotorId = 8;
  public static final int kBackRightEncoderId = 14;
  public static final Angle kBackRightEncoderOffset = Rotations.of(0.139404 + 0.5);
  public static final boolean kBackRightSteerMotorInverted = true;
  public static final boolean kBackRightEncoderInverted = false;

  public static final Distance kBackRightXPos = Inches.of(-11.5);
  public static final Distance kBackRightYPos = Inches.of(-11.5);

  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(kFrontLeftXPos.baseUnitMagnitude(), kFrontLeftYPos.baseUnitMagnitude()),
              Math.hypot(kFrontRightXPos.baseUnitMagnitude(), kFrontRightYPos.baseUnitMagnitude())),
          Math.max(
              Math.hypot(kBackLeftXPos.baseUnitMagnitude(), kBackLeftYPos.baseUnitMagnitude()),
              Math.hypot(kBackRightXPos.baseUnitMagnitude(), kBackRightYPos.baseUnitMagnitude())));

  // ================================= Limelight Standard Dev =================================

  private static double sdMultiplier = 1;

  public static double getSdMultiplier() {
    return sdMultiplier;
  }

  public static void setSdMultiplier(double multiplier) {
    sdMultiplier = multiplier;
  }

  // ================================= END OF TUNER CONSTANTS =================================

  // ================================= Robot Software Configs =================================

  public static final Slot0Configs steerGains =
      new Slot0Configs()
          .withKP(kSteerP)
          .withKI(kSteerI)
          .withKD(kSteerD)
          .withKS(kSteerS)
          .withKV(kSteerV)
          .withKA(kSteerA)
          .withStaticFeedforwardSign(kStaticFeedforwardSign);
  // When using closed-loop control, the drive motor uses the control
  // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
  public static final Slot0Configs driveGains =
      new Slot0Configs()
          .withKP(kDriveP)
          .withKI(kDriveI)
          .withKD(kDriveD)
          .withKS(kDriveS)
          .withKV(kDriveV)
          .withKA(kDriveA);

  // Initial configs for the drive and steer motors and the azimuth encoder; these
  // cannot be null.
  // Some configs will be overwritten; check the `with*InitialConfigs()` API
  // documentation.
  public static final TalonFXConfiguration driveInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(60))
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(kSlipCurrent))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
  public static final TalonFXConfiguration steerInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  // Swerve azimuth does not require much torque output, so we can set a
                  // relatively low stator current limit to help avoid brownouts without
                  // impacting performance.
                  .withStatorCurrentLimit(Amps.of(60))
                  .withStatorCurrentLimitEnable(true))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));

  public static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

  // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
  public static final Pigeon2Configuration pigeonConfigs = new Pigeon2Configuration();

  // CAN bus that the devices are located on;
  // All swerve devices must share the same CAN bus
  public static final CANBus kCANBus = new CANBus("drive", "./logs/driveCAN.hoot");

  public static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants()
          .withCANBusName(kCANBus.getName())
          .withPigeon2Id(kPigeonId)
          .withPigeon2Configs(pigeonConfigs);

  // ================================= Module Configurations =================================

  public static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      ConstantCreator =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorGearRatio(kDriveGearRatio)
              .withSteerMotorGearRatio(kSteerGearRatio)
              .withCouplingGearRatio(kCoupleRatio)
              .withWheelRadius(kWheelRadius)
              .withSteerMotorGains(steerGains)
              .withDriveMotorGains(driveGains)
              .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
              .withSlipCurrent(kSlipCurrent)
              .withSpeedAt12Volts(kSpeedAt12Volts)
              .withDriveMotorType(kDriveMotorType)
              .withSteerMotorType(kSteerMotorType)
              .withFeedbackSource(kSteerFeedbackType)
              .withDriveMotorInitialConfigs(driveInitialConfigs)
              .withSteerMotorInitialConfigs(steerInitialConfigs)
              .withEncoderInitialConfigs(encoderInitialConfigs)
              .withSteerInertia(kSteerInertia)
              .withDriveInertia(kDriveInertia)
              .withSteerFrictionVoltage(kSteerFrictionVoltage)
              .withDriveFrictionVoltage(kDriveFrictionVoltage);

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft =
          ConstantCreator.createModuleConstants(
                  DriveConstants.kFrontLeftSteerMotorId,
                  DriveConstants.kFrontLeftDriveMotorId,
                  DriveConstants.kFrontLeftEncoderId,
                  DriveConstants.kFrontLeftEncoderOffset,
                  DriveConstants.kFrontLeftXPos,
                  DriveConstants.kFrontLeftYPos,
                  DriveConstants.kInvertLeftSide,
                  DriveConstants.kFrontLeftSteerMotorInverted,
                  DriveConstants.kFrontLeftEncoderInverted)
              .withSlipCurrent(DriveConstants.kSlipCurrent);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight =
          ConstantCreator.createModuleConstants(
                  DriveConstants.kFrontRightSteerMotorId,
                  DriveConstants.kFrontRightDriveMotorId,
                  DriveConstants.kFrontRightEncoderId,
                  DriveConstants.kFrontRightEncoderOffset,
                  DriveConstants.kFrontRightXPos,
                  DriveConstants.kFrontRightYPos,
                  DriveConstants.kInvertRightSide,
                  DriveConstants.kFrontRightSteerMotorInverted,
                  DriveConstants.kFrontRightEncoderInverted)
              .withSlipCurrent(DriveConstants.kSlipCurrent);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft =
          ConstantCreator.createModuleConstants(
                  DriveConstants.kBackLeftSteerMotorId,
                  DriveConstants.kBackLeftDriveMotorId,
                  DriveConstants.kBackLeftEncoderId,
                  DriveConstants.kBackLeftEncoderOffset,
                  DriveConstants.kBackLeftXPos,
                  DriveConstants.kBackLeftYPos,
                  DriveConstants.kInvertLeftSide,
                  DriveConstants.kBackLeftSteerMotorInverted,
                  DriveConstants.kBackLeftEncoderInverted)
              .withSlipCurrent(DriveConstants.kSlipCurrent);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight =
          ConstantCreator.createModuleConstants(
                  DriveConstants.kBackRightSteerMotorId,
                  DriveConstants.kBackRightDriveMotorId,
                  DriveConstants.kBackRightEncoderId,
                  DriveConstants.kBackRightEncoderOffset,
                  DriveConstants.kBackRightXPos,
                  DriveConstants.kBackRightYPos,
                  DriveConstants.kInvertRightSide,
                  DriveConstants.kBackRightSteerMotorInverted,
                  DriveConstants.kBackRightEncoderInverted)
              .withSlipCurrent(DriveConstants.kSlipCurrent);

  // ================================= PathPlanner Configs =================================

  public static final RobotConfig PP_CONFIG =
      new RobotConfig(
          DriveConstants.ROBOT_MASS_KG,
          DriveConstants.ROBOT_MOI,
          new ModuleConfig(
              DriveConstants.kWheelRadius.baseUnitMagnitude(),
              DriveConstants.kSpeedAt12Volts.in(MetersPerSecond),
              DriveConstants.WHEEL_COF,
              DCMotor.getKrakenX60Foc(1).withReduction(DriveConstants.kDriveGearRatio),
              DriveConstants.kSlipCurrent.in(Amps),
              1),
          Drive.getModuleTranslations());

  // ================================= Simulation Configs =================================

  // Create and configure a drivetrain simulation configuration
  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          // Specify robot mass
          .withRobotMass(Kilograms.of(DriveConstants.ROBOT_MASS_KG)) // Set robot mass in kg
          // Specify gyro type (for realistic gyro drifting and error simulation)
          .withGyro(COTS.ofPigeon2())
          // Specify module positions
          .withCustomModuleTranslations(Drive.getModuleTranslations())
          // Specify swerve module (for realistic swerve dynamics)
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
                  DCMotor.getFalcon500(1), // Steer motor is a Falcon 500
                  DriveConstants.kDriveGearRatio, // Drive motor gear ratio.
                  DriveConstants.kSteerGearRatio, // Steer motor gear ratio.
                  DriveConstants.kDriveFrictionVoltage, // Drive friction voltage.
                  DriveConstants.kSteerFrictionVoltage, // Steer friction voltage
                  Inches.of(DriveConstants.kWheelRadius.magnitude()), // Wheel radius
                  DriveConstants.kSteerInertia, // Steer MOI
                  1.2)) // Wheel COF
          // Configures the track length and track width (spacing between swerve modules)
          .withTrackLengthTrackWidth(Inches.of(21), Inches.of(21))
          // Configures the bumper size (dimensions of the robot bumper)
          .withBumperSize(Inches.of(33.6), Inches.of(33.6));

  // ================================= Extra Configurations =================================

  public static final double ODOMETRY_FREQUENCY =
      new CANBus(DriveConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;

  // ================================= PathPlanner Constraints =================================

  // TODO: Tune
  public static final PathConstraints PP_CONSTRAINTS =
      new PathConstraints(
          MetersPerSecond.of(7.5),
          MetersPerSecondPerSecond.of(5.5),
          DegreesPerSecond.of(1020),
          DegreesPerSecondPerSecond.of(2400));

  public static double xyStdDev(AprilTagResult result) {
    return xyStdDevCoeff
        * Math.max(Math.pow(result.distToTag, 2.0), 0.5)
        / result.tagCount
        * Math.sqrt(result.ambiguity)
        * sdMultiplier;
  }

  public static double rStdDev(AprilTagResult result) {
    return rStdDevCoeff
        * Math.max(Math.pow(result.distToTag, 2.0), 0.5)
        / result.tagCount
        * Math.sqrt(result.ambiguity)
        * sdMultiplier;
  }
}
