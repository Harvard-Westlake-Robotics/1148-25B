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
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See thecon
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFX implements ModuleIO {
  // Constants for drift mode
  protected static final double DRIFT_CURRENT_LIMIT_STATOR =
      240; // Higher current limit for more torque during drift
  protected static final double DRIFT_CURRENT_LIMIT_SUPPLY = 100; // Supply current limit

  protected final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  // Hardware objects
  protected final TalonFX driveTalon;
  protected final TalonFX turnTalon;
  protected final CANcoder cancoder;

  // Is this a front module?
  protected final boolean isFrontModule;

  // Voltage control requests
  protected final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  protected final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0.0).withEnableFOC(true);
  protected final VelocityVoltage velocityVoltageRequest =
      new VelocityVoltage(0.0).withEnableFOC(true);

  // Torque-current control requests
  protected final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  protected final MotionMagicTorqueCurrentFOC positionTorqueCurrentRequest =
      new MotionMagicTorqueCurrentFOC(0.0);
  protected final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Timestamp inputs from Phoenix thread
  protected final Queue<Double> timestampQueue;

  // Inputs from drive motor
  protected final StatusSignal<Angle> drivePosition;
  protected final Queue<Double> drivePositionQueue;
  protected final StatusSignal<AngularVelocity> driveVelocity;
  protected final StatusSignal<Voltage> driveAppliedVolts;
  protected final StatusSignal<Current> driveCurrent;

  // Inputs from turn motor
  protected final StatusSignal<Angle> turnAbsolutePosition;
  protected final StatusSignal<Angle> turnPosition;
  protected final Queue<Double> turnPositionQueue;
  protected final StatusSignal<AngularVelocity> turnVelocity;
  protected final StatusSignal<Voltage> turnAppliedVolts;
  protected final StatusSignal<Current> turnCurrent;

  // Connection debouncers
  protected final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  protected final Debouncer turnConnectedDebounce = new Debouncer(0.5);
  protected final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

  public ModuleIOTalonFX(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.constants = constants;

    // Determine if this is a front module based on location
    this.isFrontModule = (constants.LocationY < 0);

    driveTalon = new TalonFX(constants.DriveMotorId, TunerConstants.DrivetrainConstants.CANBusName);
    turnTalon = new TalonFX(constants.SteerMotorId, TunerConstants.DrivetrainConstants.CANBusName);
    cancoder = new CANcoder(constants.EncoderId, TunerConstants.DrivetrainConstants.CANBusName);

    // Configure drive motor
    var driveConfig = constants.DriveMotorInitialConfigs;
    driveConfig.MotionMagic.MotionMagicAcceleration = 9999;
    driveConfig.MotionMagic.MotionMagicCruiseVelocity = 9999;
    driveConfig.MotionMagic.MotionMagicJerk = 9999;
    configureDriveMotor(driveConfig);
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor
    var turnConfig = new TalonFXConfiguration();
    configureTurnMotor(turnConfig);
    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

    // Configure CANCoder
    CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
    cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection =
        constants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    cancoder.getConfigurator().apply(cancoderConfig);

    // Create timestamp queue
    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    // Create drive status signals
    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    // Create turn status signals
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnTalon.getPosition());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        Drive.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon);
  }

  /**
   * Configures the drive motor with appropriate settings for normal operation or drift mode.
   *
   * @param driveConfig The drive motor configuration to modify
   */
  private void configureDriveMotor(TalonFXConfiguration driveConfig) {
    // Choose neutral mode based on drift mode and module position
    NeutralModeValue neutralMode;
    if (RobotContainer.isDriftModeActive) {
      // In drift mode: front = brake, rear = coast
      neutralMode = isFrontModule ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    } else {
      // Normal operation: all modules brake
      neutralMode = NeutralModeValue.Brake;
    }

    driveConfig.MotorOutput.NeutralMode = neutralMode;
    driveConfig.Slot0 = constants.DriveMotorGains;
    driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;

    // Configure current limits, higher in drift mode for more torque
    if (RobotContainer.isDriftModeActive) {
      driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = DRIFT_CURRENT_LIMIT_STATOR;
      driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -DRIFT_CURRENT_LIMIT_STATOR;
      driveConfig.CurrentLimits.StatorCurrentLimit = DRIFT_CURRENT_LIMIT_STATOR;
      driveConfig.CurrentLimits.SupplyCurrentLimit = DRIFT_CURRENT_LIMIT_SUPPLY;
      driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    } else {
      driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
      driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
      driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
      driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    }

    driveConfig.MotorOutput.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
  }

  /**
   * Configures the turn motor with appropriate settings.
   *
   * @param turnConfig The turn motor configuration to modify
   */
  private void configureTurnMotor(TalonFXConfiguration turnConfig) {
    // Configure neutral mode based on drift mode
    turnConfig.MotorOutput.NeutralMode =
        RobotContainer.isDriftModeActive
            ? NeutralModeValue.Brake // More resistance in drift mode
            : NeutralModeValue.Coast; // Normal operation

    turnConfig.Slot0 = constants.SteerMotorGains;
    turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
    turnConfig.Feedback.FeedbackSensorSource =
        switch (constants.FeedbackSource) {
          case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
          case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
          case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
          default -> throw new RuntimeException(
              "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
        };
    turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;

    // Adjust motion magic parameters for drift mode
    double cruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
    double acceleration = cruiseVelocity / 0.100;

    if (RobotContainer.isDriftModeActive) {
      // Faster rotation response in drift mode
      cruiseVelocity *= 1.5;
      acceleration *= 2.0;
    }

    turnConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
    turnConfig.MotionMagic.MotionMagicAcceleration = acceleration;
    turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.Inverted =
        constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Refresh all signals
    var driveStatus =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    var turnStatus =
        BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent);
    var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

    // Update drive inputs
    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    // Update turn inputs
    inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
    inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnTalon.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
          case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
        });
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnTalon.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage -> positionVoltageRequest.withPosition(rotation.getRotations());
          case TorqueCurrentFOC -> positionTorqueCurrentRequest.withPosition(
              rotation.getRotations());
        });
  }

  /**
   * Updates the motor configuration when drift mode changes. This method should be called whenever
   * RobotContainer.isDriftModeActive changes.
   */
  public void updateDriftModeConfiguration() {
    // Update drive motor configuration
    var driveConfig = new MotorOutputConfigs();
    driveTalon.getConfigurator().refresh(driveConfig);

    // Set neutral mode based on drift mode and module position
    if (RobotContainer.isDriftModeActive) {
      driveConfig.NeutralMode = isFrontModule ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    } else {
      driveConfig.NeutralMode = NeutralModeValue.Brake;
    }
    driveTalon.getConfigurator().apply(driveConfig);

    // Update current limits
    var currentLimits = new CurrentLimitsConfigs();
    driveTalon.getConfigurator().refresh(currentLimits);

    if (RobotContainer.isDriftModeActive) {
      currentLimits.StatorCurrentLimit = DRIFT_CURRENT_LIMIT_STATOR;
      currentLimits.SupplyCurrentLimit = DRIFT_CURRENT_LIMIT_SUPPLY;
      currentLimits.SupplyCurrentLimitEnable = true;
    } else {
      currentLimits.StatorCurrentLimit = constants.SlipCurrent;
      currentLimits.StatorCurrentLimitEnable = true;
    }
    driveTalon.getConfigurator().apply(currentLimits);

    // Update turn motor configuration
    var turnOutputConfig = new MotorOutputConfigs();
    turnTalon.getConfigurator().refresh(turnOutputConfig);

    turnOutputConfig.NeutralMode =
        RobotContainer.isDriftModeActive ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turnTalon.getConfigurator().apply(turnOutputConfig);
  }
}
