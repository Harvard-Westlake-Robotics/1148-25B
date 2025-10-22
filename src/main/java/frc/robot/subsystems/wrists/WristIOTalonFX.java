package frc.robot.subsystems.wrists;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.WristConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.Logger;

public class WristIOTalonFX implements WristIO {
  // Motors and wrist controllers
  private TalonFX wristMotor;
  private MotionMagicVoltage wristController;

  private WristConstants constants;
  private TalonFXConfiguration wristConfig;

  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;

  // Connection debouncers
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);

  public WristIOTalonFX(WristConstants constants, int motorNum, String canbus) {
    this.constants = constants;
    wristMotor = new TalonFX(constants.motorId + motorNum - 1, canbus);
    wristMotor.setPosition(constants.angleOffset);
    this.wristController =
        new MotionMagicVoltage(0).withEnableFOC(true).withPosition(constants.angleOffset);
    wristConfig = new TalonFXConfiguration();
    wristConfig.MotorOutput.Inverted = constants.motorInverted;
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    wristConfig.Slot0.kP = constants.kP;
    wristConfig.Slot0.kI = constants.kI;
    wristConfig.Slot0.kD = constants.kD;
    wristConfig.Slot0.kS = constants.kS;
    wristConfig.Slot0.kV = constants.kV;
    wristConfig.Slot0.kA = constants.kA;
    wristConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wristConfig.CurrentLimits.StatorCurrentLimit = constants.statorLimit;
    wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristConfig.CurrentLimits.SupplyCurrentLimit = constants.supplyLimit;
    // wristConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // wristConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
    //     constants.wristMaxAngle.in(Rotations);
    // wristConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // wristConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
    //     constants.wristMinAngle.in(Rotations) * constants.motorRotationsPerWristRotationRatio;
    // System.out.println(
    //     constants.wristMinAngle.in(Rotations) * constants.motorRotationsPerWristRotationRatio);
    this.wristConfig.MotionMagic.MotionMagicAcceleration = this.constants.motionMagicAcceleration;
    this.wristConfig.MotionMagic.MotionMagicCruiseVelocity = this.constants.motionMagicCruiseVelocity;
    this.wristConfig.MotionMagic.MotionMagicJerk = this.constants.motionMagicJerk;
    this.wristMotor.getConfigurator().apply(this.wristConfig);
    wristMotor.setControl(wristController);

    motorPosition = wristMotor.getPosition();
    motorVelocity = wristMotor.getVelocity();
    motorAppliedVolts = wristMotor.getMotorVoltage();
    motorCurrent = wristMotor.getStatorCurrent();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    StatusSignal.refreshAll(motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);

    inputs.wristMotorConnected = motorConnectedDebouncer.calculate(wristMotor.isConnected());
    inputs.wristPositionRot =
        motorPosition.getValueAsDouble() / constants.motorRotationsPerWristRotationRatio;
    inputs.wristVelocityRPS =
        motorVelocity.getValueAsDouble() / constants.motorRotationsPerWristRotationRatio;
    inputs.wristAppliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.wristCurrentAmps = motorCurrent.getValueAsDouble();

    // TODO: use key and in Wrist.java instead
    Logger.recordOutput("RealOutputs/Wrist/motionMagicState", wristController.toString());
  }

  @Override
  public void runCharacterization(double voltage) {
    // wristMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void goToAngleClosedLoop(double angle) {
    // wristMotor.setControl(
    //     wristController
    //         .withPosition(angle * constants.motorRotationsPerWristRotationRatio));
  }

  @Override
  public void tareAngle(double angle) {
    wristMotor.setPosition(angle * constants.motorRotationsPerWristRotationRatio);
  }
}
