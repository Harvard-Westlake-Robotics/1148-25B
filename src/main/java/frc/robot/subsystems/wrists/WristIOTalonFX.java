package frc.robot.subsystems.wrists;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.WristConstants;

public class WristIOTalonFX implements WristIO {
  // Motors and wrist controllers
  private TalonFX wristMotor;
  private MotionMagicVoltage wristController;

  private WristConstants wristConstants;

  private TalonFXConfiguration wristConfig;

  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;

  // Connection debouncers
  private final Debouncer motorConnectedDebounce = new Debouncer(0.5);

  public WristIOTalonFX(WristConstants wristConstants, int motorNum) {
    this.wristConstants = wristConstants;
    wristMotor = new TalonFX(wristConstants.motorId + motorNum - 1);
    wristMotor.setPosition(wristConstants.angleOffset);
    this.wristController = new MotionMagicVoltage(0).withEnableFOC(true);
    wristConfig = new TalonFXConfiguration();
    wristConfig.MotorOutput.Inverted = wristConstants.motorInverted;
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    wristConfig.Slot0.kP = wristConstants.kP;
    wristConfig.Slot0.kI = wristConstants.kI;
    wristConfig.Slot0.kD = wristConstants.kD;
    wristConfig.Slot0.kS = wristConstants.kS;
    wristConfig.Slot0.kG = wristConstants.kG;
    wristConfig.Slot0.kV = wristConstants.kV;
    wristConfig.Slot0.kA = wristConstants.kA;
    wristConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wristConfig.CurrentLimits.StatorCurrentLimit = wristConstants.statorLimit;
    wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristConfig.CurrentLimits.SupplyCurrentLimit = wristConstants.supplyLimit;
    this.wristConfig.MotionMagic.MotionMagicAcceleration =
        this.wristConstants.motionMagicAcceleration;
    this.wristConfig.MotionMagic.MotionMagicCruiseVelocity = this.wristConstants.motionMagicCruiseVelocity;
    this.wristConfig.MotionMagic.MotionMagicJerk = this.wristConstants.motionMagicJerk;
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
    
    inputs.wristMotorConnected = motorConnectedDebounce.calculate(wristMotor.isConnected());
    inputs.wristPositionRot =
        motorPosition.getValueAsDouble() * (1 / wristConstants.motorToWristRotations);
    inputs.wristVelocityMPS =
        motorVelocity.getValueAsDouble() * (1 / wristConstants.motorToWristRotations);
    inputs.wristAppliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.wristCurrentAmps = motorCurrent.getValueAsDouble();
  }

  @Override
  public void runCharacterization(double voltage) {
    wristMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void goToPosition(double position) {
    wristMotor.setControl(wristController.withPosition(position / wristConstants.motorToWristRotations));
  }

  @Override
  public void setPosition(double position) {
    wristMotor.setPosition(position);
  }
}
