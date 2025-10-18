package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
  // Motors and intake controllers
  private final TalonFX intakeMotor;
  private final MotionMagicVelocityTorqueCurrentFOC intakeController;

  private final IntakeConstants constants;

  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;

  private final CANrange input1;
  private final CANrange input2;
  private final CANrange input3;
  private final CANrange input4;

  // Connection debouncers
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);

  public IntakeIOTalonFX(IntakeConstants constants, int motorNum) {
    this.constants = constants;
    intakeMotor = new TalonFX(constants.motorId + motorNum - 1);
    intakeMotor.setPosition(0);
    intakeController =
        new MotionMagicVelocityTorqueCurrentFOC(
            AngularVelocity.ofBaseUnits(0.0, RotationsPerSecond));
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

    intakeConfig.MotorOutput.Inverted = constants.motorInverted;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.MotionMagic.MotionMagicAcceleration = constants.motionMagicAcceleration;
    intakeConfig.MotionMagic.MotionMagicCruiseVelocity = constants.motionMagicCruiseVelocity;

    intakeConfig.Slot0.kP = constants.kP;
    intakeConfig.Slot0.kI = constants.kI;
    intakeConfig.Slot0.kD = constants.kD;
    intakeConfig.Slot0.kS = constants.kS;
    intakeConfig.Slot0.kV = constants.kV;
    intakeConfig.Slot0.kA = constants.kA;
    intakeConfig.Slot1.kP = constants.positionkP;
    intakeConfig.Slot1.kD = constants.positionkD;

    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimit = constants.statorLimit;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = constants.supplyLimit;
    intakeMotor.getConfigurator().apply(intakeConfig);
    intakeMotor.setControl(intakeController);

    motorPosition = intakeMotor.getPosition();
    motorVelocity = intakeMotor.getVelocity();
    motorAppliedVolts = intakeMotor.getMotorVoltage();
    motorCurrent = intakeMotor.getStatorCurrent();

    if (motorNum == 1) {
      this.input1 = constants.sensor1ID != -1 ? new CANrange(constants.sensor1ID) : null;
      this.input2 = constants.sensor2ID != -1 ? new CANrange(constants.sensor2ID) : null;
      this.input3 = constants.sensor3ID != -1 ? new CANrange(constants.sensor3ID) : null;
      this.input4 = constants.sensor4ID != -1 ? new CANrange(constants.sensor4ID) : null;
    } else {
      this.input1 = null;
      this.input2 = null;
      this.input3 = null;
      this.input4 = null;
    }
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    StatusSignal.refreshAll(motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);

    inputs.intakeMotorConnected = motorConnectedDebouncer.calculate(intakeMotor.isConnected());
    inputs.intakeMotorPositionMeters =
        motorPosition.getValueAsDouble() / constants.rotationsToMetersRatio;
    inputs.intakeMotorVelocityMPS =
        motorVelocity.getValueAsDouble() / constants.rotationsToMetersRatio;
    inputs.intakeMotorAppliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.intakeMotorCurrentAmps = motorCurrent.getValueAsDouble();
  }

  @Override
  public void runCharacterization(double voltage) {
    intakeMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void runVelocity(LinearVelocity velocity) {
    intakeMotor.setControl(
        intakeController.withVelocity(
            velocity.in(MetersPerSecond) * constants.rotationsToMetersRatio));
  }

  public Boolean getSensor1() {
    if (input1 == null) return null;
    var distanceSignal = input1.getDistance();
    distanceSignal.refresh();
    double distance = distanceSignal.getValueAsDouble();
    if (distance != 0 && distance <= 0.02) {
      return true;
    } else {
      return false;
    }
  }

  public Boolean getSensor2() {
    if (input2 == null) return null;
    var distanceSignal = input2.getDistance();
    distanceSignal.refresh();
    double distance = distanceSignal.getValueAsDouble();
    if (distance != 0 && distance <= 0.02) {
      return true;
    } else {
      return false;
    }
  }

  public Boolean getSensor3() {
    if (input3 == null) return null;
    var distanceSignal = input3.getDistance();
    distanceSignal.refresh();
    double distance = distanceSignal.getValueAsDouble();
    if (distance != 0 && distance <= 0.02) {
      return true;
    } else {
      return false;
    }
  }

  public Boolean getSensor4() {
    if (input4 == null) return null;
    var distanceSignal = input4.getDistance();
    distanceSignal.refresh();
    double distance = distanceSignal.getValueAsDouble();
    if (distance != 0 && distance <= 0.02) {
      return true;
    } else {
      return false;
    }
  }
}
