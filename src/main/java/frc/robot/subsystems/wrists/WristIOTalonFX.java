package frc.robot.subsystems.wrists;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.WristConstants;

public class WristIOTalonFX implements WristIO {
  // Motors and wrist controllers
  private TalonFX wristMotor;
  private MotionMagicVoltage wristController;

  private ArmFeedforward wristFeedForward;

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
    // System.out.println(constants.angleOffset.baseUnitMagnitude());
    System.out.println(Rotations.of(11.98));
    wristMotor.setPosition(constants.angleOffset);
    // System.out.println(constants.angleOffset.in(Rotations));
    this.wristController = new MotionMagicVoltage(0).withEnableFOC(true);
    wristConfig = new TalonFXConfiguration();
    wristConfig.MotorOutput.Inverted = constants.motorInverted;
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    wristConfig.Slot0.kP = constants.kP;
    wristConfig.Slot0.kI = constants.kI;
    wristConfig.Slot0.kD = constants.kD;
    wristConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wristConfig.CurrentLimits.StatorCurrentLimit = constants.statorLimit;
    wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristConfig.CurrentLimits.SupplyCurrentLimit = constants.supplyLimit;
    this.wristConfig.MotionMagic.MotionMagicAcceleration = this.constants.motionMagicAcceleration;
    this.wristConfig.MotionMagic.MotionMagicCruiseVelocity =
        this.constants.motionMagicCruiseVelocity;
    this.wristConfig.MotionMagic.MotionMagicJerk = this.constants.motionMagicJerk;
    this.wristMotor.getConfigurator().apply(this.wristConfig);
    wristMotor.setControl(wristController);

    motorPosition = wristMotor.getPosition();
    motorVelocity = wristMotor.getVelocity();
    motorAppliedVolts = wristMotor.getMotorVoltage();
    motorCurrent = wristMotor.getStatorCurrent();

    // TODO: Add angle-specific kG changing
    wristFeedForward = new ArmFeedforward(constants.kS, constants.kG, constants.kV, constants.kA);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    StatusSignal.refreshAll(motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);

    inputs.wristMotorConnected = motorConnectedDebouncer.calculate(wristMotor.isConnected());
    inputs.wristPositionRot = motorPosition.getValueAsDouble() / constants.motorToWristRotations;
    inputs.wristVelocityRPS = motorVelocity.getValueAsDouble() / constants.motorToWristRotations;
    inputs.wristAppliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.wristCurrentAmps = motorCurrent.getValueAsDouble();
  }

  @Override
  public void runCharacterization(double voltage) {
    wristMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void goToAngleClosedLoop(double angle, double offsetAngle) {
    wristMotor.setControl(
        wristController
            .withPosition(angle * constants.motorToWristRotations)
            .withFeedForward(
                wristFeedForward.calculate(Units.rotationsToRadians(angle + offsetAngle), 0)));
  }

  @Override
  public void tareAngle(double angle) {
    wristMotor.setPosition(angle * constants.motorToWristRotations);
  }
}
