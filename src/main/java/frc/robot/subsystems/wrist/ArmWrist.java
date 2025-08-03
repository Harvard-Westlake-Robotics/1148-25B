package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmWrist extends SubsystemBase {
  private final WristIOTalonFX io1;
  private final WristIOTalonFX io2;
  private final WristIOTalonFX io3;
  private final WristIOInputsAutoLogged inputs1 = new WristIOInputsAutoLogged();
  private final WristIOInputsAutoLogged inputs2 = new WristIOInputsAutoLogged();
  private final WristIOInputsAutoLogged inputs3 = new WristIOInputsAutoLogged();
  private WristConstants constants;
  private String key;
  private static ArmWrist instance;

  public static ArmWrist getInstance() {
    if (instance == null) {
      instance = new ArmWrist();
    }
    return instance;
  }

  public ArmWrist() {
    this.constants = WristConstants.ArmWrist;
    this.key = "RealOutputs/Arm Wrist";
    io1 = new WristIOTalonFX(constants, 1);
    io2 = new WristIOTalonFX(constants, 2);
    io3 = new WristIOTalonFX(constants, 3);
  }

  public WristConstants getConstants() {
    return constants;
  }

  public void periodic() {
    io1.updateInputs(inputs1);
    Logger.processInputs(key + "/Motor 1", inputs1);
    io2.updateInputs(inputs2);
    Logger.processInputs(key + "/Motor 2", inputs2);
    io3.updateInputs(inputs3);
    Logger.processInputs(key + "/Motor 3", inputs3);
  }

  public void goToAngle(double angle) {
    this.io1.setAngle(angle);
    this.io2.setAngle(angle);
    this.io3.setAngle(angle);
  }

  public void runVoltage(double volts) {
    io1.runCharacterization(volts);
    io2.runCharacterization(volts);
    io3.runCharacterization(volts);
  }

  public double getWristPosition(int ioNum) {
    if (ioNum == 1) {
      return inputs1.wristPositionMeters;
    } else if (ioNum == 2) {
      return inputs2.wristPositionMeters;
    } else if (ioNum == 3) {
      return inputs3.wristPositionMeters;
    } else {
      return 0;
    }
  }
}
