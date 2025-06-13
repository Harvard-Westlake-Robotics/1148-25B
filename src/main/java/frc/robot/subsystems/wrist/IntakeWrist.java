package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeWrist extends SubsystemBase {
  private final WristIOTalonFX io1;
  private final WristIOTalonFX io2;
  private final WristIOInputsAutoLogged inputs1 = new WristIOInputsAutoLogged();
  private final WristIOInputsAutoLogged inputs2 = new WristIOInputsAutoLogged();
  private WristConstants constants;
  private String key;
  private static IntakeWrist instance;

  public static IntakeWrist getInstance() {
    if (instance == null) {
      instance = new IntakeWrist();
    }
    return instance;
  }

  public IntakeWrist() {
    this.constants = Constants.ArmWrist;
    this.key = "Intake Wrist";
    io1 = new WristIOTalonFX(constants, 1);
    io2 = new WristIOTalonFX(constants, 2);
  }

  public WristConstants getConstants() {
    return constants;
  }

  public void periodic() {
    io1.updateInputs(inputs1);
    Logger.processInputs(key + "/Motor 1", inputs1);
    io2.updateInputs(inputs2);
    Logger.processInputs(key + "/Motor 2", inputs2);
  }

  public void goToAngle(double angle) {
    this.io1.setAngle(angle);
    this.io2.setAngle(angle);
  }

  public void runVoltage(double volts) {
    io1.runCharacterization(volts);
    io2.runCharacterization(volts);
  }

  public double getWristPosition(int ioNum) {
    if (ioNum == 1) {
      return inputs1.wristPositionMeters;
    } else if (ioNum == 2) {
      return inputs2.wristPositionMeters;
    } else {
      return 0;
    }
  }
}
