package frc.robot.subsystems.wrists;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.wrist.WristIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIOTalonFX io1;
  private final WristIOTalonFX io2;
  private final WristIOInputsAutoLogged inputs1 = new WristIOInputsAutoLogged();
  private final WristIOInputsAutoLogged inputs2 = new WristIOInputsAutoLogged();

  private final WristConstants constants;
  private final String key = "RealOutputs/Wrist";
  private static Wrist instance;

  public static Wrist getInstance() {
    if (instance == null) {
      instance = new Wrist();
    }
    return instance;
  }

  public Wrist() {
    this.constants = WristConstants.Wrist;
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
  
  public void runCharacterization(double volts) {
    io1.runCharacterization(volts);
    io2.runCharacterization(volts);
  }

  public void goToPosition(double position) {
    this.io1.goToPosition(position);
    this.io2.goToPosition(position);
  }

  public double getWristPosition() {
    return inputs1.wristPositionRot;
  }
}
