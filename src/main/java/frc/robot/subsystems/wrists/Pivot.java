package frc.robot.subsystems.wrists;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.wrist.WristIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final WristIOTalonFX io1;
  private final WristIOTalonFX io2;
  private final WristIOTalonFX io3;
  private final WristIOInputsAutoLogged inputs1 = new WristIOInputsAutoLogged();
  private final WristIOInputsAutoLogged inputs2 = new WristIOInputsAutoLogged();
  private final WristIOInputsAutoLogged inputs3 = new WristIOInputsAutoLogged();

  private final WristConstants constants;
  private final String key = "RealOutputs/Pivot";
  private static Pivot instance;

  public static Pivot getInstance() {
    if (instance == null) {
      instance = new Pivot();
    }
    return instance;
  }

  public Pivot() {
    this.constants = WristConstants.Pivot;
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
  
  public void runCharacterization(double volts) {
    io1.runCharacterization(volts);
    io2.runCharacterization(volts);
    io3.runCharacterization(volts);
  }

  public void goToPosition(double position) {
    this.io1.goToPosition(position);
    this.io2.goToPosition(position);
    this.io3.goToPosition(position);
  }

  public double getWristPosition() {
    return inputs1.wristPositionRot;
  }
}
