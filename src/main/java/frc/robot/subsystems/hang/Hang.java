package frc.robot.subsystems.hang;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hang extends SubsystemBase {
  private final HangIO io;
  private static Hang instance = null;
  private final HangIOInputsAutoLogged inputs = new HangIOInputsAutoLogged();
  private boolean hasBar = false;
  public Servo servo = new Servo(0);

  public static Hang getInstance() {
    if (instance == null) {
      instance = new Hang();
    }
    return instance;
  }

  public Boolean getHasBar() {
    return hasBar;
  }

  public void setHasBar(Boolean hasBar) {
    this.hasBar = hasBar;
  }

  public Hang() {
    io = new HangIOTalonFX();
  }

  public void periodic() {
    io.updateInputs(inputs);
    // TODO: What is "RealOutputs"?
    Logger.processInputs("RealOutputs/Hang", inputs);
    hasBar = inputs.motorAppliedVolts > 1.0 && inputs.motorVelocityMPS < 0.2;
  }

  public void setVelocity(LinearVelocity velocity) {
    io.runVelocity(velocity);
  }

  public void setVoltage(double volts) {
    io.runCharacterization(volts);
  }
}
