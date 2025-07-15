package frc.robot.subsystems.hang;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hang extends SubsystemBase {
  private HangIO io;
  private static Hang instance = null;
  private HangIOInputsAutoLogged inputs = new HangIOInputsAutoLogged();
  private String key;
  private boolean hasBar = false;

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
    this.key = "Hang";
    io = new HangIOTalonFX();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(key, inputs);
    hasBar = inputs.motorAppliedVolts > 1.0 && inputs.motorVelocityMPS < 0.2;
  }

  public void setVelocity(LinearVelocity velocity) {
    io.runVelocity(velocity);
  }

  public void setVoltage(double volts) {
    io.runCharacterization(volts);
  }
}
