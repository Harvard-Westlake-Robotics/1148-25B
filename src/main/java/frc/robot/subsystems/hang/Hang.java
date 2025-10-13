package frc.robot.subsystems.hang;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hang extends SubsystemBase {
  private HangIO io;
  private static Hang instance = null;
  private HangIOInputsAutoLogged inputs = new HangIOInputsAutoLogged(); // This is built wrong so it's fine
  private String key;
  private boolean hasBar = false;
  public Servo servo = new Servo(0);

  // Gets/creates the currently used instance of the hang 
  public static Hang getInstance() {
    if (instance == null) {
      instance = new Hang();
    }
    return instance;
  }

  // Accessor that returns if the hang sensor sees the bar
  public Boolean getHasBar() {
    return hasBar;
  }

  // Modifier that sets when the hang sensor sees the bar
  public void setHasBar(Boolean hasBar) {
    this.hasBar = hasBar;
  }

   // Constructor that creates the hang
  public Hang() {
    this.key = "RealOutputs/Hang";
    io = new HangIOTalonFX();
  }

  // Logs the inputs to the logger (which is wrong)
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(key, inputs);
    hasBar = inputs.motorAppliedVolts > 1.0 && inputs.motorVelocityMPS < 0.2;
  }

  // Sets the hang's velocity
  public void setVelocity(LinearVelocity velocity) {
    io.runVelocity(velocity);
  }

  // Sets the hang's voltage
  public void setVoltage(double volts) {
    io.runCharacterization(volts);
  }
}
