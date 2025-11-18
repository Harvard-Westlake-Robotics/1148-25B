package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;

public class Vision extends SubsystemBase {
    private static Vision instance;

    private String key = "Vision";

    @AutoLogOutput
    public double sdMultiplier = 1;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }
}
