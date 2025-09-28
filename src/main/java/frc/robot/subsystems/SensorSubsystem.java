package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SensorSubsystem extends SubsystemBase{
    public static AnalogInput analogInput;

    public SensorSubsystem(){
        analogInput = new AnalogInput(3);
    }

    public boolean getState() {
        return analogInput.getVoltage() < 4;  // Volt cinsinden değer
    }
}
