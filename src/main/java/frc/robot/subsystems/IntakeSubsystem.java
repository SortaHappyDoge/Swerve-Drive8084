package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX motor;

            
        
        
            public IntakeSubsystem() {
                // 1. Motor tanımla
                //motor = new SparkMax(20, MotorType.kBrushless);
                motor = new TalonFX(20);  
    
            // 2. Config nesnesi oluştur
            //SparkMaxConfig config = new SparkMaxConfig();
    
            // 3. PID Encoder yapılandırması
            
    
            // İstersen motoru hemen sür
            motor.set(0);
        }

        
    
        

    public void setMotor(double speed) {
      motor.set(speed);
    }

    
}
