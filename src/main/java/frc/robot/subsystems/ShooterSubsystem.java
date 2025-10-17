package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.hal.MatchInfoData;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX motor;
    private final TalonFX ballDropper;
    public PIDController ballController = new PIDController(0.4, 0, 0);
    public double stickSpeedMultiplier = 0.4;
    public DutyCycleEncoder ballEncoder = new DutyCycleEncoder(9, Math.PI*2, Math.PI);
    public double[] ballEncoderRange = {0.59, 4.3};
    public double ballStickPos;
    public double[] ballStickPoses = {0.66, 3, 3.2};
    public double desiredBallStickRotation = ballStickPoses[0];
    
            public ShooterSubsystem() { 
                // 1. Motor tanımla
                motor = new TalonFX(30);
                ballDropper= new TalonFX(41);
                //ballDropper.getDutyCycle;
            // 2. Config nesnesi oluştur
            SparkMaxConfig config = new SparkMaxConfig();
                
            // 3. PID Encoder yapılandırması
            
    
            // İstersen motoru hemen sür
            motor.set(0.0);  // %50 hızla ileri döner
        }

        
    
        
        @Override
        public void periodic(){
            ballStickPos = getStickPos();

            ballDropper.set(-ballController.calculate(ballStickPos, desiredBallStickRotation)*stickSpeedMultiplier);
            //System.out.println(ballStickPos);
        }
    public void setMotor(double speed) {
      motor.set(speed);
    }
    
    public void setDesiredStickPos(double target){
        desiredBallStickRotation = MathUtil.clamp(target, ballEncoderRange[0], ballEncoderRange[1]);
    }
    public double getStickPos(){
        return ballEncoder.get();
    }
}
