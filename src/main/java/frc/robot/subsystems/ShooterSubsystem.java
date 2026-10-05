package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{

    private SparkFlex forwardSameChainNoBrake, forward, backward, forwardSameChain;
    private double shooterSpeed;
    private SparkFlex motor;
    private final double defaultShooterSpeed = .45;
    
    public ShooterSubsystem(){
        forwardSameChainNoBrake = new SparkFlex(11, MotorType.kBrushless);
        forward = new SparkFlex(12, MotorType.kBrushless);
        backward = new SparkFlex(13, MotorType.kBrushless);
        forwardSameChain = new SparkFlex(14, MotorType.kBrushless);
        SmartDashboard.putString("DB/String 2", "Slider 2: Shooter Speed");
    }

    public void shoot(){
        shooterSpeed = SmartDashboard.getNumber("DB/Slider 2", 0);
        if (shooterSpeed == 0.0){
            shooterSpeed = defaultShooterSpeed;
        }
        forwardSameChainNoBrake.set(shooterSpeed * 0.3);
        forward.set(-shooterSpeed * 0.75);
        backward.set(shooterSpeed);
        forwardSameChain.set(-shooterSpeed * 0.3);
    }    

    public void stop(){
        forwardSameChain.set(0);
        forwardSameChainNoBrake.set(0);
        forward.set(0);
        backward.set(0);
    }
    
}
