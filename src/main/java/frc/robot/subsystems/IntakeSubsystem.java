package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final double intakeSpeed = 1;
    private SparkFlex upper, lower;

    public IntakeSubsystem(){
        upper = new SparkFlex(9, MotorType.kBrushless);
        lower = new SparkFlex(10, MotorType.kBrushless);
    }

    //assuming "dir" is either 1 or -1 to specify direction from the Command
    public void drive(int dir){
        upper.set(intakeSpeed*dir);
        lower.set(-intakeSpeed*dir);
    }

    public void stop(){
        upper.set(0);
        lower.set(0);
    }

    
    
}
