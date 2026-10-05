package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class LineupLimelightTimed extends LineupLimelight{

    Timer timer;

    public LineupLimelightTimed(LimelightSubsystem limelight, SwerveSubsystem swerve){
        super(limelight, swerve);
        timer = new Timer();
    }

    @Override
    public void initialize(){
        timer.restart();
    }

    @Override
    public boolean isFinished(){
        return timer.get() >= 5;
    }
    
}
