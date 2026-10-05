package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommandTimed extends ShooterCommand{

    private Timer timer;
    double time;

    public ShooterCommandTimed (double seconds, ShooterSubsystem shooter){
        super(shooter);
        timer = new Timer();
        time = seconds;
    }

    @Override
    public void initialize(){
        super.initialize();
        timer.restart();
    }

    @Override
    public boolean isFinished(){
        return timer.get() >= time;
    }
    
}
