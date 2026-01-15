package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class Autonomous extends Command {
    Timer timer;

    public Autonomous(){
        timer = new Timer();
    }

    @Override
    public void initialize(){
        timer.start();
    }

    @Override
    public void execute(){
        if (timer.get() < 3){
            //first three seconds
        } else if (timer.get() < 10){
            //first ten seconds
        } else {
            //final five seconds
        }
    }


    @Override
    public boolean isFinished(){
        return timer.get() >= 15;
    }
}
