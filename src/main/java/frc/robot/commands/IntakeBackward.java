package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeBackward extends Command {

    IntakeSubsystem intake;

    public IntakeBackward(IntakeSubsystem intake){
        this.intake = intake;
        addRequirements(intake);
    }



    @Override
    public void initialize(){
        intake.drive(-1);
    }

    @Override
    public void end (boolean interrupted){
        intake.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}