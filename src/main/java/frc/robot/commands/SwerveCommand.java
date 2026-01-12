package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveCommand extends Command {
    
    public final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY, headingAdjust;
    private double speedConstant = 0.75;
    private double turnConstant = 0.125;

    public SwerveCommand(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingAdjust){

        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.headingAdjust = headingAdjust;
        addRequirements(swerve);

    }

    @Override
    public void execute(){
        Translation2d translation = new Translation2d(vX.getAsDouble()*speedConstant, vY.getAsDouble() * speedConstant);
        swerve.drive(translation, (turnConstant*headingAdjust.getAsDouble()), false);
    }
   
}
