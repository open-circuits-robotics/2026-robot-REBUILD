package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import java.lang.Math;

public class SwerveCommand extends Command {
    
    public final SwerveSubsystem swerve;
    private final double THRESHOLD = 0.2;
    private final DoubleSupplier vX, vY, headingAdjust;
    private double speedConstant = 3.0;
    private double turnConstant = 0.8;

    public SwerveCommand(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingAdjust){

        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.headingAdjust = headingAdjust;
        addRequirements(swerve);

    }

    /**
     * if the input parameter is closer to 0 than the deadband, returns 0
     * otherwise, returns the same input parameter
     * @param in the value that the deadband should apply to
     * @param threshold the distance from 0 that the joystick should be deadbanded to
     * @return the correct value for the joystick after the deadband is applied
    */

    public double applyDeadband(double in, double threshold) {
        if (Math.abs(in) < threshold) {
            return 0;
        } else {
            return in;
        }
    }

    @Override
    public void execute(){
        speedConstant = SmartDashboard.getNumber("DB/Slider 0", 0);
        turnConstant  = SmartDashboard.getNumber("DB/Slider 1", 0);
        Translation2d translation = new Translation2d(
            applyDeadband(vX.getAsDouble(), THRESHOLD) * -speedConstant, 
            applyDeadband(vY.getAsDouble(), THRESHOLD) * -speedConstant
            );
        System.out.println(translation);
        swerve.drive(
            translation, 
            (applyDeadband(headingAdjust.getAsDouble(), THRESHOLD) * -turnConstant), false
            );
    }
   
}
