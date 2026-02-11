package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
    private double tx;
    private double ty;

    public LimelightSubsystem(){
        
    }

    @Override
    public void periodic() {
        tx = LimelightHelpers.getTX("limelight-lefty");
        ty = LimelightHelpers.getTY("limelight-lefty");
    }

    public double getTX() {
        return this.tx;
    }

    public double getTY() {
        return this.ty;
    }
}
