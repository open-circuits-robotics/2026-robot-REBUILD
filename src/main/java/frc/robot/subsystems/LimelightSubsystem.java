package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
    private double tx;
    private double ty;
    private boolean tv;
    private double fidID;
    private final String name = "limelight-lefty";

    public LimelightSubsystem(){

    }

    @Override
    public void periodic() {
        tx = LimelightHelpers.getTX(name);
        ty = LimelightHelpers.getTY(name);
        fidID = LimelightHelpers.getFiducialID(name);
        tv = LimelightHelpers.getTV(name);
    }

    public double getTX() {
        return this.tx;
    }

    public double getTY() {
        return this.ty;
    }

    public double getFiducialID(){
        return this.fidID;
    }

    public boolean getTV(){
        return this.tv;
    }
}
