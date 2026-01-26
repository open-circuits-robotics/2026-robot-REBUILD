package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

//IMPORTANT NOTE!
//In order to import frc.robot.LimelightHelpers,
//You need to get that file from the Limelight Vision website.
//There, there will be a GitHub link.
//Just follow the link, open the most recent file, and copy-paste the contents into a new LimelightHelpers.java
//For this import to work, make sure the LimelightHelpers.java file is in the java/frc/robot folder, with Constants.java and Main.java and the like

public class LineupLimelight extends Command {
    protected double pitch, yaw, distToTag;
    protected final double wantedDist = 2;
    protected final double fov = 48.9;
    protected boolean hasTarget;
    protected final String name = "limelight-lefty";

    public LineupLimelight(){
    }

    @Override
    public void execute(){
        hasTarget = LimelightHelpers.getTV(name);
        if (hasTarget){
            double tx = LimelightHelpers.getTX(name);
            double ty = LimelightHelpers.getTY(name);
        }
    }

}
