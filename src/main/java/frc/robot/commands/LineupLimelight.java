package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;

//IMPORTANT NOTE!
//In order to import frc.robot.LimelightHelpers,
//You need to get that file from the Limelight Vision website.
//There, there will be a GitHub link.
//Just follow the link, open the most recent file, and copy-paste the contents into a new LimelightHelpers.java
//For this import to work, make sure the LimelightHelpers.java file is in the java/frc/robot folder, with Constants.java and Main.java and the like

public class LineupLimelight extends Command {
    //variables to measure what it currently thinks it is at
    protected double pitch, yaw, distToTag;
    protected boolean hasTarget;

    //final values which are used to calculate distances. 
    //these should be edited according to robot specifications.
    protected final double wantedDist = 90.0; //distance away that you want
    protected final double vertFOV = 48.9; //vertical camera FOV. For limelight 2, 48.9
    protected final double horiFOV = 62.5; //horizontal camera FOV. For limelight 2, 62.5
    protected final double mountingAngleDegrees = 0; //angle the camera is mounted at (unit circle style)
    public final double  targetHeight = 30.5; //height of the april tag in question
    public final double cameraHeight = 7.5; //height of the camera once mounted on the robot
    protected final double targetID = 2; //id of the target that the camera is meant to look for
    protected final double acceptableLRRange = 20; //robot will not re-angle if it is facing target april tag within this many degrees
    protected final double acceptableDistRange = 2; //robot will not move forward/backward if it is near wantedDist, within this distance
    
    private final LimelightSubsystem limelightSubsystem; //the subsystem for the command to work with

    public LineupLimelight(LimelightSubsystem limelightSubsystem){
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(limelightSubsystem);
    }
    

    @Override
    public void execute(){
        hasTarget = limelightSubsystem.getTV(); //determines if there is a target on camera
        if (hasTarget && limelightSubsystem.getFiducialID() == targetID){ //if there is, and it's the right # target, proceed
            double tx = limelightSubsystem.getTX(); //gets x and y offsets of target (i think)
            double ty = limelightSubsystem.getTY();
            pitch = (-ty/2)*vertFOV + 360; //calculates the pitch (tilt up/down)of the camera
            yaw = (tx/2)*horiFOV; //calculates the yaw (turn left/right) of the camera
            if (yaw > 0 + Math.toRadians(acceptableLRRange)){
               // System.out.println("go left");
            } else if (yaw < 0 - Math.toRadians(acceptableLRRange)){
               // System.out.println("go right");
            }
            distToTag = (targetHeight - cameraHeight)/Math.tan(Math.toRadians(mountingAngleDegrees + pitch)); //calculates horizontal ground distance between target and camera
            distToTag = (targetHeight - cameraHeight)/Math.tan(Math.toRadians(mountingAngleDegrees + ty)); //calculates horizontal ground distance between target and camera
            System.out.println("pitch is " + pitch + ", distance is " + distToTag);
            if (distToTag > wantedDist + acceptableDistRange){
                //System.out.println("go forward");
            } else if (distToTag < wantedDist - acceptableDistRange){
             //  System.out.println("go backward");
            }
        }
    }

}
