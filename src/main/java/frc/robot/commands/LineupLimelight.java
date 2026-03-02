package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

//IMPORTANT NOTE!
//In order to import frc.robot.LimelightHelpers,
//You need to get that file from the Limelight Vision website.
//There, there will be a GitHub link.
//Just follow the link, open the most recent file, and copy-paste the contents into a new LimelightHelpers.java
//For this import to work, make sure the LimelightHelpers.java file is in the java/frc/robot folder, with Constants.java and Main.java and the like

public class LineupLimelight extends Command {

    protected SwerveSubsystem swerveSubsystem;

    //do you want it to be precise distance measurements (doesn't work) or relative location measurements (much better)?
    protected final boolean preciseDist = false;

    //If in relative location mode, are you calibrating? or just fully using it?
    protected final boolean calibrationMode = false;

    //variables to measure what it currently thinks it is at
    protected double pitch, yaw, distToTag;
    protected boolean hasTarget;

    //These values are only used for the (non functional) precise distance function.
    protected final double wantedDist = 90.0; //distance away that you want
    protected final double vertFOV = 48.9; //vertical camera FOV. For limelight 2, 48.9
    protected final double horiFOV = 62.5; //horizontal camera FOV. For limelight 2, 62.5
    protected final double mountingAngleDegrees = 0; //angle the camera is mounted at (unit circle style)
    public final double  targetHeight = 30.5; //height of the april tag in question
    public final double cameraHeight = 7.5; //height of the camera once mounted on the robot
    protected final double acceptableDistRange = 2; //robot will not move forward/backward if it is near wantedDist, within this distance

    //these vlaues are used only for the relative location function
    protected final double locX = -2;
    protected final double locY = 16;
    protected final double acceptableUDRange = 1;


    //these values are used for both that precise distance function and the better relative location function
    protected final double targetID = 2; //id of the target that the camera is meant to look for
    protected final double acceptableLRRange = 5; //robot will not re-angle if it is facing target april tag within this many degrees

    //for driving, take same value as in SwerveCommand
    private double speedConstant = 0.75;
    private double turnConstant = 0.125;
    
    private final LimelightSubsystem limelightSubsystem; //the subsystem for the command to work with

    public LineupLimelight(LimelightSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(limelightSubsystem);
    }

    @Override
    public void execute(){
        if (preciseDist){
            usePreciseDistanceMeasurements();
        } else {
            useRelativeLocationMeasurements();
        }
    }

    public void useRelativeLocationMeasurements(){
        hasTarget = limelightSubsystem.getTV(); //determines if there is a target on camera
        if (hasTarget && limelightSubsystem.getFiducialID() == targetID){ //if there is a target on camera, and the target's id number is the correct one, proceed with the code
            double tx = limelightSubsystem.getTX(); //gets x location of the target on the camera
            double ty = limelightSubsystem.getTY(); //gets y location of the target on the camera
            if (calibrationMode){ //if the robot is in calibration mode, prints out the x and y locations of the target on the camera, then ends the method
                System.out.println("tx: " + tx);
                System.out.println("ty: " + ty);
                return;
            } //otherwise, proceeds to determine for both tx and ty whether they are within range, and gives directions to move accordingly
            if (tx > locX + Math.toRadians(acceptableLRRange)){
                System.out.println("go right");
                adjustRobotLeftRight(Math.min(1,(tx - locX)/50));
                return;
            } else if (tx < locX - Math.toRadians(acceptableLRRange)){
                System.out.println("go left");
                adjustRobotLeftRight(Math.max(-1,(tx - locX)/50));
                return;
            } else {
                System.out.println("in LR range");
            }
            if (ty > locY + Math.toRadians(acceptableUDRange)){
                System.out.println("go backward");
                adjustRobotForwardBackward(Math.min(1, (ty - locY)/50));
            } else if (ty < locY - Math.toRadians(acceptableUDRange)){
                System.out.println("go forward");
                adjustRobotForwardBackward(Math.max(-1, (ty-locY)/50));
            } else {
                System.out.println("in UD range");
            }
        }
    }

    public void adjustRobotForwardBackward(double amt){
        Translation2d translation = new Translation2d(0, amt * speedConstant);
        swerveSubsystem.drive(translation, 0, false);
    }

    public void adjustRobotLeftRight(double amt){
        Translation2d translation = new Translation2d(0, 0);
        swerveSubsystem.drive(translation, amt*turnConstant, true);
    }
    

    public void usePreciseDistanceMeasurements(){
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
            if (distToTag > wantedDist + acceptableUDRange){
                //System.out.println("go forward");
            } else if (distToTag < wantedDist - acceptableUDRange){
             //  System.out.println("go backward");
            }
        }
    }

}
