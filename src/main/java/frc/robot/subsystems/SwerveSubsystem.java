package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    double maximumSpeed;
    double turnSpeed;
    File swerveJsonDirectory;
    SwerveDrive swerveDrive;

    public SwerveSubsystem(){

      // Changes speed. May not work :(
      SmartDashboard.putString("DB/String 0", "Slider 0: Drive Speed");
      maximumSpeed = Units.feetToMeters(14.5);  // /1.5 in last years code (why?? test without?)

      // Swerve things
      swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
        } catch (IOException e) {
            e.printStackTrace();
        }

      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        // Changes turn speed (maybe probably doesn't work)

        SmartDashboard.putString("DB/String 1", "Slider 1: Max turn speed. Do not exceded ");
          

    }
  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                        angularRotationX.getAsDouble() * turnSpeed, // <- this is turn speed?
                        true,
                        false);
    });
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative){
    swerveDrive.drive(translation, rotation, true, false);
    }

    public Pose2d getPose(){
      return swerveDrive.getPose();
    }

    public void resetPose(Pose2d poseIn){
      swerveDrive.resetOdometry(poseIn);
    }

    public ChassisSpeeds getRobotVelocity(){
      return swerveDrive.getRobotVelocity();
    }

    // Path Planner

  public void setupPathPlanner()
    {
      // Load the RobotConfig from the GUI settings. You should probably
      // store this in your Constants file
      RobotConfig config;
      try
      {
        config = RobotConfig.fromGUISettings();

        final boolean enableFeedforward = true;
        // Configure AutoBuilder last
        AutoBuilder.configure(
            this::getPose,
            // Robot pose supplier
            this::resetPose,
            // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotVelocity,
            // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speedsRobotRelative, moduleFeedForwards) -> {
              if (enableFeedforward)
              {
                swerveDrive.drive(
                    speedsRobotRelative,
                    swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                    moduleFeedForwards.linearForces()
                                );
              } else
              {
                swerveDrive.setChassisSpeeds(speedsRobotRelative);
              }
            },
            // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController(
                // PPHolonomicController is the built in path following controller for holonomic drive trains
                new PIDConstants(5.0, 0.0, 0.0),
                // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0)
                // Rotation PID constants
            ),
            config,
            // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent())
              {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
            // Reference to this subsystem to set requirements
                            );

      } catch (Exception e)
      {
        // Handle exception as needed
        e.printStackTrace();
      }

      //Preload PathPlanner Path finding
      // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
      PathfindingCommand.warmupCommand().schedule();
    }


}
