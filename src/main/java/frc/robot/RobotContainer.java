// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeBackward;
import frc.robot.commands.IntakeForward;
import frc.robot.commands.LineupLimelight;
import frc.robot.commands.LineupLimelightTimed;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterCommandTimed;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


//DRIVE 4 angle 3
//drive 7 angle 8
//drive 2 angle 1
//drive 5 angle 6

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveDrive = new SwerveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  // Auto
  private String selectedAuto;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_shooterController = 
      new CommandXboxController(1);


  private final SwerveCommand swerveCommand = new SwerveCommand(swerveDrive, () -> m_driverController.getLeftY(), () -> m_driverController.getLeftX(), () -> m_driverController.getRightX());
  private final IntakeForward intakeForward = new IntakeForward(intakeSubsystem);
  private final IntakeBackward intakeBackward = new IntakeBackward(intakeSubsystem);
  private final ShooterCommand shooterCommand = new ShooterCommand(shooterSubsystem);
  
  //private final SwerveCommand swerveCommand = new SwerveCommand(swerveDrive, () -> m_driverController.getLeftY(), () -> m_driverController.getLeftX(), () -> m_driverController.getRightX());
  private final LineupLimelight lineUpCommand = new LineupLimelight(limelightSubsystem, swerveDrive);
  
  private final LineupLimelightTimed lineUpTimed = new LineupLimelightTimed(limelightSubsystem, swerveDrive);
  private final ShooterCommandTimed shootTimed = new ShooterCommandTimed(10.0, shooterSubsystem);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {



    // Configure the trigger bindings
    configureBindings();

    // Path Planner
    //selectedAuto = "Super Duper Test";
    //SmartDashboard.putString("Auto Selection", selectedAuto);

    //// Event Commands
   new EventTrigger("shoot").whileTrue(Commands.print("Shooting"));
  }

  /**
   * Use this method to define your trigger->command mappings. 
   */
  private void configureBindings() {

    m_shooterController.rightTrigger().whileTrue(intakeForward);
    m_shooterController.leftTrigger().whileTrue(intakeBackward);
    swerveDrive.setDefaultCommand(swerveCommand);
    m_driverController.b().whileTrue(lineUpCommand);
    m_shooterController.a().whileTrue(shooterCommand);
    m_driverController.a().onTrue(Commands.runOnce(() -> swerveDrive.resetPose(new Pose2d()), swerveDrive));
  }


  // Return auto command
  public Command getAutonomousCommand() {
   return shootTimed;
   /* 
    swerveDrive.setupPathPlanner();

    NamedCommands.registerCommand("shoot", shootTimed);
    return new PathPlannerAuto("Backup and Shoot", true);
    */
    /* 
    System.out.println("AUTONOMOUS SELECTED: " + auto_selection);
    if (auto_selection.equals("nothing")) {
      return null;
    } else if (auto_selection.equals("shoot")) {
      return shootTimed;
    } else {
      return null;
    }
    */
     
       
    
  }
}
