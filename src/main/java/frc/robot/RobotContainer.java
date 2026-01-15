// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autonomous;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem swerveDrive = new SwerveSubsystem();

  // Auto
  private String selectedAuto;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);


  private final SwerveCommand swerveCommand = new SwerveCommand(swerveDrive, () -> m_driverController.getLeftY(), () -> m_driverController.getLeftX(), () -> m_driverController.getRightX());
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Path Planner
    selectedAuto = "Super Duper Test";
    SmartDashboard.putString("Auto Selection", selectedAuto);

    //// Event Commands
   new EventTrigger("shoot").whileTrue(Commands.print("Shooting"));

  }

  /**
   * Use this method to define your trigger->command mappings. 
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    swerveDrive.setDefaultCommand(swerveCommand);
  }

  // Return auto command
  public Command getAutonomousCommand() {
        try{
        PathPlannerPath path = PathPlannerPath.fromPathFile(SmartDashboard.getString("Auto Selection", selectedAuto));
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("ERROR!~!!!!!!!: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }
}
