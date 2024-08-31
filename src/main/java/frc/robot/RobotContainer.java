// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArmNinety;
import frc.robot.commands.ArmOneEight;
import frc.robot.commands.VoltageDriveCommand;
import frc.robot.subsystems.VoltageDrivetrain;
import frc.robot.subsystems.XRPArm;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final VoltageDrivetrain voltDrive = new VoltageDrivetrain();
  private final XRPArm armSubsystem = new XRPArm();
  CommandXboxController xbox1 = new CommandXboxController(0);
  private final VoltageDriveCommand driveCommand = new VoltageDriveCommand(voltDrive, xbox1);
  private final ArmNinety armNinety = new ArmNinety(armSubsystem);
  private final ArmOneEight armOneEight = new ArmOneEight(armSubsystem);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    voltDrive.setDefaultCommand(driveCommand);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    xbox1.a().onTrue(armNinety);
    xbox1.b().onTrue(armOneEight);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return m_autoCommand;
  // }
}
