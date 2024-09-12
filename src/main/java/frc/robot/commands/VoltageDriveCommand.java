// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.VoltageDrivetrain;

public class VoltageDriveCommand extends Command {
  VoltageDrivetrain drivetrain;
  CommandXboxController joystick;
  public VoltageDriveCommand(VoltageDrivetrain drivetrain, CommandXboxController joystick) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.joystick = joystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = drivetrain.applyDeadband(-joystick.getLeftY());
    double rotation = drivetrain.applyDeadband(joystick.getRightX());

    drivetrain.drive(throttle, rotation); 
  }                                                                 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
