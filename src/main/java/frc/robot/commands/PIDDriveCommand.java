// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.PIDDrivetrain;

public class PIDDriveCommand extends Command {
  private PIDDrivetrain drivetrain;
  private CommandXboxController xbox1;
  public PIDDriveCommand(PIDDrivetrain drivetrain, CommandXboxController xbox1) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.xbox1 = xbox1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(drivetrain.applyDeadband(-xbox1.getLeftY()), drivetrain.applyDeadband(-xbox1.getRightX()));
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
