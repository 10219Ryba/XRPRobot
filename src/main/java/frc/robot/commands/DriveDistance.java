// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PIDDrivetrain;

public class DriveDistance extends Command {
  private PIDDrivetrain drivetrain;
  private double setpoint;
  private double speed;
  public DriveDistance(PIDDrivetrain drivetrain, double setpoint, double speed) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.setpoint = setpoint;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.getDistance() >= setpoint;
  }
}
