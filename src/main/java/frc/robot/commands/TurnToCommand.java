// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.VoltageDrivetrain;

public class TurnToCommand extends Command {
    Gyro gyro;
    VoltageDrivetrain drivetrain;
    double targetAngle;
    PIDController gyroPID = new PIDController(0, 0, 0);

    public TurnToCommand(Gyro gyro, VoltageDrivetrain drivetrain, double targetAngle) {
        addRequirements(gyro);
        addRequirements(drivetrain);
        this.gyro = gyro;
        this.drivetrain = drivetrain;
        this.targetAngle = targetAngle;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        gyroPID.setP(0.1);
        gyroPID.calculate(gyro.getAngle() - targetAngle);
        System.out.println(gyroPID);
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
