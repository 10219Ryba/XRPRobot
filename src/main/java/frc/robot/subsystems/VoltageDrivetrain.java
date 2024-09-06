// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VoltageDrivetrain extends SubsystemBase {
  private final XRPMotor leftMotor = new XRPMotor(0);
  private final XRPMotor rightMotor = new XRPMotor(1);
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);
  public VoltageDrivetrain() {
    leftEncoder.setDistancePerPulse((Math.PI * Constants.kWheelDiameterInch) / Constants.kCountsPerRevolution);
    rightEncoder.setDistancePerPulse((Math.PI * Constants.kWheelDiameterInch) / Constants.kCountsPerRevolution);
    resetEncoders();
    leftMotor.setInverted(false);
    rightMotor.setInverted(true);
    SmartDashboard.putNumber("leftEncoder", getLeftRate());
    SmartDashboard.putNumber("rightEncoder", getRightRate());
  }
  public final DifferentialDrive diffDrive =
     new DifferentialDrive(leftMotor::set, rightMotor::set);

  public void drive(double throttle, double rotation) {
    // double leftVoltage = 12*(throttle + rotation);
    // leftMotor.setVoltage(leftVoltage);
    // double rightVoltage = 11.5*(throttle - rotation);
    // rightMotor.setVoltage(rightVoltage);
    //diffDrive.arcadeDrive(throttle, rotation);
  }

  public double applyDeadband(double value) {
    if (Math.abs(value) < Constants.deadband) {
      return 0.0;
    }
    return value;
  }
  
  public void stopAll() {
    leftMotor.setVoltage(0);
    rightMotor.setVoltage(0);
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }
  public double getLeftRate() {
    return leftEncoder.getRate();
  }
  public double getRightRate() {
    return rightEncoder.getRate();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
