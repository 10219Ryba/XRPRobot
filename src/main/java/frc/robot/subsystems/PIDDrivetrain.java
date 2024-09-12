// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PIDDrivetrain extends SubsystemBase {
  private final XRPMotor leftMotor = new XRPMotor(0);
  private final XRPMotor rightMotor = new XRPMotor(1);
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);
  private final XRPGyro gyro = new XRPGyro();
  private final PIDController leftPID = new PIDController(0, 0, 0);
  private final PIDController rightPID = new PIDController(0, 0, 0);
  private DifferentialDriveWheelSpeeds diffDriveSpeed = new DifferentialDriveWheelSpeeds();
  private final DifferentialDriveKinematics diffDriveKinematics = new DifferentialDriveKinematics(
      Units.inchesToMeters(6.125));
  private final double maxSpeedInMPS = Units.feetToMeters(10 / 4.5);
  private final double maxRotationInRadians = 3.5 * Math.PI;

  public PIDDrivetrain() {
    leftEncoder.setDistancePerPulse(
        (Math.PI * Units.inchesToMeters(Constants.kWheelDiameterInch)) / Constants.kCountsPerRevolution);
    rightEncoder.setDistancePerPulse(
        (Math.PI * Units.inchesToMeters(Constants.kWheelDiameterInch)) / Constants.kCountsPerRevolution);
    resetEncoders();
    leftMotor.setInverted(false);
    rightMotor.setInverted(true);
    SmartDashboard.putNumber("leftEncoder", getLeftRate());
    SmartDashboard.putNumber("rightEncoder", getRightRate());
    SmartDashboard.putNumber("leftP", 6);
    SmartDashboard.putNumber("rightP", 6);
    leftPID.setTolerance(maxRotationInRadians);
  }

  public void drive(double throttle, double rotation) {
    leftPID.setP(SmartDashboard.getNumber("leftP", 6));
    rightPID.setP(SmartDashboard.getNumber("rightP", 6));
    SmartDashboard.putNumber("Gyro Angle", getHeading());
    diffDriveSpeed = diffDriveKinematics
        .toWheelSpeeds(new ChassisSpeeds(throttle * maxSpeedInMPS, 0, rotation * maxRotationInRadians));
    double leftPIDOutput = leftPID.calculate(leftEncoder.getRate(), diffDriveSpeed.leftMetersPerSecond);
    double rightPIDOutput = rightPID.calculate(rightEncoder.getRate(), diffDriveSpeed.rightMetersPerSecond);
    SmartDashboard.putNumber("leftPIDOutput", leftPIDOutput);
    SmartDashboard.putNumber("rightPIDOutput", rightPIDOutput);
    leftMotor.setVoltage(leftPIDOutput);
    rightMotor.setVoltage(rightPIDOutput);
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

  public double getDistance() {
    return leftEncoder.getDistance();
  }

  public void resetGyro() {
    gyro.reset();
  }

  public double getHeading() {
    return gyro.getAngleZ();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
