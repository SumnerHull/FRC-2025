// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CageSubsystemConstants;

public class CageSubsystem extends SubsystemBase {
  /** Creates a new Wrist. */
  private SparkMax motor = new SparkMax(CageSubsystemConstants.kCageArmMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController cageController = motor.getClosedLoopController();
  private RelativeEncoder cageEncoder = motor.getEncoder();
  private double setPoint = 0;

  public CageSubsystem() {
    motor.configure(
        Configs.cageConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    // Zero arm encoder on initialization
    cageEncoder.setPosition(0);
  }

  public void setSetPoint(double setPoint)
  {
    this.setPoint = setPoint;
    cageController.setReference(setPoint, ControlType.kPosition);
  }

  public boolean isAtPose()
  {
    return Math.abs(cageEncoder.getPosition() - setPoint) < 0.1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("cagePosition", cageEncoder.getPosition());
    SmartDashboard.putNumber("cageSetPoint", this.setPoint);
    SmartDashboard.putBoolean("cageAtPose", this.isAtPose());
  }
}
