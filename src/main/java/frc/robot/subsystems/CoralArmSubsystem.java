// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralSubsystemConstants;

public class CoralArmSubsystem extends SubsystemBase {
  /** Creates a new Wrist. */
  private SparkMax motor = new SparkMax(CoralSubsystemConstants.kCoralArmMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController armController = motor.getClosedLoopController();
  private RelativeEncoder coralArmEncoder = motor.getEncoder();
  private double setPoint = 0;

  public CoralArmSubsystem() {
    motor.configure(
        Configs.coralArmConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    // Zero arm encoder on initialization
    coralArmEncoder.setPosition(0);
  }

  public void setSetPoint(double setPoint)
  {
    this.setPoint = setPoint;
    armController.setReference(setPoint, ControlType.kPosition);
  }

  public boolean isAtPose()
  {
    return Math.abs(coralArmEncoder.getPosition() - setPoint) < 0.1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("armPosition", coralArmEncoder.getPosition());
    SmartDashboard.putNumber("ArmSetPoint", this.setPoint);
    SmartDashboard.putBoolean("ArmAtPose", this.isAtPose());
  }
}
