// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralSubsystemConstants;

public class CoralIntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private SparkMax motor = new SparkMax(CoralSubsystemConstants.kCoralIntakeMotorCanId, MotorType.kBrushless);
 private double power;
 
  public CoralIntakeSubsystem() {
        motor.configure(
        Configs.coralIntakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }


  public void setPower(double power) {
    this.power = power;
    motor.set(power);
  }

  @Override
   public void periodic() {SmartDashboard.putNumber("coralSpeed", this.power);}
}
