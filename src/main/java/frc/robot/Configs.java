package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


public final class Configs {
      public static final SparkMaxConfig coralIntakeConfig = new SparkMaxConfig();
    public static final SparkMaxConfig coralArmConfig = new SparkMaxConfig();
    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig cageConfig = new SparkMaxConfig();
    public static final SparkMaxConfig algaeFollowerConfig = new SparkMaxConfig();
    public static final  SparkBaseConfig algaeIntakeConfig= new SparkMaxConfig();    

    static {
      // Configure basic setting of the arm motor
      coralArmConfig
      .inverted(true)
      .smartCurrentLimit(50)
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(0.2)
          .outputRange(-0.3, 0.5);

        cageConfig
          .smartCurrentLimit(40)
              .closedLoop
              .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
              .p(0.7)
              .outputRange(-1, 1);

      // Configure basic settings of the intake motor
      coralIntakeConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(50);
  
      // Configure basic settings of the elevator motor
      elevatorConfig
      .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .closedLoop
          .outputRange(-0.2, 0.7)
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(0.3);

          algaeIntakeConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);

          algaeFollowerConfig
        .follow(Constants.AlgaeSubsystemConstants.kLeftIntakeMotorCanId,true);
    }
  }