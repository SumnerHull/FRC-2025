// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class CageSubsystemConstants {
  public static final int kCageArmMotorCanId = 30;
  

  public static final class CageSetpoints {
    public static final double kCageLevel1 = 0;
    public static final double kCageLevel2 = 325;
    }
  }
    
  public static final class CoralSubsystemConstants {
    public static final int kElevatorMotorCanId1 = 25;
    //public static final int kElevatorMotorCanId2 = 31;
    public static final int kCoralArmMotorCanId = 26;
    public static final int kCoralIntakeMotorCanId = 27;


    public static final class ElevatorSetpoints {
      public static final double kFeederStation = 0;
      public static final double kLevel1 = 0;
      public static final double kLevel2 = 12;
      //public static final double kLevel3 = 45;
      public static final double kLevel3 = 64;
    }

    public static final class ArmSetpoints {
//      public static final double kFeederStation = 33;
      public static final double kArmLevel1 = 0;
      public static final double kArmLevel2 = 1.9;
      public static final double kArmLevel3 = 4;
    }

    public static final class IntakeSetpoints {
      public static final double kForward = 0.5;
      public static final double kReverse = -1;
      //public static final double kReverse = -0.5;
    }
  }

  public static final class AlgaeSubsystemConstants {
    public static final int kRightIntakeMotorCanId = 28;
    public static final int kLeftIntakeMotorCanId = 29;
    //public static final int kPivotMotorCanId = 35;

    public static final class ArmSetpoints {
      public static final double kStow = 18.5;
      public static final double kHold = 11.5;
      public static final double kDown = 0;
    }

    public static final class IntakeSetpoints {
      public static final double kForward = 0.5;
      public static final double kReverse = -0.5;
      public static final double kHold = 0.25;
    }
  }

}
