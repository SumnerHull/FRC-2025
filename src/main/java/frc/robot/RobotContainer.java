// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SetCagePose;
import frc.robot.commands.SetCoralArmPose;
import frc.robot.commands.SetCoralIntakeSpeed;
import frc.robot.commands.SetElevatorPose;
import frc.robot.commands.SetAlgaeIntakeSpeed;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CageSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;


public class RobotContainer {

  double tx = LimelightHelpers.getTX("");

  /* swerve drive nonsense */
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private double MaxAngularRate = RotationsPerSecond.of(0.854).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  //private final SendableChooser<Command> autoChooser;


  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /* subystems */
  private final AlgaeIntakeSubsystem intake = new AlgaeIntakeSubsystem();
  private final CoralArmSubsystem wrist = new CoralArmSubsystem();
  private final CoralIntakeSubsystem coralintake = new CoralIntakeSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final CageSubsystem cage = new CageSubsystem();

  /* commands */
  private final SetCagePose cageHold = new SetCagePose(cage, Constants.CageSubsystemConstants.CageSetpoints.kCageLevel2);
  private final SetCagePose cageRelease = new SetCagePose(cage, Constants.CageSubsystemConstants.CageSetpoints.kCageLevel1);

  private final SetElevatorPose elevatorLv1 = new SetElevatorPose(elevator, Constants.CoralSubsystemConstants.ElevatorSetpoints.kLevel1);
  private final SetElevatorPose elevatorLv2 = new SetElevatorPose(elevator, Constants.CoralSubsystemConstants.ElevatorSetpoints.kLevel2);
  private final SetElevatorPose elevatorLv3 = new SetElevatorPose(elevator, Constants.CoralSubsystemConstants.ElevatorSetpoints.kLevel3);

  //private final SetElevatorPose elevatorHome = new SetElevatorPose(elevator, 0);
  private final SetCoralArmPose armLv1 = new SetCoralArmPose(wrist, Constants.CoralSubsystemConstants.ArmSetpoints.kArmLevel1);
  private final SetCoralArmPose armLv2 = new SetCoralArmPose(wrist, Constants.CoralSubsystemConstants.ArmSetpoints.kArmLevel2);
  private final SetCoralArmPose armLv3 = new SetCoralArmPose(wrist, Constants.CoralSubsystemConstants.ArmSetpoints.kArmLevel3);
  //private  final ElevatorStateMachine elevatorStateMachine = new ElevatorStateMachine(elevator, wrist);
  private final SetAlgaeIntakeSpeed algaeIntake = new SetAlgaeIntakeSpeed(intake, 5);
  private final SetAlgaeIntakeSpeed algaeOutake = new SetAlgaeIntakeSpeed(intake, -.5);
  private final SetAlgaeIntakeSpeed algaeOff = new SetAlgaeIntakeSpeed(intake, 0);

  private final SetCoralIntakeSpeed coralIntake = new SetCoralIntakeSpeed(coralintake, 5);
  private final SetCoralIntakeSpeed coralOutake = new SetCoralIntakeSpeed(coralintake, -.5);
  private final SetCoralIntakeSpeed coralOff = new SetCoralIntakeSpeed(coralintake, 0);

  private final CommandXboxController drivecontroller = new CommandXboxController(0);
  private final CommandXboxController opController = new CommandXboxController(1);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final AutoFactory autoFactory;
  private final AutoRoutines autoRoutines;
  private final AutoChooser autoChooser = new AutoChooser(); 

  

  public RobotContainer() {
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory, intake);

        autoChooser.addRoutine("SimplePath with Intake", autoRoutines::simplePathWithIntakeAuto);
        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() ->
        drive.withVelocityX(-drivecontroller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(-drivecontroller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(-drivecontroller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
      )
    );

    drivecontroller.a().whileTrue(drivetrain.applyRequest(() -> brake));
    drivecontroller.b().whileTrue(drivetrain.applyRequest(() ->
      point.withModuleDirection(new Rotation2d(-drivecontroller.getLeftY(), -drivecontroller.getLeftX()))
    ));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    drivecontroller.back().and(drivecontroller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    drivecontroller.back().and(drivecontroller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    drivecontroller.start().and(drivecontroller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    drivecontroller.start().and(drivecontroller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    
    // reset the field-centric heading on left bumper press
    drivecontroller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    drivecontroller.rightTrigger(.2).toggleOnTrue(cageHold);
    drivecontroller.leftTrigger(.2).toggleOnTrue(cageRelease);



    opController.a().onTrue(elevatorLv1);
    opController.x().onTrue(elevatorLv2);
    opController.y().onTrue(elevatorLv3);
    opController.leftBumper().toggleOnTrue(algaeIntake);
    opController.rightBumper().toggleOnTrue(algaeOutake);
    opController.rightBumper().toggleOnFalse(algaeOff);
    opController.leftTrigger(.2).toggleOnTrue(coralIntake);
    opController.rightTrigger(.2).toggleOnTrue(coralOutake);
    opController.rightTrigger(.2).toggleOnFalse(coralOff);
    opController.povDown().toggleOnTrue(armLv1);
    opController.povRight().toggleOnTrue(armLv2);
    opController.povUp().toggleOnTrue(armLv3);

  }

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}