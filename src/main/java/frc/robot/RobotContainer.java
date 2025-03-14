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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SetCagePose;
import frc.robot.commands.SetCoralArmPose;
import frc.robot.commands.SetCoralIntakeSpeed;
import frc.robot.commands.SetElevatorPose;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SetAlgaeIntakeSpeed;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CageSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.AutoBuilder;




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
  private final LedSubsystem led = new LedSubsystem();

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

  public final CommandSwerveDrivetrain drivetrain =
  new CommandSwerveDrivetrain(
      new GyroIO() {},
      new ModuleIOTalonFX(TunerConstants.FrontLeft),
      new ModuleIOTalonFX(TunerConstants.FrontRight),
      new ModuleIOTalonFX(TunerConstants.BackLeft),
      new ModuleIOTalonFX(TunerConstants.BackRight));


  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final AutoFactory autoFactory;
  private final AutoRoutines autoRoutines;
  private final AutoChooser autoChooser = new AutoChooser(); 


  

  public RobotContainer() {
        final LoggedDashboardChooser<Command> autoChooser;
        

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        //autoChooser.addOption(
        //    "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
       // autoChooser.addOption(
         //   "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
       // autoChooser.addOption(
         //   "Drive SysId (Quasistatic Forward)",
         //   drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
       // autoChooser.addOption(
         //   "Drive SysId (Quasistatic Reverse)",
          //  drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
       // autoChooser.addOption(
          //  "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        //autoChooser.addOption(
         //   "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));


        configureBindings();

  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    CommandSwerveDrivetrain.setDefaultCommand(
        new DriveCommands.joystickDrive(
            drive,
            () -> -drivecontroller.getLeftY(
            tLeftY(),
            () -> -drivecontroller.getLeftX(),
            () -> -drivecontroller.getRightX())));

    // Lock to 0° when A button is held
    drivecontroller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -drivecontroller.getLeftY(),
                () -> -drivecontroller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    drivecontroller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    drivecontroller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }

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

    // Trigger solid red pattern when pressing the "A" button
   // opController.a().onTrue(led.runPattern(LEDPattern.solid(Color.kRed)));
    
    // Trigger solid green pattern when pressing the "B" button
   // opController.b().onTrue(led.runPattern(LEDPattern.solid(Color.kGreen)));
    
    // Trigger solid blue pattern when pressing the "X" button
    //opController.x().onTrue(led.runPattern(LEDPattern.solid(Color.kBlue)));
    
    // Turn off LED strip when "Y" button is pressed
   // opController.y().onTrue(led.runPattern(LEDPattern.solid(Color.kBlack)));

    // You can also add more complex patterns based on your needs, for example:
    //opController.leftBumper().onTrue(led.runPattern(LEDPattern.rainbow()));

  

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
}