package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SetAlgaeIntakeSpeed;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class AutoRoutines {
    private final AutoFactory m_factory;
    private final AlgaeIntakeSubsystem m_intake;

    public AutoRoutines(AutoFactory factory, AlgaeIntakeSubsystem intake) {
        m_factory = factory;
        m_intake = intake;
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory simplePath = routine.trajectory("SimplePath");

        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }

    public AutoRoutine simplePathWithIntakeAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SimplePath with Intake Auto");
        final AutoTrajectory simplePath = routine.trajectory("SimplePath");

        routine.active().onTrue(
            new SequentialCommandGroup(
                simplePath.resetOdometry(),
                simplePath.cmd(),
                new SetAlgaeIntakeSpeed(m_intake, 1.0).withTimeout(2.0), // Turn on intake for 2 seconds
                new SetAlgaeIntakeSpeed(m_intake, 0.0) // Turn off intake
            )
        );
        return routine;
    }
}
