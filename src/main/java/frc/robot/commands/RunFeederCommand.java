package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PickupSubsystem;

public class RunFeederCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final PickupSubsystem m_pickup;

    public RunFeederCommand(ShooterSubsystem shooter, PickupSubsystem pickup) {
        m_shooter = shooter;
        m_pickup = pickup;

        addRequirements(m_shooter, m_pickup);
    }

    @Override
    public void initialize() {
        m_shooter.runFeeder();
        m_shooter.runAgitator();
        m_pickup.requestFeederRoller();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopFeeder();
        m_shooter.stopAgitator();
        m_pickup.releaseFeederRoller();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}