package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command {

  private final ClimberSubsystem m_climber;
  private final DoubleSupplier m_speedSupplier;

  public ClimberCommand(ClimberSubsystem climber, DoubleSupplier speedSupplier) {
    m_climber = climber;
    m_speedSupplier = speedSupplier;

    addRequirements(climber);
  }

  @Override
  public void execute() {
    m_climber.setClimberSpeed(m_speedSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.stopClimber();
  }
}