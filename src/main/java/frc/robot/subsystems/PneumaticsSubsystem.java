package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsSubsystem extends SubsystemBase {
  // The REV Pneumatics Hub (PH)
  private final PneumaticHub m_ph = new PneumaticHub(1);

  // NOTE: Solenoids have been removed from here! 
  // They belong in the PickupSubsystem.

  public PneumaticsSubsystem() {
    // Enable the compressor to run automatically based on the pressure switch
    m_ph.enableCompressorAnalog(100,120);
  }

  @Override
  public void periodic() {
    // Push the analog pressure to the dashboard (Analog Port 0)
    SmartDashboard.putNumber("Pneumatics/Pressure PSI", m_ph.getPressure(0));
    SmartDashboard.putBoolean("Pneumatics/Compressor Active", m_ph.getCompressor());
  }
}