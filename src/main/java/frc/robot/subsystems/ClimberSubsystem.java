package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax m_climberMotor =
      new SparkMax(ClimberConstants.kClimberMotorCanId, MotorType.kBrushless);

  private final RelativeEncoder m_climberEncoder = m_climberMotor.getEncoder();

  private final DoubleSolenoid m_ratchetSolenoid =
      new DoubleSolenoid(
          PneumaticsModuleType.REVPH,
          ClimberConstants.kRatchetForwardChannel,
          ClimberConstants.kRatchetReverseChannel);

  public ClimberSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(ClimberConstants.kClimberMotorInverted);

    m_climberMotor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public double getClimberRotations() {
    return m_climberEncoder.getPosition();
  }

    public boolean isAtOrAboveRotations(double targetRotations) {
    return getClimberRotations() >= targetRotations;
  }

  public void disengageRatchet() {
  m_ratchetSolenoid.set(DoubleSolenoid.Value.kForward);
}

public void engageRatchet() {
  m_ratchetSolenoid.set(DoubleSolenoid.Value.kReverse);
}
public void setClimberSpeed(double speed) {
  m_climberMotor.set(speed);
}

public void runClimberDownManual() {
  disengageRatchet();
  setClimberSpeed(-0.25);
}

public void stopClimberAndEngageRatchet() {
  stopClimber();
  engageRatchet();
}

public void runClimberUpManual() {
  engageRatchet();
  setClimberSpeed(0.4);
}

public void stopClimber() {
  m_climberMotor.stopMotor();
}

public void zeroClimberEncoder() {
  m_climberEncoder.setPosition(0.0);
}


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Rotations", getClimberRotations());
  }
}