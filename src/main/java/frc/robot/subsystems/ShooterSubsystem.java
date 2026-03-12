package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax m_shooterMotor;
  private final SparkMax m_feederMotor;
  private final SparkMax m_agitatorMotor;

  private final SparkClosedLoopController m_shooterPID;
  private final InterpolatingDoubleTreeMap m_distanceToSpeedMap = new InterpolatingDoubleTreeMap();

  private double m_targetSpeedRpm = 0.0;
  private double m_lastDistance = 0.0;
  private double m_lastAppliedkP = ShooterConstants.kP;
private double m_lastAppliedkI = ShooterConstants.kI;
private double m_lastAppliedkD = ShooterConstants.kD;
private double m_lastAppliedkFF = ShooterConstants.kFF;

  public ShooterSubsystem() {
    m_shooterMotor = new SparkMax(ShooterConstants.kShooterCanId, MotorType.kBrushless);
    m_feederMotor = new SparkMax(ShooterConstants.kFeederCanId, MotorType.kBrushless);
    m_agitatorMotor = new SparkMax(ShooterConstants.kAgitatorCanId, MotorType.kBrushed);

    m_shooterPID = m_shooterMotor.getClosedLoopController();

    // Publish tuning values to dashboard once at startup
    SmartDashboard.putNumber("Shooter/kP", Constants.ShooterConstants.kP);
    SmartDashboard.putNumber("Shooter/kI", Constants.ShooterConstants.kI);
    SmartDashboard.putNumber("Shooter/kD", Constants.ShooterConstants.kD);
    SmartDashboard.putNumber("Shooter/kFF", Constants.ShooterConstants.kFF);

    SmartDashboard.putNumber("Shooter Test Power", 0.5);

    // Shooter motor config
    SparkMaxConfig shooterConfig = new SparkMaxConfig();
    shooterConfig.smartCurrentLimit(50)
             .inverted(true)
             .closedLoopRampRate(0.25);

    shooterConfig.closedLoop.pidf(
        Constants.ShooterConstants.kP,
        Constants.ShooterConstants.kI,
        Constants.ShooterConstants.kD,
        Constants.ShooterConstants.kFF
    );

    // Feeder motor config
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(40);

    // Agitator motor config
    SparkMaxConfig agitatorConfig = new SparkMaxConfig();
    agitatorConfig.smartCurrentLimit(20)
                  .inverted(true);

    // Apply configs
    m_shooterMotor.configure(
        shooterConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters
    );

    m_feederMotor.configure(
        feederConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters
    );

    m_agitatorMotor.configure(
        agitatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters
    );

    // Populate distance-to-RPM map
    for (double[] point : ShooterConstants.kDistanceMap) {
      m_distanceToSpeedMap.put(point[0], point[1]);
    }
  }

  // --- SHOOTER CONTROL ---

  public void setSpeedFromDistance(double distanceMeters) {
    m_lastDistance = distanceMeters;

    if (distanceMeters < 0) {
      setShooterSpeed(ShooterConstants.kShooterIdleRpm);
      runShooter();
      return;
    }

    double calculatedRpm = m_distanceToSpeedMap.get(distanceMeters);
    setShooterSpeed(calculatedRpm);
    runShooter();
  }

  public void setShooterSpeed(double speedRpm) {
    m_targetSpeedRpm = MathUtil.clamp(speedRpm, 0.0, ShooterConstants.kMaxShooterRpm);
  }

public void runShooter() {
  m_shooterPID.setSetpoint(m_targetSpeedRpm, ControlType.kVelocity);
}


  public void stopShooter() {
    m_targetSpeedRpm = 0.0;
    m_shooterMotor.stopMotor();
  }

  /** Direct percent output for testing. Bypasses PID. */
  public void setShooterRaw(double speed) {
    m_shooterMotor.set(speed);
  }

  // --- FEEDER & AGITATOR CONTROL ---

  public void runFeeder() {
    m_feederMotor.set(ShooterConstants.kFeederSpeed);
  }

  public void stopFeeder() {
    m_feederMotor.stopMotor();
  }

  public void runAgitator() {
    m_agitatorMotor.set(ShooterConstants.kAgitatorSpeed);
  }

  public void stopAgitator() {
    m_agitatorMotor.stopMotor();
  }

  public void stopAll() {
    stopShooter();
    stopFeeder();
    stopAgitator();
  }

  @Override
  public void periodic() {
    double kP = SmartDashboard.getNumber("Shooter/kP", ShooterConstants.kP);
double kI = SmartDashboard.getNumber("Shooter/kI", ShooterConstants.kI);
double kD = SmartDashboard.getNumber("Shooter/kD", ShooterConstants.kD);
double kFF = SmartDashboard.getNumber("Shooter/kFF", ShooterConstants.kFF);

if (kP != m_lastAppliedkP || kI != m_lastAppliedkI || kD != m_lastAppliedkD || kFF != m_lastAppliedkFF) {
  SparkMaxConfig updatedConfig = new SparkMaxConfig();
  updatedConfig.closedLoop.pidf(kP, kI, kD, kFF);

  m_shooterMotor.configure(
      updatedConfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
  );

  m_lastAppliedkP = kP;
  m_lastAppliedkI = kI;
  m_lastAppliedkD = kD;
  m_lastAppliedkFF = kFF;
}
    SmartDashboard.putNumber("Shooter/Actual RPM", m_shooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter/Target RPM", m_targetSpeedRpm);
    SmartDashboard.putNumber("Shooter/Last Distance", m_lastDistance);

    SmartDashboard.setDefaultNumber("Shooter Test Power", 0.5);
    SmartDashboard.setDefaultNumber("Shooter/kP", Constants.ShooterConstants.kP);
    SmartDashboard.setDefaultNumber("Shooter/kI", Constants.ShooterConstants.kI);
    SmartDashboard.setDefaultNumber("Shooter/kD", Constants.ShooterConstants.kD);
    SmartDashboard.setDefaultNumber("Shooter/kFF", Constants.ShooterConstants.kFF);
  }
}