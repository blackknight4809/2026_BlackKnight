package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Configs;

public class MAXSwerveModule {
  private final SparkFlex m_drivingSpark;
  private final SparkFlex m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private Rotation2d m_chassisAngularOffset = new Rotation2d();
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, boolean driveInverted) {
    m_drivingSpark = new SparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkFlex(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    SparkMaxConfig thisDrivingConfig = new SparkMaxConfig();
    thisDrivingConfig.apply(Configs.MAXSwerveModule.drivingConfig);
    thisDrivingConfig.inverted(driveInverted);

    m_drivingSpark.configure(
        thisDrivingConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_turningSpark.configure(
        Configs.MAXSwerveModule.turningConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = Rotation2d.fromRadians(chassisAngularOffset);

    m_desiredState = new SwerveModuleState(
        0.0,
        new Rotation2d(m_turningEncoder.getPosition()));

    m_drivingEncoder.setPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition()).minus(m_chassisAngularOffset));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition()).minus(m_chassisAngularOffset));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    setDesiredState(desiredState, false);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean allowAngleWhenStopped) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(m_chassisAngularOffset);

   //correctedDesiredState.optimize(getMeasuredAngle().plus(m_chassisAngularOffset));
   //correctedDesiredState.cosineScale(getMeasuredAngle().plus(m_chassisAngularOffset));

    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001 && !allowAngleWhenStopped) {
      m_drivingClosedLoopController.setSetpoint(0.0, ControlType.kVelocity);
      m_turningClosedLoopController.setSetpoint(
          m_desiredState.angle.getRadians(),
          ControlType.kPosition);
    } else {
      m_drivingClosedLoopController.setSetpoint(
          correctedDesiredState.speedMetersPerSecond,
          ControlType.kVelocity);
      m_turningClosedLoopController.setSetpoint(
          correctedDesiredState.angle.getRadians(),
          ControlType.kPosition);
    }

m_desiredState = correctedDesiredState;
  }

  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public Rotation2d getMeasuredAngle() {
    return new Rotation2d(m_turningEncoder.getPosition()).minus(m_chassisAngularOffset);
  }

  public Rotation2d getDesiredAngle() {
    return m_desiredState.angle;
  }
}