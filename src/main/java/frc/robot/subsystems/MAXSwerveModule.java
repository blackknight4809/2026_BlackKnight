// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;

public class MAXSwerveModule {
  private final SparkFlex m_drivingSpark;
  private final SparkFlex m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  // Stored as a raw double in radians, exactly as the REV template does it.
  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  private final String m_name;

  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    this(drivingCANId, turningCANId, chassisAngularOffset, false, "Module[" + drivingCANId + "]");
  }

  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset,
      boolean invertDrive) {
    this(drivingCANId, turningCANId, chassisAngularOffset, invertDrive,
        "Module[" + drivingCANId + "]");
  }

  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset,
      boolean invertDrive, String name) {
    m_name = name;

    m_drivingSpark = new SparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkFlex(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Build a per-module drive config inheriting shared settings,
    // with per-module inversion applied where needed.
    SparkFlexConfig driveConfig = new SparkFlexConfig();
    driveConfig.apply(Configs.MAXSwerveModule.drivingConfig);
    driveConfig.inverted(invertDrive);

    m_drivingSpark.configure(driveConfig,
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // turningConfig is also SparkFlexConfig now -- matches the Flex Dock turning motors
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig,
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /** Returns the current state of the module. */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /** Returns the current position of the module. */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /** Zeroes the drive encoder. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  /** Returns the measured angle of the module. */
  public Rotation2d getMeasuredAngle() {
    return new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset);
  }

  /** Returns the last commanded desired state. */
  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  /**
   * Sets the desired state for the module.
   *
   * Follows the REV MAXSwerve template pattern exactly:
   * - chassisAngularOffset is a raw double throughout
   * - optimize operates in encoder frame
   * - turn setpoint sent as raw radians (SparkFlex wrapping handles [0, 2*PI])
   * - m_desiredState always stores the original unmodified input
   *
   * @param desiredState          Desired speed and angle in robot frame.
   * @param allowAngleWhenStopped When true, steers even at zero speed (X-lock).
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean allowAngleWhenStopped) {

    // Apply chassis angular offset to translate into encoder frame.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle =
        desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize: flip speed and angle if turning >90 degrees is the longer path.
    SwerveModuleState optimized = SwerveModuleState.optimize(
        correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Cosine compensation (2025/2026 WPILib API).
    optimized.cosineScale(new Rotation2d(m_turningEncoder.getPosition()));

    // Skip commanding the drive motor if stopped and not forcing angle (e.g. X-lock).
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.05 && !allowAngleWhenStopped) {
      m_drivingClosedLoopController.setSetpoint(0.0, ControlType.kVelocity);
    } else {
      m_drivingClosedLoopController.setSetpoint(
          optimized.speedMetersPerSecond, ControlType.kVelocity);
    }

    // Send turn setpoint as raw radians -- SparkFlex position wrapping config
    // (positionWrappingInputRange 0 to 2*PI in Configs.java) handles the wrapping.
    m_turningClosedLoopController.setSetpoint(
        optimized.angle.getRadians(), ControlType.kPosition);

    // --- DIAGNOSTICS ---
    SmartDashboard.putNumber("Swerve/" + m_name + "/EncoderAngleDeg",
        Math.toDegrees(m_turningEncoder.getPosition()));
    SmartDashboard.putNumber("Swerve/" + m_name + "/DesiredAngleDeg",
        desiredState.angle.getDegrees());
    SmartDashboard.putNumber("Swerve/" + m_name + "/OptimizedAngleDeg",
        optimized.angle.getDegrees());
    SmartDashboard.putNumber("Swerve/" + m_name + "/OptimizedSpeed",
        optimized.speedMetersPerSecond);
    SmartDashboard.putString("Swerve/" + m_name + "/Mode",
        optimized.speedMetersPerSecond < 0 ? "FLIPPED" : "NORMAL");

    // Store the raw original input -- this is exactly what the REV template does.
    // Do NOT store the optimized or offset-modified state here.
    m_desiredState = desiredState;
  }
}