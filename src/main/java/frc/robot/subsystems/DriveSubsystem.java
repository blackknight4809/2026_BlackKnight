package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules using template-style constructor
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  private final VisionSubsystem m_vision;

  // Gyro sensor
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  // Field display
  private final Field2d m_field = new Field2d();

  // Pose estimator (odometry + vision fusion)
  private final SwerveDrivePoseEstimator m_poseEstimator;

  private static final String kVisionMinTaKey = "Vision/MinTA";
  private static final String kVisionMaxTurnRateKey = "Vision/MaxTurnRateDps";
  private static final String kVisionXYStdDevKey = "Vision/XYStdDevM";

  public DriveSubsystem(VisionSubsystem vision) {
    m_vision = vision;

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    SmartDashboard.putData("Field", m_field);

    SmartDashboard.putNumber(kVisionMinTaKey, 0.1);
    SmartDashboard.putNumber(kVisionMaxTurnRateKey, 90.0);
    SmartDashboard.putNumber(kVisionXYStdDevKey, 0.5);

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      throw new RuntimeException("Failed to load PathPlanner RobotConfig from GUI settings", e);
    }

    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0)),
        config,
        () -> DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
        this);

    m_gyro.zeroYaw();

    m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        getGyroRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, 0.02),
        VecBuilder.fill(0.5, 0.5, 0.5));
  }

  /**
   * Single source of truth for heading used everywhere.
   * If field-relative feels backwards or spins wrong, this is one of the first
   * places to check.
   */
  private Rotation2d getGyroRotation() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  @Override
  public void periodic() {
    m_poseEstimator.update(
        getGyroRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    fuseLimelightPose();

    Pose2d est = m_poseEstimator.getEstimatedPosition();
    SmartDashboard.putNumber("GyroYawDeg(raw)", m_gyro.getYaw());
    SmartDashboard.putNumber("GyroHeadingDeg(used)", getGyroRotation().getDegrees());
    SmartDashboard.putNumber("Pose/X", est.getX());
    SmartDashboard.putNumber("Pose/Y", est.getY());
    SmartDashboard.putNumber("Pose/HeadingDeg", est.getRotation().getDegrees());

    m_field.setRobotPose(est);
  }

  private void fuseLimelightPose() {
    double minTa = SmartDashboard.getNumber(kVisionMinTaKey, 0.1);
    if (!m_vision.hasTarget() || m_vision.getTargetArea() < minTa) {
      return;
    }

    double maxTurn = SmartDashboard.getNumber(kVisionMaxTurnRateKey, 90.0);
    if (Math.abs(getTurnRate()) > maxTurn) {
      return;
    }

    var visionMeasurement = m_vision.getEstimatedGlobalPose(getGyroRotation());
    if (visionMeasurement == null) {
      return;
    }

    double now = Timer.getFPGATimestamp();
    if (visionMeasurement.timestampSeconds() > now
        || (now - visionMeasurement.timestampSeconds()) > 0.5) {
      return;
    }

    addVisionMeasurement(visionMeasurement.pose(), visionMeasurement.timestampSeconds());
  }

  public void addVisionMeasurement(Pose2d visionPose, double visionTimestampSeconds) {
    if (visionPose == null) {
      return;
    }

    if (Double.isNaN(visionPose.getX()) || Double.isNaN(visionPose.getY())) {
      return;
    }

    double xyStd = SmartDashboard.getNumber(kVisionXYStdDevKey, 0.5);
    xyStd = Math.max(0.05, xyStd);

    m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStd, xyStd, 999.0));
    m_poseEstimator.addVisionMeasurement(visionPose, visionTimestampSeconds);
  }

  /**
   * Returns the currently estimated pose of the robot.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        getGyroRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward)
   * @param ySpeed Speed of the robot in the y direction (sideways)
   * @param rot Angular rate of the robot
   * @param fieldRelative Whether x and y are field-relative
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    SwerveModuleState[] swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    getGyroRotation())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve module states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.zeroYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return robot heading in degrees
   */
  public double getHeading() {
    return getGyroRotation().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return turn rate in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns chassis speeds in robot-relative coordinates.
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  /**
   * Drives using robot-relative chassis speeds.
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] states =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_rearLeft.setDesiredState(states[2]);
    m_rearRight.setDesiredState(states[3]);
  }

  public void setLimelightLED(int mode) {
    NetworkTableInstance.getDefault()
        .getTable("limelight")
        .getEntry("ledMode")
        .setNumber(mode);
  }
}