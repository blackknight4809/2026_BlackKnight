package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.math.MathUtil;
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
  // Modules
private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
    DriveConstants.kFrontLeftDrivingCanId,
    DriveConstants.kFrontLeftTurningCanId,
    DriveConstants.kFrontLeftChassisAngularOffset,
    false);

private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
    DriveConstants.kFrontRightDrivingCanId,
    DriveConstants.kFrontRightTurningCanId,
    DriveConstants.kFrontRightChassisAngularOffset,
    false);

private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
    DriveConstants.kRearLeftDrivingCanId,
    DriveConstants.kRearLeftTurningCanId,
    DriveConstants.kBackLeftChassisAngularOffset,
    false);

private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
    DriveConstants.kRearRightDrivingCanId,
    DriveConstants.kRearRightTurningCanId,
    DriveConstants.kBackRightChassisAngularOffset,
    false);


  private final VisionSubsystem m_vision;
  // Gyro (Studica navX)
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  // Field display
  private final Field2d m_field = new Field2d();

  // Pose estimator (odometry + vision fusion)
  private final SwerveDrivePoseEstimator m_poseEstimator;

  // Dashboard keys (new + old aliases)
  private static final String kXLockEnabledKey     = "Drive/XLockEnabled";
  private static final String kSpeedLimitKey       = "Drive/SpeedLimit";
  private static final String kXLockEnabledKeyOld  = "XLockEnabled";
  private static final String kSpeedLimitKeyOld    = "SpeedLimit";

  private static final String kVisionMinTaKey = "Vision/MinTA";
  private static final String kVisionMaxTurnRateKey = "Vision/MaxTurnRateDps";
  private static final String kVisionXYStdDevKey = "Vision/XYStdDevM";

  public DriveSubsystem(VisionSubsystem vision) {
    this.m_vision = vision; // Save it for later
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    SmartDashboard.putData("Field", m_field);

    // Publish defaults WITHOUT overwriting existing dashboard values
    Preferences.initBoolean(kXLockEnabledKey, false);
    Preferences.initDouble(kSpeedLimitKey, 1.0);

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
    this::getPose,                 // Robot pose supplier
    this::resetOdometry,           // Reset odometry to a given pose
    this::getRobotRelativeSpeeds,  // MUST be robot-relative
    (speeds, feedforwards) -> driveRobotRelative(speeds), // robot-relative output
    new PPHolonomicDriveController(
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID
        new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID
    ),
    config,
    () -> DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
    this
);

    // Make current facing be "0"
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
        VecBuilder.fill(0.5, 0.5, 0.5)
    );
  }

  /** Single source of truth for heading used everywhere. */
  private Rotation2d getGyroRotation() {
    // Continuous + WPILib-friendly sign
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
    //DriveTrain SmartDashboard Values
    SmartDashboard.putNumber("FL Angle Deg", m_frontLeft.getMeasuredAngle().getDegrees());
    SmartDashboard.putNumber("FR Angle Deg", m_frontRight.getMeasuredAngle().getDegrees());
    SmartDashboard.putNumber("BL Angle Deg", m_rearLeft.getMeasuredAngle().getDegrees());
    SmartDashboard.putNumber("BR Angle Deg", m_rearRight.getMeasuredAngle().getDegrees());

    SmartDashboard.putNumber("GyroYawDeg(raw)", m_gyro.getYaw());
    SmartDashboard.putNumber("GyroHeadingDeg(used)", getGyroRotation().getDegrees());

    Pose2d est = m_poseEstimator.getEstimatedPosition();
    SmartDashboard.putNumber("Pose/X", est.getX());
    SmartDashboard.putNumber("Pose/Y", est.getY());
    SmartDashboard.putNumber("Pose/HeadingDeg", est.getRotation().getDegrees());

    m_field.setRobotPose(est);
  }

 /** Read botpose from VisionSubsystem and add it as a vision measurement when valid. */
  private void fuseLimelightPose() {
    // 1. Check if we even have a target and if the target area is large enough to trust
    double minTa = SmartDashboard.getNumber(kVisionMinTaKey, 0.1);
    if (!m_vision.hasTarget() || m_vision.getTargetArea() < minTa) {
      return;
    }

    // 2. Don't trust vision if we are spinning super fast (motion blur!)
    double maxTurn = SmartDashboard.getNumber(kVisionMaxTurnRateKey, 90.0);
    if (Math.abs(getTurnRate()) > maxTurn) {
      return;
    }

    // 3. Ask VisionSubsystem for the calculated pose and timestamp
    var visionMeasurement = m_vision.getEstimatedGlobalPose(getGyroRotation());
    if (visionMeasurement == null) {
      return;
    }

    // 4. Reject old data
    double now = Timer.getFPGATimestamp();
    if (visionMeasurement.timestampSeconds() > now || (now - visionMeasurement.timestampSeconds()) > 0.5) {
      return;
    }

    // 5. Send it to the pose estimator
    addVisionMeasurement(visionMeasurement.pose(), visionMeasurement.timestampSeconds());
  }

  public void addVisionMeasurement(Pose2d visionPose, double visionTimestampSeconds) {
    if (visionPose == null) return;
    if (Double.isNaN(visionPose.getX()) || Double.isNaN(visionPose.getY())) return;

    double xyStd = SmartDashboard.getNumber(kVisionXYStdDevKey, 0.5);
    xyStd = Math.max(0.05, xyStd);

    m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStd, xyStd, 999.0));
    m_poseEstimator.addVisionMeasurement(visionPose, visionTimestampSeconds);
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

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

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Read old widgets if they exist, otherwise use new keys
boolean xLockEnabled =
    Preferences.getBoolean(kXLockEnabledKey, false);

double speedLimit =
    Preferences.getDouble(kSpeedLimitKey, 1.0);

speedLimit = MathUtil.clamp(speedLimit, 0.0, 1.0);

    if (xLockEnabled
        && Math.abs(xSpeed) < 0.05
        && Math.abs(ySpeed) < 0.05
        && Math.abs(rot) < 0.05) {
      setX();
      return;
    }

    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond * speedLimit;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond * speedLimit;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed * speedLimit;

    SwerveModuleState[] states =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered, ySpeedDelivered, rotDelivered, getGyroRotation())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_rearLeft.setDesiredState(states[2]);
    m_rearRight.setDesiredState(states[3]);
  }

  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
  }

  public void zeroHeading() {
    m_gyro.zeroYaw();
  }

  public double getHeading() {
    return getGyroRotation().getDegrees();
  }

  public double getTurnRate() {
    double rateDps = m_gyro.getRate();
    rateDps *= (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    return rateDps;
  }
// PathPlanner wants robot-relative chassis speeds (m/s, m/s, rad/s)
public ChassisSpeeds getRobotRelativeSpeeds() {
  return DriveConstants.kDriveKinematics.toChassisSpeeds(
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
  );
}

// PathPlanner will call this to actually move the robot during auto
public void driveRobotRelative(ChassisSpeeds speeds) {
  SwerveModuleState[] states =
      DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

  SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);

  m_frontLeft.setDesiredState(states[0]);
  m_frontRight.setDesiredState(states[1]);
  m_rearLeft.setDesiredState(states[2]);
  m_rearRight.setDesiredState(states[3]);
}

/**
 * Changes the Limelight LED mode.
 * @param mode 1 for OFF, 3 for ON.
 */
public void setLimelightLED(int mode) {
  NetworkTableInstance.getDefault()
    .getTable("limelight")
    .getEntry("ledMode")
    .setNumber(mode);
}

}