package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final NetworkTable m_limelightTable;

    /**
     * A simple record to hold both the pose and the exact time it was captured.
     * This mimics how PhotonVision returns data.
     */
    public record VisionMeasurement(Pose2d pose, double timestampSeconds) {}

    public VisionSubsystem() {
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget());
        SmartDashboard.putNumber("Vision/Yaw (tx)", getYawOffset());
        SmartDashboard.putNumber("Vision/Pitch (ty)", getPitchOffset());
        SmartDashboard.putNumber("Vision/Distance", getDistanceToTarget());
        SmartDashboard.putNumber("Vision/TargetID", getTargetID());
    }

    // =========================
    // TARGETING METHODS
    // =========================

    /** Returns true if the camera sees a valid target */
    public boolean hasTarget() {
        return m_limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    /** Horizontal offset from crosshair to target in degrees */
    public double getYawOffset() {
        return m_limelightTable.getEntry("tx").getDouble(0.0);
    }

    /** Vertical offset from crosshair to target in degrees */
    public double getPitchOffset() {
        return m_limelightTable.getEntry("ty").getDouble(0.0);
    }

    /** Target area as a percentage of image */
    public double getTargetArea() {
        return m_limelightTable.getEntry("ta").getDouble(0.0);
    }

    /**
     * Returns straight-line 3D distance from camera to target in meters.
     * Uses Limelight's targetpose_cameraspace array:
     * [x, y, z, roll, pitch, yaw]
     */
    public double getDistanceToTarget() {
        if (!hasTarget()) {
            return -1.0;
        }

        double[] targetPose = m_limelightTable
                .getEntry("targetpose_cameraspace")
                .getDoubleArray(new double[6]);

        if (targetPose.length < 3) {
            return -1.0;
        }

        double x = targetPose[0];
        double y = targetPose[1];
        double z = targetPose[2];

        return Math.sqrt((x * x) + (y * y) + (z * z));
    }

    /** Returns the ID of the primary AprilTag currently in view, or -1 if none */
    public int getTargetID() {
        return (int) m_limelightTable.getEntry("tid").getInteger(-1);
    }

    // =========================
    // ODOMETRY / BOTPOSE METHODS
    // =========================

    /**
     * Grabs the current botpose from Limelight and calculates the
     * latency-compensated timestamp.
     * Returns null if the data is invalid or missing.
     */
    public VisionMeasurement getEstimatedGlobalPose(Rotation2d gyroRotation) {
        if (!hasTarget()) {
            return null;
        }

        String botposeKey = "botpose_wpiblue";
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            botposeKey = "botpose_wpired";
        }

        double[] botpose = m_limelightTable.getEntry(botposeKey).getDoubleArray(new double[6]);
        if (botpose.length < 6) {
            return null;
        }

        double latencyMs =
                m_limelightTable.getEntry("tl").getDouble(0.0)
                        + m_limelightTable.getEntry("cl").getDouble(0.0);
        double timestamp = Timer.getFPGATimestamp() - (latencyMs / 1000.0);

        Pose2d visionPose = new Pose2d(botpose[0], botpose[1], gyroRotation);

        return new VisionMeasurement(visionPose, timestamp);
    }
}