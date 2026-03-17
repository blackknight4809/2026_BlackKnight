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
        SmartDashboard.putNumber("Vision/TargetSpaceX", 0.0);
        SmartDashboard.putNumber("Vision/TargetSpaceY", 0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget());
        SmartDashboard.putNumber("Vision/Yaw (tx)", getYawOffset());
        SmartDashboard.putNumber("Vision/Pitch (ty)", getPitchOffset());
        SmartDashboard.putNumber("Vision/Distance", getDistanceToTarget());
        SmartDashboard.putNumber("Vision/TargetID", getTargetID());
        SmartDashboard.putNumber("Vision/TargetSpaceX", getTargetSpaceX());
        SmartDashboard.putNumber("Vision/TargetSpaceY", getTargetSpaceY());
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
/**
     * Returns straight-line 3D distance from camera to target in meters.
     * Uses Limelight's targetpose_cameraspace array:
     * [x, y, z, roll, pitch, yaw]
     */
    public double getDistanceToTarget() {
        if (!hasTarget()) {
            return -1.0;
        }

        // targetpose_cameraspace is a 6-element array
        double[] targetPose = m_limelightTable
                .getEntry("targetpose_cameraspace")
                .getDoubleArray(new double[6]);

        // If the array is empty or too small, abort
        if (targetPose.length < 6) {
            return -1.0;
        }

        double x = targetPose[0];
        double y = targetPose[1];
        double z = targetPose[2];

        // Calculate the 3D hypotenuse. 
        double distance = Math.sqrt((x * x) + (y * y) + (z * z));
        
        // If distance is perfectly 0.0, it usually means the Limelight sent dummy data
        if (distance == 0.0) {
            return -1.0; 
        }

        return distance;
    }

    /** Returns the ID of the primary AprilTag currently in view, or -1 if none */
    public int getTargetID() {
        // FIX: Limelight broadcasts tid as a double. Read as double, then cast to int!
        return (int) m_limelightTable.getEntry("tid").getDouble(-1.0);
    }


        /** Robot X position relative to the target, in meters, from botpose_targetspace */
    public double getTargetSpaceX() {
        if (!hasTarget()) {
            return 0.0;
        }

        double[] botPoseTargetSpace =
                m_limelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6]);

        if (botPoseTargetSpace.length < 6) {
            return 0.0;
        }

        return botPoseTargetSpace[0];
    }

    /** Robot Y position relative to the target, in meters, from botpose_targetspace */
    public double getTargetSpaceY() {
        if (!hasTarget()) {
            return 0.0;
        }

        double[] botPoseTargetSpace =
                m_limelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6]);

        if (botPoseTargetSpace.length < 6) {
            return 0.0;
        }

        return botPoseTargetSpace[1];
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

        // FIX: Botpose array length must be at least 7 to include pipeline latency at index 6
        double[] botpose = m_limelightTable.getEntry(botposeKey).getDoubleArray(new double[7]);
        if (botpose.length < 7) {
            return null;
        }

        // FIX: botpose[6] is the pipeline latency in ms. 'cl' is the capture latency.
        double latencyMs = botpose[6] + m_limelightTable.getEntry("cl").getDouble(0.0);
        
        double timestamp = Timer.getFPGATimestamp() - (latencyMs / 1000.0);

        Pose2d visionPose = new Pose2d(botpose[0], botpose[1], gyroRotation);

        return new VisionMeasurement(visionPose, timestamp);
    }
}