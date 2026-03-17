package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
        public static final SparkFlexConfig turningConfig = new SparkFlexConfig();

        static {
            // Conversion factors
            double drivingFactor =
                    ModuleConstants.kWheelDiameterMeters * Math.PI
                            / ModuleConstants.kDrivingMotorReduction;

            double turningFactor = 2 * Math.PI;

            // Feedforward for drive velocity control
            // Using 12V here matches the template behavior that drove correctly
            double nominalVoltage = 12.0;
            double drivingVelocityFeedForward =
                    nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

            /* -------------------- DRIVE MOTOR CONFIG -------------------- */

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);

            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor)         // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters/sec

            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(0.04, 0, 0)
                    .outputRange(-1, 1)
                    .velocityFF(drivingVelocityFeedForward);

            /* -------------------- TURN MOTOR CONFIG -------------------- */

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);

            turningConfig.absoluteEncoder
                    .inverted(true) // required for MAXSwerve module geometry
                    .positionConversionFactor(turningFactor)         // radians
                    .velocityConversionFactor(turningFactor / 60.0)
                    .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .pid(1.0, 0, 0)
                    .outputRange(-1, 1)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }
}