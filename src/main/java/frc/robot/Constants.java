// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

// CHASSIS ANGULAR OFFSETS
    // We are adding/subtracting PI (180 degrees) to the left side to flip the drive direction.
    
public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
public static final double kFrontRightChassisAngularOffset = 0;
public static final double kBackLeftChassisAngularOffset = Math.PI;
public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final int kFrontLeftDrivingCanId = 5;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kRearRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 9;
    public static final int kRearLeftTurningCanId = 4;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ShooterConstants {
    public static final int kShooterCanId = 10;   // Example ID
    public static final int kFeederCanId = 11;    // Example ID
    public static final int kAgitatorCanId = 14;  // Example ID

    public static final double kFeederSpeed = 1;
    public static final double kAgitatorSpeed = 0.4;
    // In Constants.java
    public static final double kMaxShooterSpeed = 1.0; // 100% output
    public static final double kShooterIdleSpeed = 0.2; // Keep it spinning slow so it spins up faster
    public static final double kMaxShooterRpm = 5676.0; 
    public static final double kShooterIdleRpm = 1500.0; // Idle in RPM now

    // Flywheel PID Constants
    public static final double kP = .00023; // Muscle: Fixes small errors
    public static final double kI = .000001;
    public static final double kD = 0.000;
    // Feedforward is the secret to flywheels! It guesses the exact power needed.
    public static final double kFF = .00026 / kMaxShooterRpm; 

    // NEW Distance to Speed Map: {Distance in Meters, Target RPM}
    public static final double[][] kDistanceMap = {
        {1.5, 2500.0}, // Closer = Slower RPM
        {2.5, 3500.0}, 
        {3.5, 4500.0}, 
        {5.0, 5500.0}  // Farther = Faster RPM
    };
  }
    
  public static final class PickupConstants {
    // Motor CAN IDs
    public static final int kIntakeCanId = 12;
    public static final int kIntake2CanId = 15; // NEW: Second intake motor
    public static final int kRollerCanId = 13;

    // Motor Speeds
    public static final double kIntakeSpeed = -.5;
    public static final double kRollerPickupSpeed = -0.5;
    public static final double kRollerFeederSpeed = 0.1;

    // Pneumatics (REV Pneumatics Hub)
    public static final int kPickupDeployPort = 2;   // Forward channel
    public static final int kPickupRetractPort = 3;  // Reverse channel
    
    public static final int kRollerDeployPort = 0;   // Forward channel
    public static final int kRollerRetractPort = 1;  // Reverse channel
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VisionConstants {
    // --- RED ALLIANCE HUB TAGS ---
    // Format: {CenterTag, LeftTag}
    public static final int[] kRedHubFaceA = {5, 8}; 
    public static final int[] kRedHubFaceB = {10, 9};
    public static final int[] kRedHubFaceC = {2, 11};

    // --- BLUE ALLIANCE HUB TAGS ---
    // Format: {CenterTag, LeftTag}
    public static final int[] kBlueHubFaceA = {18, 27}; 
    public static final int[] kBlueHubFaceB = {26, 25};
    public static final int[] kBlueHubFaceC = {21, 24};

    /** * Helper: Returns true if the ID is a CENTER tag for either alliance.
     * This makes your 'Aim' logic very simple!
     */
    public static boolean isCenterTag(int id) {
        return (id == 2 || id == 5 || id == 10 || id == 18 || id == 21 || id == 26);
    }

    /** * Helper: Returns true if the tag belongs to our current alliance's Hub.
     */
    public static boolean isOurAllianceTag(int id) {
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        if (alliance.isEmpty()) return false;

        boolean isRed = alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
        
        if (isRed) {
            // Strictly check for Red Hub IDs
            return (id == 2 || id == 11 || id == 10 || id == 9 || id == 8 || id == 5); 
        } else {
            // Strictly check for Blue Hub IDs
            return (id == 18 || id == 27 || id == 26 || id == 25 || id == 21 || id == 24); 
        }
    }
    
    /** * Returns the geometric offset needed to hit the center of the Hub 
     * based on which specific tag the camera is looking at.
     */
    public static double getHubOffset(int id) {
        // "Left" Side Tags (Robot needs to aim slightly Right, so we use a negative offset)
        if (id == 9 || id == 8 || id == 11 || id == 27 || id == 25 || id == 24) {
            return -3.0; 
        }
        
        // Center tags (or unknown tags) need 0 offset
        return 0.0; 
    }
  }
}