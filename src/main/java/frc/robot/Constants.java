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
 * numerical or boolean constants. This class should not be used for any other purpose.
 * All constants should be declared globally (i.e. public static). Do not put anything
 * functional in this class.
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
    // Steering offsets align each module's encoder zero to the robot's forward direction.
    // Right-side modules use their raw calibrated offsets.
    // Front-left drive motor is physically mounted in reverse, so invertDrive=true
    // is passed in DriveSubsystem -- the angular offset for FL does NOT include a
    // +PI flip anymore since inversion is handled electrically via the SparkFlex config.
    //
    //   Front-Right base: 0
    //   Back-Right  base: +PI/2
    //   Front-Left  base: -PI/2   (drive inversion handled by kFrontLeftDriveInverted)
    //   Back-Left   base: +PI     (was PI+PI=2*PI=0, then 3*PI/2 -- now corrected to PI
    //                              because the 90-deg CCW error means we add PI/2 back,
    //                              and without a +PI flip the true offset is just PI)
    public static final double kFrontLeftChassisAngularOffset  = -Math.PI / 2.0;
    public static final double kFrontRightChassisAngularOffset = 0.0;
    public static final double kBackLeftChassisAngularOffset   = Math.PI;
    public static final double kBackRightChassisAngularOffset  = Math.PI / 2.0;

    // Drive motor inversion flags -- passed as the 4th arg to MAXSwerveModule.
    // Only front-left is inverted on this robot.
    public static final boolean kFrontLeftDriveInverted  = false; // Motor was already inverted in flash; double-invert cancels out
    public static final boolean kFrontRightDriveInverted = false;
    public static final boolean kBackLeftDriveInverted   = false;
    public static final boolean kBackRightDriveInverted  = false;

    public static final int kFrontLeftDrivingCanId  = 3;
    public static final int kRearLeftDrivingCanId   = 5;
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kRearRightDrivingCanId  = 7;

    public static final int kFrontLeftTurningCanId  = 9;
    public static final int kRearLeftTurningCanId   = 4;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kRearRightTurningCanId  = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ShooterConstants {
    public static final int kShooterCanId   = 10;
    public static final int kFeederCanId    = 11;
    public static final int kAgitatorCanId  = 14;

    public static final double kFeederSpeed   = 1.0;
    public static final double kAgitatorSpeed = 0.4;

    public static final double kMaxShooterSpeed = 1.0;    // 100% output
    public static final double kShooterIdleSpeed = 0.2;   // Keep spinning slow for faster spin-up
    public static final double kMaxShooterRpm  = 5676.0;
    public static final double kShooterIdleRpm = 1500.0;

    // Flywheel PID Constants
    public static final double kP  = .00023;
    public static final double kI  = .000001;
    public static final double kD  = 0.000;
    public static final double kFF = .00026 / kMaxShooterRpm;

    // Distance to Speed Map: {Distance in Meters, Target RPM}
    public static final double[][] kDistanceMap = {
        {1.5, 2500.0},
        {2.5, 3500.0},
        {3.5, 4500.0},
        {5.0, 5500.0}
    };
  }

  public static final class PickupConstants {
    // Motor CAN IDs
    public static final int kIntakeCanId  = 12;
    public static final int kIntake2CanId = 15;
    public static final int kRollerCanId  = 13;

    // Motor Speeds
    public static final double kIntakeSpeed       = -0.5;
    public static final double kRollerPickupSpeed = -0.5;
    public static final double kRollerFeederSpeed =  0.1;

    // Pneumatics (REV Pneumatics Hub)
    public static final int kPickupDeployPort  = 2;
    public static final int kPickupRetractPort = 3;
    public static final int kRollerDeployPort  = 0;
    public static final int kRollerRetractPort = 1;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm       = 5676;
    public static final double kVortexFreeSpeedRpm = 6784;
  }

  public static final class ModuleConstants {
    // MAXSwerve pinion gear teeth: 12T, 13T, or 14T
    public static final int kDrivingMotorPinionTeeth = 14;

    // Using Vortex free speed for drive motor calculations
    public static final double kDrivingMotorFreeSpeedRps   = NeoMotorConstants.kVortexFreeSpeedRpm / 60.0;
    public static final double kWheelDiameterMeters        = 0.0762;
    public static final double kWheelCircumferenceMeters   = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear,
    // 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction      = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps     =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int    kDriverControllerPort = 0;
    public static final double kDriveDeadband        = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond                  = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared     = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond           = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared    = Math.PI;

    public static final double kPXController     = 1;
    public static final double kPYController     = 1;
    public static final double kPThetaController = 1;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularSpeedRadiansPerSecondSquared);
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

    /** Returns true if the ID is a CENTER tag for either alliance. */
    public static boolean isCenterTag(int id) {
      return (id == 2 || id == 5 || id == 10 || id == 18 || id == 21 || id == 26);
    }

    /** Returns true if the tag belongs to our current alliance's Hub. */
    public static boolean isOurAllianceTag(int id) {
      var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
      if (alliance.isEmpty()) return false;

      boolean isRed = alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;

      if (isRed) {
        return (id == 2 || id == 11 || id == 10 || id == 9 || id == 8 || id == 5);
      } else {
        return (id == 18 || id == 27 || id == 26 || id == 25 || id == 21 || id == 24);
      }
    }

    /**
     * Returns the geometric offset needed to hit the center of the Hub
     * based on which specific tag the camera is looking at.
     */
    public static double getHubOffset(int id) {
      // "Left" side tags — robot needs to aim slightly right (negative offset)
      if (id == 9 || id == 8 || id == 11 || id == 27 || id == 25 || id == 24) {
        return -3.0;
      }
      // Center tags (or unknown) need no offset
      return 0.0;
    }
  }
}