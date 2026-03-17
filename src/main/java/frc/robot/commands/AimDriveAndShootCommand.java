package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimDriveAndShootCommand extends Command {
    private final DriveSubsystem m_drive;
    private final VisionSubsystem m_vision;
    private final ShooterSubsystem m_shooter;
    private final PickupSubsystem m_pickup;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_manualRotationSupplier;
    private final BooleanSupplier m_feedSupplier;

    private final PIDController m_turnPID;

    public AimDriveAndShootCommand(
            DriveSubsystem drive,
            VisionSubsystem vision,
            ShooterSubsystem shooter,
            PickupSubsystem pickup,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier manualRotationSupplier,
            BooleanSupplier feedSupplier) {

        m_drive = drive;
        m_vision = vision;
        m_shooter = shooter;
        m_pickup = pickup;
        m_translationXSupplier = translationXSupplier;
        m_translationYSupplier = translationYSupplier;
        m_manualRotationSupplier = manualRotationSupplier;
        m_feedSupplier = feedSupplier;

        m_turnPID = new PIDController(0.01, 0.0, 0.001);
        m_turnPID.setTolerance(1.0);
        m_turnPID.setSetpoint(0.0);

        SmartDashboard.putNumber("Aim/kP", m_turnPID.getP());
        SmartDashboard.putNumber("Aim/kD", m_turnPID.getD());
        SmartDashboard.putNumber("Aim/YawOffsetTarget", 0.0);
        SmartDashboard.putBoolean("Aim/UseAllianceTagCheck", false);
        SmartDashboard.putNumber("Aim/MaxTurnSpeed", 0.30);

        addRequirements(m_drive, m_shooter, m_pickup);
    }

    @Override
    public void initialize() {
        m_turnPID.reset();
    }

    @Override
    public void execute() {
        m_turnPID.setP(SmartDashboard.getNumber("Aim/kP", m_turnPID.getP()));
        m_turnPID.setD(SmartDashboard.getNumber("Aim/kD", m_turnPID.getD()));

        double translationX = m_translationXSupplier.getAsDouble();
        double translationY = m_translationYSupplier.getAsDouble();
        double manualRotation = m_manualRotationSupplier.getAsDouble();
        boolean wantsToFeed = m_feedSupplier.getAsBoolean();

        boolean hasTarget = m_vision.hasTarget();
        int targetID = m_vision.getTargetID();
        double tx = m_vision.getYawOffset();

        boolean useAllianceCheck = SmartDashboard.getBoolean("Aim/UseAllianceTagCheck", false);

        double rotationSpeed = manualRotation;
        double commandedDistance = -1.0;
        String shooterMode = "NO_TARGET";

        var alliance = DriverStation.getAlliance();
        String allianceString = alliance.isEmpty() ? "EMPTY" : alliance.get().toString();

        SmartDashboard.putString("Aim/Diag_Alliance", allianceString);
        SmartDashboard.putBoolean("Aim/Seeing Target", hasTarget);
        SmartDashboard.putNumber("Aim/Diag_SeenID", targetID);
        SmartDashboard.putBoolean("Aim/Diag_IsOurTag", Constants.VisionConstants.isOurAllianceTag(targetID));
        SmartDashboard.putNumber("Aim/tx", tx);

        if (hasTarget && Constants.VisionConstants.isOurAllianceTag(targetID)) {
            double bullseye = SmartDashboard.getNumber("Aim/YawOffsetTarget", 0.0);

            if (useAllianceCheck) {
                bullseye += Constants.VisionConstants.getHubOffset(targetID);
            }

            m_turnPID.setSetpoint(bullseye);

            double rawTurn = m_turnPID.calculate(tx);
            double maxTurn = SmartDashboard.getNumber("Aim/MaxTurnSpeed", 0.30);
            rotationSpeed = MathUtil.clamp(rawTurn, -maxTurn, maxTurn);

            if (m_turnPID.atSetpoint()) {
                rotationSpeed = 0.0;
            }

            double distance = m_vision.getDistanceToTarget();
            commandedDistance = distance;
            shooterMode = "VISION";
            m_shooter.setSpeedFromDistance(distance);

            SmartDashboard.putString("Aim/Status", "LOCKED ON ID: " + targetID);
            SmartDashboard.putNumber("Aim/Setpoint", bullseye);
            SmartDashboard.putNumber("Aim/Error", bullseye - tx);
            SmartDashboard.putNumber("Aim/RotCmd", rotationSpeed);
        } else if (hasTarget) {
            rotationSpeed = manualRotation;
            commandedDistance = 2.5;
            shooterMode = "WRONG_TAG";
            m_shooter.setSpeedFromDistance(2.5);

            SmartDashboard.putString("Aim/Status", "WRONG TAG (ID " + targetID + ")");
            SmartDashboard.putNumber("Aim/Setpoint", 0.0);
            SmartDashboard.putNumber("Aim/Error", 0.0);
            SmartDashboard.putNumber("Aim/RotCmd", rotationSpeed);
        } else {
            rotationSpeed = manualRotation;
            commandedDistance = 2.5;
            shooterMode = "NO_TARGET";
            m_shooter.setSpeedFromDistance(2.5);

            SmartDashboard.putString("Aim/Status", "NO TARGET");
            SmartDashboard.putNumber("Aim/Setpoint", 0.0);
            SmartDashboard.putNumber("Aim/Error", 0.0);
            SmartDashboard.putNumber("Aim/RotCmd", rotationSpeed);
        }

boolean shooterReady = true;

if (wantsToFeed) {
    m_shooter.runFeeder();
    m_shooter.runAgitator();
    m_pickup.requestFeederRoller();
} else {
    m_shooter.stopFeeder();
    m_shooter.stopAgitator();
    m_pickup.releaseFeederRoller();
}

        SmartDashboard.putString("Aim/ShooterMode", shooterMode);
        SmartDashboard.putNumber("Aim/CommandedDistance", commandedDistance);
        SmartDashboard.putBoolean("Aim/WantsFeed", wantsToFeed);
        SmartDashboard.putBoolean("Aim/FeedingAllowed", true);

        m_drive.drive(translationX, translationY, rotationSpeed, false);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, false);
        m_shooter.stopFeeder();
        m_shooter.stopAgitator();
        m_pickup.releaseFeederRoller();
        m_shooter.stopShooter();
        m_turnPID.reset();

        System.out.println(">>> AimDriveAndShootCommand ENDED. Interrupted: " + interrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}