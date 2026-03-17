package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimAndRevCommand extends Command {
    private final DriveSubsystem m_drive;
    private final VisionSubsystem m_vision;
    private final ShooterSubsystem m_shooter;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_manualRotationSupplier;

    private final PIDController m_turnPID;

    public AimAndRevCommand(
            DriveSubsystem drive,
            VisionSubsystem vision,
            ShooterSubsystem shooter,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier manualRotationSupplier) {

        m_drive = drive;
        m_vision = vision;
        m_shooter = shooter;
        m_translationXSupplier = translationXSupplier;
        m_translationYSupplier = translationYSupplier;
        m_manualRotationSupplier = manualRotationSupplier;

        m_turnPID = new PIDController(0.01, 0.0, 0.001);
        m_turnPID.setTolerance(1.0);
        m_turnPID.setSetpoint(0.0);

        SmartDashboard.putNumber("Aim/kP", m_turnPID.getP());
        SmartDashboard.putNumber("Aim/kD", m_turnPID.getD());
        SmartDashboard.putNumber("Aim/YawOffsetTarget", 0.0);
        SmartDashboard.putBoolean("Aim/UseAllianceTagCheck", false);
        SmartDashboard.putNumber("Aim/MaxTurnSpeed", 0.30);

        addRequirements(m_drive, m_shooter);
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

    boolean hasTarget = m_vision.hasTarget();
    int targetID = m_vision.getTargetID();
    double tx = m_vision.getYawOffset();

    boolean useAllianceCheck = SmartDashboard.getBoolean("Aim/UseAllianceTagCheck", false);
    boolean tagAllowed = !useAllianceCheck || Constants.VisionConstants.isOurAllianceTag(targetID);

    double rotationSpeed = manualRotation;
    double commandedDistance = -1.0;
    String shooterMode = "NO_TARGET";

    var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
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
        rotationSpeed = m_manualRotationSupplier.getAsDouble();
        commandedDistance = 2.5;
        shooterMode = "WRONG_TAG";
        m_shooter.setSpeedFromDistance(2.5);

        SmartDashboard.putString("Aim/Status", "WRONG TAG (ID " + targetID + ")");
        SmartDashboard.putNumber("Aim/Setpoint", 0.0);
        SmartDashboard.putNumber("Aim/Error", 0.0);
        SmartDashboard.putNumber("Aim/RotCmd", rotationSpeed);
    } else {
        rotationSpeed = m_manualRotationSupplier.getAsDouble();
        commandedDistance = 2.5;
        shooterMode = "NO_TARGET";
        m_shooter.setSpeedFromDistance(2.5);

        SmartDashboard.putString("Aim/Status", "NO TARGET");
        SmartDashboard.putNumber("Aim/Setpoint", 0.0);
        SmartDashboard.putNumber("Aim/Error", 0.0);
        SmartDashboard.putNumber("Aim/RotCmd", rotationSpeed);
    }

    SmartDashboard.putString("Aim/ShooterMode", shooterMode);
    SmartDashboard.putNumber("Aim/CommandedDistance", commandedDistance);

    m_drive.drive(translationX, translationY, rotationSpeed, false);
}

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, false);
        m_shooter.stopShooter();
        m_turnPID.reset();

        System.out.println(">>> AimAndRevCommand ENDED. Interrupted: " + interrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}