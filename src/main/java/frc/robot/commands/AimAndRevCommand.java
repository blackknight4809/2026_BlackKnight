package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

// Make sure to import Constants so we can use your new VisionConstants!
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
        
        this.m_drive = drive;
        this.m_vision = vision;
        this.m_shooter = shooter;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_manualRotationSupplier = manualRotationSupplier;

        // 1. Create the PID Controller FIRST
        m_turnPID = new PIDController(0.05, 0.0, 0.005);
        m_turnPID.setSetpoint(0.0);

        // 2. Dashboard tuning values (Permanent)
        SmartDashboard.putNumber("Aim/kP", m_turnPID.getP());
        SmartDashboard.putNumber("Aim/kD", m_turnPID.getD());
        SmartDashboard.putNumber("Aim/YawOffsetTarget", 0.0);

        addRequirements(m_drive, m_shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // 1. Update PID constants from dashboard tuning
        m_turnPID.setP(SmartDashboard.getNumber("Aim/kP", m_turnPID.getP()));
        m_turnPID.setD(SmartDashboard.getNumber("Aim/kD", m_turnPID.getD()));

        double commandedDistance = -1.0;
        String shooterMode = "UNKNOWN";

        boolean hasTarget = m_vision.hasTarget();
        SmartDashboard.putBoolean("Aim/Seeing Target", hasTarget);

        double translationX = m_translationXSupplier.getAsDouble();
        double translationY = m_translationYSupplier.getAsDouble();
        double rotationSpeed;

 if (hasTarget) {
    int targetID = m_vision.getTargetID();

    if (Constants.VisionConstants.isOurAllianceTag(targetID)) {
        double bullseye = SmartDashboard.getNumber("Aim/YawOffsetTarget", 0.0);
        bullseye += Constants.VisionConstants.getHubOffset(targetID);

        m_turnPID.setSetpoint(bullseye);
        rotationSpeed = m_turnPID.calculate(m_vision.getYawOffset());

        double distance = m_vision.getDistanceToTarget();
        commandedDistance = distance;
        shooterMode = "VISION";
        m_shooter.setSpeedFromDistance(distance);

        SmartDashboard.putString("Aim/Status", "LOCKED ON ID: " + targetID);
    } else {
        rotationSpeed = m_manualRotationSupplier.getAsDouble();
        commandedDistance = 1.5;
        shooterMode = "WRONG_TAG";
        m_shooter.setSpeedFromDistance(1.5);

        SmartDashboard.putString("Aim/Status", "WRONG TAG (ID " + targetID + ")");
    }
} else {
    rotationSpeed = m_manualRotationSupplier.getAsDouble();
    commandedDistance = -1.0;
    shooterMode = "NO_TARGET";
    m_shooter.setSpeedFromDistance(-1.0);

    SmartDashboard.putString("Aim/Status", "NO TARGET");
}

SmartDashboard.putString("Aim/ShooterMode", shooterMode);
SmartDashboard.putNumber("Aim/CommandedDistance", commandedDistance);
        

        // Apply everything to the Swerve Drive
        m_drive.drive(translationX, translationY, rotationSpeed, true);
}

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, false);
        m_shooter.stopShooter();
        
        System.out.println(">>> AimAndRevCommand ENDED. Interrupted: " + interrupted);
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}