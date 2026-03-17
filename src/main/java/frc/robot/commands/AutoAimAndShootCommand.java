package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAimAndShootCommand extends Command {
    private final DriveSubsystem m_drive;
    private final VisionSubsystem m_vision;
    private final ShooterSubsystem m_shooter;
    private final PickupSubsystem m_pickup;

    private final PIDController m_turnPID;
    private final Timer m_feedTimer = new Timer();
    private final Timer m_readyTimer = new Timer();

    private final double m_feedTimeSeconds = 10.0;
    private final double m_shooterToleranceRpm = 100.0;
    private final double m_readyHoldTimeSeconds = 0.15;

    private boolean m_hasSeenTarget = false;
    private boolean m_feeding = false;

    public AutoAimAndShootCommand(
            DriveSubsystem drive,
            VisionSubsystem vision,
            ShooterSubsystem shooter,
            PickupSubsystem pickup) {

        m_drive = drive;
        m_vision = vision;
        m_shooter = shooter;
        m_pickup = pickup;

        m_turnPID = new PIDController(0.01, 0.0, 0.001);
        m_turnPID.setTolerance(1.0);
        m_turnPID.setSetpoint(0.0);

        addRequirements(m_drive, m_shooter, m_pickup);
    }

    @Override
    public void initialize() {
        m_turnPID.reset();

        m_feedTimer.stop();
        m_feedTimer.reset();

        m_readyTimer.stop();
        m_readyTimer.reset();

        m_feeding = false;
        m_hasSeenTarget = false;
    }

    @Override
    public void execute() {
        boolean hasTarget = m_vision.hasTarget();
        int targetID = m_vision.getTargetID();
        double tx = m_vision.getYawOffset();

        boolean validTarget = hasTarget && Constants.VisionConstants.isOurAllianceTag(targetID);

        if (validTarget) {
            m_hasSeenTarget = true;
        }

        double rotationSpeed = 0.0;

        if (!m_feeding && validTarget) {
            double rawTurn = m_turnPID.calculate(tx);
            rotationSpeed = MathUtil.clamp(rawTurn, -0.3, 0.3);

            if (m_turnPID.atSetpoint()) {
                rotationSpeed = 0.0;
            }

            double distance = m_vision.getDistanceToTarget();
            m_shooter.setSpeedFromDistance(distance);
        } else if (!m_feeding) {
            m_drive.drive(0.0, 0.0, 0.0, false);
            m_shooter.setSpeedFromDistance(2.5);
        }

        boolean readyToShootNow =
                m_hasSeenTarget
                        && validTarget
                        && m_turnPID.atSetpoint()
                        && m_shooter.isShooterAtSpeed(m_shooterToleranceRpm);

        if (!m_feeding) {
            if (readyToShootNow) {
                if (!m_readyTimer.isRunning()) {
                    m_readyTimer.reset();
                    m_readyTimer.start();
                }
            } else {
                m_readyTimer.stop();
                m_readyTimer.reset();
            }
        }

        if (!m_feeding && m_readyTimer.hasElapsed(m_readyHoldTimeSeconds)) {
            m_feeding = true;
            m_feedTimer.reset();
            m_feedTimer.start();

            m_shooter.runFeeder();
            m_shooter.runAgitator();
            m_pickup.requestFeederRoller();
        }

        if (m_feeding) {
            m_drive.drive(0.0, 0.0, 0.0, false);
            m_shooter.setSpeedFromDistance(m_vision.getDistanceToTarget());
        } else {
            m_drive.drive(0.0, 0.0, rotationSpeed, false);
        }

        SmartDashboard.putBoolean(
                "AutoShoot/ShooterAtSpeed",
                m_shooter.isShooterAtSpeed(m_shooterToleranceRpm));
        SmartDashboard.putBoolean("AutoShoot/ReadyNow", readyToShootNow);
        SmartDashboard.putBoolean("AutoShoot/Feeding", m_feeding);
    }

    @Override
    public boolean isFinished() {
        return m_feeding && m_feedTimer.hasElapsed(m_feedTimeSeconds);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0.0, 0.0, 0.0, false);

        m_shooter.stopFeeder();
        m_shooter.stopAgitator();
        m_shooter.stopShooter();

        m_pickup.releaseFeederRoller();

        m_feedTimer.stop();
        m_feedTimer.reset();

        m_readyTimer.stop();
        m_readyTimer.reset();

        m_turnPID.reset();
    }
}