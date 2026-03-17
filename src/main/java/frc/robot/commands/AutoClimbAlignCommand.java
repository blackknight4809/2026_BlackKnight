package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoClimbAlignCommand extends Command {

  private final DriveSubsystem m_drive;
  private final VisionSubsystem m_vision;
  private final ClimberSubsystem m_climber;

  private enum State {
    RAISE_CLIMBER,
    ALIGN_TO_TAG,
    FINAL_APPROACH,
    READY_TO_CLIMB
    // CLIMBING
  }

  private State m_state = State.RAISE_CLIMBER;

  private int m_targetTagId = -1;

  public AutoClimbAlignCommand(
      DriveSubsystem drive,
      VisionSubsystem vision,
      ClimberSubsystem climber) {
    m_drive = drive;
    m_vision = vision;
    m_climber = climber;

    addRequirements(m_drive, m_climber);
  }

  @Override
  public void initialize() {
    m_state = State.RAISE_CLIMBER;

    SmartDashboard.putNumber("Climb Target X Offset", 0.0);
    SmartDashboard.putNumber("Climb Target Y Offset", 0.0);
    SmartDashboard.putNumber("Climb Raise Rotations", 25.0);
    SmartDashboard.putNumber("Climb Approach Speed", 0.20);
    SmartDashboard.putNumber("Climb Strafe Speed", 0.15);
    SmartDashboard.putNumber("Climb Rotation Speed", 0.15);
    SmartDashboard.putNumber("Climb X Tolerance", 0.10);
    SmartDashboard.putNumber("Climb Y Tolerance", 0.10);

    m_targetTagId = getAllianceClimbTag();

    SmartDashboard.putString("Climb State", m_state.name());
    SmartDashboard.putNumber("Climb Target Tag", m_targetTagId);
  }

  @Override
  public void execute() {
    double targetXOffset = SmartDashboard.getNumber("Climb Target X Offset", 0.0);
    double targetYOffset = SmartDashboard.getNumber("Climb Target Y Offset", 0.0);
    double raiseRotations = SmartDashboard.getNumber("Climb Raise Rotations", 25.0);
    double approachSpeed = SmartDashboard.getNumber("Climb Approach Speed", 0.20);
    double strafeSpeed = SmartDashboard.getNumber("Climb Strafe Speed", 0.15);
    double rotationSpeed = SmartDashboard.getNumber("Climb Rotation Speed", 0.15);
    double xTolerance = SmartDashboard.getNumber("Climb X Tolerance", 0.10);
    double yTolerance = SmartDashboard.getNumber("Climb Y Tolerance", 0.10);

    SmartDashboard.putString("Climb State", m_state.name());
    SmartDashboard.putNumber("Climb Target Tag", m_targetTagId);

    switch (m_state) {
      case RAISE_CLIMBER:
        // Raise climber to pre-set encoder position before driving in.
        if (!m_climber.isAtOrAboveRotations(raiseRotations)) {
          m_climber.runClimberUpManual();
        } else {
          m_climber.stopClimberAndEngageRatchet();
          m_state = State.ALIGN_TO_TAG;
        }
        break;

      case ALIGN_TO_TAG:
        boolean hasTarget = m_vision.hasTarget();
        int seenTagId = m_vision.getTargetID();
        double currentX = m_vision.getTargetSpaceX();
        double currentY = m_vision.getTargetSpaceY();

        SmartDashboard.putBoolean("Climb Has Target", hasTarget);
        SmartDashboard.putNumber("Climb Seen Tag", seenTagId);
        SmartDashboard.putNumber("Climb Current X", currentX);
        SmartDashboard.putNumber("Climb Current Y", currentY);

        if (!hasTarget || seenTagId != m_targetTagId) {
          // No valid tag yet, hold still.
          m_drive.drive(0.0, 0.0, 0.0, true);
          break;
        }

        double xError = targetXOffset - currentX;
        double yError = targetYOffset - currentY;

        SmartDashboard.putNumber("Climb X Error", xError);
        SmartDashboard.putNumber("Climb Y Error", yError);

        double xCommand = 0.0;
        double yCommand = 0.0;
        double rotCommand = 0.0;

        if (Math.abs(xError) > xTolerance) {
          xCommand = Math.copySign(approachSpeed, xError);
        }

        if (Math.abs(yError) > yTolerance) {
          yCommand = Math.copySign(strafeSpeed, yError);
        }

        // Leave rotation support ready for later if you decide you want it.
        // rotCommand = ... use yaw / tx / heading correction here later
        rotCommand = 0.0;

        m_drive.drive(xCommand, yCommand, rotCommand, true);

        if (Math.abs(xError) <= xTolerance && Math.abs(yError) <= yTolerance) {
          m_drive.drive(0.0, 0.0, 0.0, true);
          m_state = State.FINAL_APPROACH;
        }
        break;

      case FINAL_APPROACH:
        // This is intentionally gentle and temporary.
        // Once you test, you can replace this with a better final move or
        // a tighter second-stage offset.
        m_drive.drive(0.10, 0.0, 0.0, true);

        // For now, we immediately move to READY_TO_CLIMB.
        // After testing, this could become:
        // - timer based
        // - distance based
        // - another offset check
        m_state = State.READY_TO_CLIMB;
        break;

      case READY_TO_CLIMB:
        m_drive.drive(0.0, 0.0, 0.0, true);

        // Leave actual climb commented until numbers are known.
        /*
        m_climber.disengageRatchet();
        m_climber.setClimberSpeed(-0.5);
        */

        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0.0, 0.0, 0.0, true);
    m_climber.stopClimber();
    m_climber.engageRatchet();
    SmartDashboard.putString("Climb State", interrupted ? "INTERRUPTED" : "ENDED");
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private int getAllianceClimbTag() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Blue) {
        return 32;
      } else if (alliance.get() == DriverStation.Alliance.Red) {
        return 16;
      }
    }

    // Safe fallback if alliance is not available yet
    return 32;
  }
}