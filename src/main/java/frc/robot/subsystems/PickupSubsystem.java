package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.Constants.PickupConstants;

public class PickupSubsystem extends SubsystemBase {

  private enum RollerMode {
    OFF,
    PICKUP,
    FEEDER
  }

  private final SparkMax m_intakeMotor;
  private final SparkMax m_rollerMotor;

  private final DoubleSolenoid m_pickupSolenoid;
  private final DoubleSolenoid m_rollerSolenoid;

  private RollerMode m_rollerMode = RollerMode.OFF;

  public PickupSubsystem() {
    m_intakeMotor = new SparkMax(PickupConstants.kIntakeCanId, MotorType.kBrushed);
    m_rollerMotor = new SparkMax(PickupConstants.kRollerCanId, MotorType.kBrushless);

    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.smartCurrentLimit(30);

    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.smartCurrentLimit(40);

    m_intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_pickupSolenoid = new DoubleSolenoid(
        PneumaticsModuleType.REVPH,
        PickupConstants.kPickupDeployPort,
        PickupConstants.kPickupRetractPort
    );

    m_rollerSolenoid = new DoubleSolenoid(
        PneumaticsModuleType.REVPH,
        PickupConstants.kRollerDeployPort,
        PickupConstants.kRollerRetractPort
    );

    retractPickup();
    retractRoller();
  }

  private void applyRollerMode() {
    switch (m_rollerMode) {
      case FEEDER:
        m_rollerMotor.set(PickupConstants.kRollerFeederSpeed);
        break;
      case PICKUP:
        m_rollerMotor.set(PickupConstants.kRollerPickupSpeed);
        break;
      case OFF:
      default:
        m_rollerMotor.stopMotor();
        break;
    }
  }

  public void runMotors() {
    m_intakeMotor.set(PickupConstants.kIntakeSpeed);
    requestPickupRoller();
  }

  public void stopMotors() {
    m_intakeMotor.stopMotor();
    releasePickupRoller();
  }

  public void requestPickupRoller() {
    if (m_rollerMode != RollerMode.FEEDER) {
      m_rollerMode = RollerMode.PICKUP;
      applyRollerMode();
    }
  }

  public void releasePickupRoller() {
    if (m_rollerMode == RollerMode.PICKUP) {
      m_rollerMode = RollerMode.OFF;
      applyRollerMode();
    }
  }

  public void requestFeederRoller() {
    m_rollerMode = RollerMode.FEEDER;
    applyRollerMode();
  }

  public void releaseFeederRoller() {
    if (m_rollerMode == RollerMode.FEEDER) {
      m_rollerMode = RollerMode.OFF;
      applyRollerMode();
    }
  }

  public void extendPickup() {
    m_pickupSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retractPickup() {
    m_pickupSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void extendRoller() {
    m_rollerSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retractRoller() {
    m_rollerSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public Command deployAndRunCommand() {
    return Commands.sequence(
        this.runOnce(this::extendPickup),
        Commands.waitSeconds(0.25),
        this.runOnce(() -> {
          extendRoller();
          runMotors();
        })
    ).withName("DeployPickupSequence");
  }

  public Command retractAndStopCommand() {
    return Commands.sequence(
        this.runOnce(() -> {
          stopMotors();
          retractRoller();
        }),
        Commands.waitSeconds(0.25),
        this.runOnce(this::retractPickup)
    );
  }

  @Override
  public void periodic() {}
}