package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.AimAndRevCommand;
import frc.robot.commands.RunFeederCommand;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class RobotContainer {
  // --- Subsystems ---
  private final VisionSubsystem m_vision = new VisionSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_vision);
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();


  
  @SuppressWarnings("unused")
  private final PneumaticsSubsystem m_pneumatics = new PneumaticsSubsystem();
  private final PickupSubsystem m_pickup = new PickupSubsystem();

  // Driver controller
  private final XboxController m_driverController =
      new XboxController(OIConstants.kDriverControllerPort);

  // Auto chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    // 1. INITIALIZE SMARTDASHBOARD VALUES
    // This creates the entry. If it already exists, it uses the current value.
    SmartDashboard.putNumber("Shooter Test Power", 0.5); // Default to 50%

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(-m_driverController.getRightX(), 0.12),
                false),
            m_robotDrive));

    
    // Turn off Limelight LEDs on startup
    m_robotDrive.setLimelightLED(1);
    configureButtonBindings();

    autoChooser.setDefaultOption("Default", AutoBuilder.buildAuto("Default_Auto"));
    autoChooser.addOption("Test_Auto", AutoBuilder.buildAuto("Test_Auto"));
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void configureButtonBindings() {
    // Right Bumper: Lock wheels
    JoystickButton lockX =
        new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
    lockX.onTrue(new InstantCommand(m_robotDrive::setX, m_robotDrive));
    lockX.onFalse(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false), m_robotDrive));

    // Zero Heading
    new JoystickButton(m_driverController, XboxController.Button.kBack.value)
        .onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));

    // Left Bumper: Toggle Intake
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .toggleOnTrue(
            m_pickup.deployAndRunCommand()
            .andThen(Commands.idle(m_pickup)) 
            .finallyDo((interrupted) -> m_pickup.retractAndStopCommand().schedule())
        );

    // Left Trigger: Aim and Rev
    new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.5)
        .whileTrue(new AimAndRevCommand(
            m_robotDrive, m_vision, m_shooter, 
            () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kDriveDeadband),
            () -> MathUtil.applyDeadband(-m_driverController.getRightX(), OIConstants.kDriveDeadband)
        ));

    // Right Trigger: Fire
    new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.5)
        .whileTrue(new RunFeederCommand(m_shooter, m_pickup));

// --- TUNING OVERRIDE in RobotContainer.java ---
new JoystickButton(m_driverController, XboxController.Button.kB.value)
    .whileTrue(new RunCommand(
        () -> {
            // Get the value, defaulting to 0.0 if it's missing
            double speed = SmartDashboard.getNumber("Shooter Test Power", 0.0);
            
            // Clamp the value for safety so a typo doesn't break things
            speed = MathUtil.clamp(speed, -1.0, 1.0);
            
            m_shooter.setShooterRaw(speed);
        
        },
        m_shooter
    ).finallyDo((interrupted) -> m_shooter.stopShooter()));

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
    .whileTrue(new RunCommand(
        () -> {
            m_shooter.setShooterSpeed(4400);
            m_shooter.runShooter();
        },
        m_shooter
    ).finallyDo((interrupted) -> m_shooter.stopShooter()));

    // X Button: Toggle Pistons
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .toggleOnTrue(
            new StartEndCommand(
                () -> { m_pickup.extendPickup(); m_pickup.extendRoller(); }, 
                () -> { m_pickup.retractPickup(); m_pickup.retractRoller(); }, 
                m_pickup
            )
        );
  }

  

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}