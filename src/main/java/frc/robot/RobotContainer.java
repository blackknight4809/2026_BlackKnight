package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.AimAndRevCommand;
import frc.robot.commands.AimDriveAndShootCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.RunFeederCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.commands.AutoAimAndShootCommand;
import frc.robot.commands.AutoClimbAlignCommand;

public class RobotContainer {

  // Subsystems
  private final VisionSubsystem m_vision = new VisionSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_vision);
  private final PickupSubsystem m_pickup = new PickupSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final PneumaticsSubsystem m_pneumatics = new PneumaticsSubsystem();

  private UsbCamera m_pickupCamera;

  // Pickup toggle state
  private boolean m_pickupDeployed = false;

  // Driver controller
  private final XboxController m_driverController =
      new XboxController(OIConstants.kDriverControllerPort);
  

  // Auto chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {

m_pickupCamera = CameraServer.startAutomaticCapture("Pickup Cam", 0);
m_pickupCamera.setResolution(320, 240);
m_pickupCamera.setFPS(15);
    // Default drive command
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(-m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

    configureButtonBindings();

      NamedCommands.registerCommand(
      "AutoAimAndShoot",
      new AutoAimAndShootCommand(
          m_robotDrive,
          m_vision,
          m_shooter,
          m_pickup));

    autoChooser.setDefaultOption("Default", AutoBuilder.buildAuto("Default_Auto"));
    autoChooser.addOption("Test_Auto", AutoBuilder.buildAuto("Test_Auto"));
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void configureButtonBindings() {

    // =========================
    // DRIVE CONTROLS
    // =========================


new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(
            edu.wpi.first.wpilibj2.command.Commands.startEnd(
                () -> m_shooter.runFeeder(), // What to do when you PRESS X
                () -> m_shooter.stopFeeder(), // What to do when you RELEASE X
                m_shooter // The subsystem it requires
            )
        );


    // =========================
    // AIM + SHOOT (LEFT TRIGGER)
    // =========================

Trigger leftTrigger = new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.5);

leftTrigger.whileTrue(
    new AimDriveAndShootCommand(
        m_robotDrive,
        m_vision,
        m_shooter,
        m_pickup,
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.kDriveDeadband),
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kDriveDeadband),
        () -> MathUtil.applyDeadband(-m_driverController.getRightX(), OIConstants.kDriveDeadband),
        () -> m_driverController.getRightTriggerAxis() > 0.5));

    //rest yaw
        new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));

    // =========================
    // PICKUP TOGGLE (LEFT BUMPER)
    // =========================

    JoystickButton pickupButton =
        new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);

    pickupButton.onTrue(new InstantCommand(() -> {
      if (m_pickupDeployed) {
        m_pickup.retractAndStopCommand().schedule();
        m_pickupDeployed = false;
      } else {
        m_pickup.deployAndRunCommand().schedule();
        m_pickupDeployed = true;
      }
    }));

    //Button for Shooter Raw

new JoystickButton(m_driverController, XboxController.Button.kB.value)
    .whileTrue(
        new RunCommand(
            () -> m_shooter.setShooterRaw(
                SmartDashboard.getNumber("Shooter Test Power", 0.5)),
            m_shooter))
    .onFalse(new InstantCommand(() -> m_shooter.stopShooter(), m_shooter));


    //Climber Buttons

    //Auto Climb Button

        new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(
            new AutoClimbAlignCommand(
                m_robotDrive,
                m_vision,
                m_climberSubsystem));
    new Trigger(() -> m_driverController.getPOV() == 0)
        .whileTrue(
            edu.wpi.first.wpilibj2.command.Commands.startEnd(
                () -> m_climberSubsystem.runClimberUpManual(),
                () -> m_climberSubsystem.stopClimberAndEngageRatchet(),
                m_climberSubsystem));

    //Manual Climb Button

    new Trigger(() -> m_driverController.getPOV() == 180)
        .whileTrue(
            edu.wpi.first.wpilibj2.command.Commands.startEnd(
                () -> m_climberSubsystem.runClimberDownManual(),
                () -> m_climberSubsystem.stopClimberAndEngageRatchet(),
                m_climberSubsystem));




  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}