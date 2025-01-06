// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.Base;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Base base = new Base();

  private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(base);
  // Game Controllers
  public static Joystick logitech;
  public static Joystick compStreamDeck;
  public static Joystick testStreamDeck;
  public static Joystick autonTestStreamDeck;
  public static XboxController xbox;
  
  // Controller Buttons/Triggers
  public JoystickButton logitechBtnX, logitechBtnA, logitechBtnB, logitechBtnY, logitechBtnLB, logitechBtnRB,
      logitechBtnLT, logitechBtnRT; // Logitech Button

  public JoystickButton xboxBtnA, xboxBtnB, xboxBtnX, xboxBtnY, xboxBtnLB, xboxBtnRB, xboxBtnStrt, xboxBtnSelect;

  public Trigger xboxBtnRT, xboxBtsnLT;

  public JoystickButton compStreamDeck1, compStreamDeck2, compStreamDeck3, compStreamDeck4, compStreamDeck5,
      compStreamDeck6, compStreamDeck7, compStreamDeck8, compStreamDeck9, compStreamDeck10, compStreamDeck11,
      compStreamDeck12, compStreamDeck13,
      compStreamDeck14, compStreamDeck15;

  // Top Left SD = 1, numbered from left to right
  public JoystickButton testStreamDeck1, testStreamDeck2, testStreamDeck3, testStreamDeck4, testStreamDeck5,
      testStreamDeck6, testStreamDeck7,
      testStreamDeck8, testStreamDeck9, // Vjoy 2
      testStreamDeck10, testStreamDeck11, testStreamDeck12, testStreamDeck13, testStreamDeck14, testStreamDeck15;

  public JoystickButton autonTestStreamDeck1, autonTestStreamDeck2, autonTestStreamDeck3, autonTestStreamDeck4,
      autonTestStreamDeck5, autonTestStreamDeck6, autonTestStreamDeck7,
      autonTestStreamDeck8, autonTestStreamDeck9, // Vjoy 2
      autonTestStreamDeck10, autonTestStreamDeck11, autonTestStreamDeck12, autonTestStreamDeck13, autonTestStreamDeck14,
      autonTestStreamDeck15;




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    base.setDefaultCommand(driveWithJoysticks);
    logitech = new Joystick(KLogitechPort); // Logitech Dual Action
    xbox = new XboxController(KXboxPort); // Xbox 360 for Windows
    compStreamDeck = new Joystick(KCompStreamDeckPort); // Stream Deck + vjoy
    testStreamDeck = new Joystick(KTestingStreamDeckPort); // Stream Deck + vjoy
    autonTestStreamDeck = new Joystick(KAutonTestingStreamDeckPort); // Stream Deck + vjoy

    // Logitch Buttons
    logitechBtnX = new JoystickButton(logitech, KLogitechButtonX);
    logitechBtnA = new JoystickButton(logitech, KLogitechButtonA);
    logitechBtnB = new JoystickButton(logitech, KLogitechButtonB);
    logitechBtnY = new JoystickButton(logitech, KLogitechButtonY);
    logitechBtnLB = new JoystickButton(logitech, KLogitechLeftBumper);
    logitechBtnRB = new JoystickButton(logitech, KLogitechRightBumper);
    logitechBtnLT = new JoystickButton(logitech, KLogitechLeftTrigger);
    logitechBtnRT = new JoystickButton(logitech, KLogitechRightTrigger);

    // XBox Buttons
    xboxBtnA = new JoystickButton(xbox, KXboxButtonA);
    xboxBtnB = new JoystickButton(xbox, KXboxButtonB);
    xboxBtnX = new JoystickButton(xbox, KXboxButtonX);
    xboxBtnY = new JoystickButton(xbox, KXboxButtonY);
    xboxBtnLB = new JoystickButton(xbox, KXboxLeftBumper);
    xboxBtnRB = new JoystickButton(xbox, KXboxRightBumper);
    xboxBtnSelect = new JoystickButton(xbox, KXboxSelectButton);
    xboxBtnStrt = new JoystickButton(xbox, KXboxStartButton);

    compStreamDeck1 = new JoystickButton(compStreamDeck, 1);
    compStreamDeck2 = new JoystickButton(compStreamDeck, 2);
    compStreamDeck3 = new JoystickButton(compStreamDeck, 3);
    compStreamDeck4 = new JoystickButton(compStreamDeck, 4);
    compStreamDeck5 = new JoystickButton(compStreamDeck, 5);
    compStreamDeck6 = new JoystickButton(compStreamDeck, 6);
    compStreamDeck7 = new JoystickButton(compStreamDeck, 7);
    compStreamDeck8 = new JoystickButton(compStreamDeck, 8);
    compStreamDeck9 = new JoystickButton(compStreamDeck, 9);
    compStreamDeck10 = new JoystickButton(compStreamDeck, 10);
    compStreamDeck11 = new JoystickButton(compStreamDeck, 11);
    compStreamDeck12 = new JoystickButton(compStreamDeck, 12);
    compStreamDeck13 = new JoystickButton(compStreamDeck, 13);
    compStreamDeck14 = new JoystickButton(compStreamDeck, 14);
    compStreamDeck15 = new JoystickButton(compStreamDeck, 15);

    testStreamDeck1 = new JoystickButton(testStreamDeck, 1);
    testStreamDeck2 = new JoystickButton(testStreamDeck, 2);
    testStreamDeck3 = new JoystickButton(testStreamDeck, 3);
    testStreamDeck4 = new JoystickButton(testStreamDeck, 4);
    testStreamDeck5 = new JoystickButton(testStreamDeck, 5);
    testStreamDeck6 = new JoystickButton(testStreamDeck, 6);
    testStreamDeck7 = new JoystickButton(testStreamDeck, 7);
    testStreamDeck8 = new JoystickButton(testStreamDeck, 8);
    testStreamDeck9 = new JoystickButton(testStreamDeck, 9);
    testStreamDeck10 = new JoystickButton(testStreamDeck, 10);
    testStreamDeck11 = new JoystickButton(testStreamDeck, 11);
    testStreamDeck12 = new JoystickButton(testStreamDeck, 12);
    testStreamDeck13 = new JoystickButton(testStreamDeck, 13);
    testStreamDeck14 = new JoystickButton(testStreamDeck, 14);
    testStreamDeck15 = new JoystickButton(testStreamDeck, 15);

    autonTestStreamDeck1 = new JoystickButton(testStreamDeck, 1);
    autonTestStreamDeck2 = new JoystickButton(testStreamDeck, 2);
    autonTestStreamDeck3 = new JoystickButton(testStreamDeck, 3);
    autonTestStreamDeck4 = new JoystickButton(testStreamDeck, 4);
    autonTestStreamDeck5 = new JoystickButton(testStreamDeck, 5);
    autonTestStreamDeck6 = new JoystickButton(testStreamDeck, 6);
    autonTestStreamDeck7 = new JoystickButton(testStreamDeck, 7);
    autonTestStreamDeck8 = new JoystickButton(testStreamDeck, 9);
    autonTestStreamDeck9 = new JoystickButton(testStreamDeck, 9);
    autonTestStreamDeck10 = new JoystickButton(testStreamDeck, 10);
    autonTestStreamDeck11 = new JoystickButton(testStreamDeck, 11);
    autonTestStreamDeck12 = new JoystickButton(testStreamDeck, 12);
    autonTestStreamDeck13 = new JoystickButton(testStreamDeck, 13);
    autonTestStreamDeck14 = new JoystickButton(testStreamDeck, 14);
    autonTestStreamDeck15 = new JoystickButton(testStreamDeck, 15);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public double getLogiRightYAxis() {
    final double Y = logitech.getRawAxis(KRightYAxis);
    SmartDashboard.putNumber("getLogiRightYAxis", -Y);
    if (Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else
      return 0;
  }

  public double getLogiLeftYAxis() {
    final double Y = logitech.getY();
    SmartDashboard.putNumber("getLogiLeftYAxis", -Y);
    if (Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else
      return 0;
  }

  public double getLogiRightXAxis() {
    double X = logitech.getZ();
    SmartDashboard.putNumber("getLogiRightXAxis", -X);
    if (X > KDeadZone || X < -KDeadZone) {
      return -X;
    } else {
      return 0;
    }
  }

  public double getLogiLeftXAxis() {
    double X = logitech.getX();
    SmartDashboard.putNumber("getLogiLeftXAxis", -X);
    if (X > KDeadZone || X < -KDeadZone) {
      return -X;
    } else {
      return 0;
    }
  }

  public double getXboxLeftAxis() {
    final double Y = xbox.getRawAxis(KXboxLeftYAxis);
    if (Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else
      return 0;
  }

  public double getXboxLeftXAxis() {
    final double X = xbox.getRawAxis(KXboxLeftXAxis);
    if (X > KDeadZone || X < -KDeadZone)
      return X;
    else
      return 0;
  }

  public double getXboxRightXAxis() {
    final double X = xbox.getRawAxis(KXboxRightXAxis);
    if (X > KDeadZone || X < -KDeadZone)
      return -X;
    else
      return 0;
  }

  public double getXboxLeftYAxis() {
    final double Y = xbox.getRawAxis(KXboxLeftYAxis);
    if (Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else
      return 0;
  }

  public double getXboxRightYAxis() {
    final double Y = xbox.getRawAxis(KXboxRightYAxis);
    if (Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else
      return 0;
  }

  public boolean joystickThreshold(double triggerValue) {
    if (Math.abs(triggerValue) < .09)
      return false;
    else
      return true;
  }

}
