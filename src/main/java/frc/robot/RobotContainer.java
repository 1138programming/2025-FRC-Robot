// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;


import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.EndTelemetry;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static frc.robot.Constants.TunerConstants.*;
import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.generated.TunerSwerve.*;

public class RobotContainer {

    private double MaxSpeed = kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public static Joystick logitech;
    public static Joystick compStreamDeck;
    public static Joystick testStreamDeck;
    public static Joystick autonTestStreamDeck;
    // public final CommandXboxController joystick = new CommandXboxController(1);
    public JoystickButton logitechBtnX, logitechBtnA, logitechBtnB, logitechBtnY, logitechBtnLB, logitechBtnRB,
            logitechBtnLT, logitechBtnRT, logitechBtnBack, logitechBtnStart; // Logitech Button

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
            autonTestStreamDeck10, autonTestStreamDeck11, autonTestStreamDeck12, autonTestStreamDeck13,
            autonTestStreamDeck14,
            autonTestStreamDeck15;

    public final CommandSwerveDrivetrain drivetrain = createDrivetrain();
    public final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(drivetrain);
    public final EndTelemetry endTelemetry;
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // boolean isCompetition = false;
        endTelemetry = new EndTelemetry(logger);
        logitech = new Joystick(KLogitechPort); // Logitech Dual Action
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
        logitechBtnBack = new JoystickButton(logitech, 9);
        logitechBtnStart = new JoystickButton(logitech, 10);

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
        configureBindings();

        
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> Kdrive.withVelocityX(-getLogiLeftYAxis() * KMaxSpeed)
                .withVelocityY(-getLogiLeftXAxis() * KMaxSpeed)
                .withRotationalRate(getLogiRightXAxis() * KMaxAngularRate)));

        logitechBtnA.whileTrue(drivetrain.applyRequest(() -> brake));
        // logitechBtnB.whileTrue(drivetrain.applyRequest(
        // () -> point.withModuleDirection(new Rotation2d(-getLogiLeftYAxis(),
        // -getLogiLeftXAxis()))));

        // Run SysId routines when holding back/st^art and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // Run in desending order (Hold for alteast 5 seconds)
        testStreamDeck1.whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        testStreamDeck2.whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        testStreamDeck3.whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        testStreamDeck4.whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        testStreamDeck5.onTrue(endTelemetry);

        // reset the field-centric heading on left bumper press
        logitechBtnX.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        // autoChooser.getSelected();
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
}