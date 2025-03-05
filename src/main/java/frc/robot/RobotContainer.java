// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.Util;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.CommandGroups.LiftSetpoints.LiftandArmIntake;
import frc.robot.CommandGroups.LiftSetpoints.LiftandArmTier1;
import frc.robot.CommandGroups.LiftSetpoints.LiftandArmTier2;
import frc.robot.CommandGroups.LiftSetpoints.LiftandArmTier3;
import frc.robot.CommandGroups.LiftSetpoints.LiftandArmTier4;
import frc.robot.commands.Arm.MoveArmStow;
import frc.robot.commands.Arm.SetArmManualMode;
import frc.robot.commands.Arm.TiltArmManually;
import frc.robot.commands.Arm.TiltArmToSetPosition;
import frc.robot.commands.Base.BaseSpeed;
import frc.robot.commands.Base.DriveWithJoysticks;
import frc.robot.commands.Coral.CoralDefault;
import frc.robot.commands.Coral.SpinCoralIntake;
import frc.robot.commands.Lift.MoveLift;
import frc.robot.commands.Lift.MoveLiftToPos;
import frc.robot.commands.Lift.SetLiftManualMode;
import frc.robot.commands.Telemetry.EndTelemetry;
import frc.robot.commands.Telemetry.StartTelemetry;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Lift;
import frc.robot.util.Telemetry;
import com.pathplanner.lib.auto.NamedCommands;

import static frc.robot.Constants.TunerConstants.*;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ArmConstants.ArmPositionConstants.*;
import static frc.robot.Constants.LiftConstants.*;
import static frc.robot.Constants.LiftConstants.LiftPositionConstants.*;
import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.generated.TunerSwerve.*;
import static frc.robot.Constants.CoralIntakeConstants.*;

public class RobotContainer {

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain;
    public final Arm arm;
    public final Lift lift;
    public final CoralIntake coralIntake;
    public final SubsystemUtil subsystemUtil;

    // Commands
    public final DriveWithJoysticks driveWithJoysticks;
    public final EndTelemetry endTelemetry;
    public final StartTelemetry startTelemetry;
    public final BaseSpeed baseTurboMode;
    public final BaseSpeed baseNormalMode;
    public final BaseSpeed baseSlowMode;

    public final MoveLift liftStop;
    public final MoveLift moveLiftUp;
    public final MoveLift moveLiftDown;
    public final MoveLiftToPos liftStow;
    public final SetLiftManualMode setLiftManualMode;

    public final TiltArmManually armStop;
    public final TiltArmManually tiltArmManuallyUp;
    public final TiltArmManually tiltArmManuallyDown;
    public final MoveArmStow armStow;
    public final TiltArmToSetPosition armtopos;
    public final SetArmManualMode setArmManualMode;

    public final SpinCoralIntake stopCoralIntake;
    public final CoralDefault coralDefault;
    public final SpinCoralIntake spinCoralIntakeForward;
    public final SpinCoralIntake spinCoralIntakeBackward;

    // Command Groups
    public final LiftandArmTier4 liftandArmTier4;
    public final LiftandArmTier3 liftandArmTier3;
    public final LiftandArmTier2 liftandArmTier2;
    public final LiftandArmTier1 liftandArmTier1;
    public final LiftandArmIntake liftandArmIntake;
    private final SendableChooser<Command> autoChooser;

    /* Setting up bindings for necessary control of the swerve drive platform */
    // private final SwerveRequest.SwerveDriveBrake brake = new
    // SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new
    // SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(KMaxSpeed);
    public static Joystick logitech;
    public static Joystick compStreamDeck;
    public static Joystick testStreamDeck;
    public static Joystick autonTestStreamDeck;
    // public final CommandXboxController joystick = new CommandXboxController(1);
    public JoystickButton logitechBtnX, logitechBtnA, logitechBtnB, logitechBtnY, logitechBtnLB, logitechBtnRB,
            logitechBtnLT, logitechBtnRT, logitechBtnBack, logitechBtnStart; // Logitech Button

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

    public RobotContainer() {

        // Subsystems
        drivetrain = createDrivetrain();
        arm = new Arm();
        lift = new Lift();
        coralIntake = new CoralIntake();
        subsystemUtil = new SubsystemUtil(arm);

        // Commands
        driveWithJoysticks = new DriveWithJoysticks(drivetrain);

        baseTurboMode = new BaseSpeed(subsystemUtil, KBaseTurboMode);
        baseNormalMode = new BaseSpeed(subsystemUtil, KBaseNormalMode);
        baseSlowMode = new BaseSpeed(subsystemUtil, KBaseSlowMode);

        startTelemetry = new StartTelemetry(logger);
        endTelemetry = new EndTelemetry(logger);

        liftStop = new MoveLift(lift, KLiftStopVelocity);
        moveLiftUp = new MoveLift(lift, KLiftMoveVelocity);
        moveLiftDown = new MoveLift(lift, -KLiftMoveVelocity);
        liftStow = new MoveLiftToPos(lift, KLiftPositionStow);
        setLiftManualMode = new SetLiftManualMode(lift);

        armStop = new TiltArmManually(arm, KArmStopVelocity);
        tiltArmManuallyUp = new TiltArmManually(arm, KArmMoveVelocity);
        tiltArmManuallyDown = new TiltArmManually(arm, -KArmMoveVelocity);
        armtopos = new TiltArmToSetPosition(arm, 50);
        setArmManualMode = new SetArmManualMode(arm);
        armStow = new MoveArmStow(arm, KArmPositionStow);

        stopCoralIntake = new SpinCoralIntake(coralIntake, 0);
        coralDefault = new CoralDefault(coralIntake, subsystemUtil);

        spinCoralIntakeForward = new SpinCoralIntake(coralIntake, KCoralIntakeSpeed);
        spinCoralIntakeBackward = new SpinCoralIntake(coralIntake, -KCoralIntakeSpeed);

        // Command Groups
        liftandArmTier4 = new LiftandArmTier4(arm, lift);
        liftandArmTier3 = new LiftandArmTier3(arm, lift);
        liftandArmTier2 = new LiftandArmTier2(arm, lift);
        liftandArmTier1 = new LiftandArmTier1(arm, lift);
        liftandArmIntake = new LiftandArmIntake(arm, lift);

        //Named Commands (for Pathplanner)
        NamedCommands.registerCommand("liftandArmTier4", liftandArmTier4);
        NamedCommands.registerCommand("liftandArmTier3", liftandArmTier3);
        NamedCommands.registerCommand("liftandArmTier2", liftandArmTier2);
        NamedCommands.registerCommand("liftandArmTier1", liftandArmTier1);
        NamedCommands.registerCommand("spinCoralIntakeBackward", spinCoralIntakeBackward);

        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                // Swerve 1
                builder.addDoubleProperty("Front Left Angle",
                        () -> drivetrain.getModule(0).getCurrentState().angle.getDegrees(),
                        null);
                builder.addDoubleProperty("Front Left Velocity",
                        () -> drivetrain.getModule(0).getCurrentState().speedMetersPerSecond,
                        null);

                // Swerve 2
                builder.addDoubleProperty("Front Right Angle",
                        () -> drivetrain.getModule(1).getCurrentState().angle.getDegrees(),
                        null);
                builder.addDoubleProperty("Front Right Velocity",
                        () -> drivetrain.getModule(1).getCurrentState().speedMetersPerSecond,
                        null);

                // Swerve 3
                builder.addDoubleProperty("Back Left Angle",
                        () -> drivetrain.getModule(2).getCurrentState().angle.getDegrees(),
                        null);
                builder.addDoubleProperty("Back Left Velocity",
                        () -> drivetrain.getModule(2).getCurrentState().speedMetersPerSecond,
                        null);

                // Swerve 4
                builder.addDoubleProperty("Back Right Angle",
                        () -> drivetrain.getModule(3).getCurrentState().angle.getDegrees(),
                        null);
                builder.addDoubleProperty("Back Right Velocity",
                        () -> drivetrain.getModule(3).getCurrentState().speedMetersPerSecond,
                        null);

                // Rotation
                builder.addDoubleProperty("Robot Angle", () -> drivetrain.getRotation3d().toRotation2d().getDegrees(),
                        null);

            }
        });

        // Auto Chooser For Shuffleboard
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        

        // DS Ports
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
        logitechBtnBack = new JoystickButton(logitech, KLogitechBtnBack);
        logitechBtnStart = new JoystickButton(logitech, KLogitechRightStart);

        // Streamdeck Pages used in match
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

        // Streamdeck Pages used for testing
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
    }

    private void configureBindings() {

        // Default Commands
        drivetrain.setDefaultCommand(drivetrain.applyRequest(
                () -> Kdrive.withVelocityX(-getLogiLeftYAxis() * KMaxSpeed * subsystemUtil.getSwerveMaxSpeed())
                        .withVelocityY(-getLogiLeftXAxis() * KMaxSpeed * subsystemUtil.getSwerveMaxSpeed())
                        .withRotationalRate(
                                -getLogiRightXAxis() * KMaxAngularRate * subsystemUtil.getSwerveMaxSpeed())));
        arm.setDefaultCommand(armStow);
        lift.setDefaultCommand(liftStow);
        // coralIntake.setDefaultCommand(armStow);
        coralIntake.setDefaultCommand(coralDefault); // could be an issue
        // Logitech Controller:

        // Reset the field-centric heading on y button press
        logitechBtnY.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        logitechBtnRB.whileTrue(spinCoralIntakeForward);
        logitechBtnRT.whileTrue(spinCoralIntakeBackward);

        logitechBtnLB.whileTrue(baseTurboMode);
        logitechBtnLT.whileTrue(baseSlowMode);

        // When either button is released, it sets it equal to normal mode
        logitechBtnLB.and(logitechBtnLT.whileFalse(baseNormalMode));

        // Comp Stream Deck:

        // Lift and Arm Setpoints
        compStreamDeck2.whileTrue(liftandArmIntake);
        compStreamDeck1.whileTrue(liftandArmTier3);
        compStreamDeck6.whileTrue(liftandArmTier2);
        compStreamDeck7.whileTrue(liftandArmTier4);
        compStreamDeck11.whileTrue(liftandArmTier1);

        // Coral Intake
        compStreamDeck5.whileTrue(spinCoralIntakeForward);
        compStreamDeck10.whileTrue(spinCoralIntakeBackward);

        // Manual Modes
        compStreamDeck13.onTrue(setArmManualMode);
        compStreamDeck14.onTrue(setLiftManualMode);

        compStreamDeck3.whileTrue(tiltArmManuallyUp);
        compStreamDeck8.whileTrue(tiltArmManuallyDown);
        compStreamDeck4.whileTrue(moveLiftUp);
        compStreamDeck9.whileTrue(moveLiftDown);

        // Test Stream Deck:

        // Base Sysid
        testStreamDeck1.whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        testStreamDeck2.whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        testStreamDeck3.whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        testStreamDeck4.whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        // logger
        testStreamDeck5.onTrue(endTelemetry);
        // testStreamDeck10.onTrue(armtopos);

        testStreamDeck6.whileTrue(lift.sysIdQuasistaticLift(Direction.kForward));
        testStreamDeck7.whileTrue(lift.sysIdQuasistaticLift(Direction.kReverse));
        testStreamDeck8.whileTrue(lift.sysIdDynamicLift(Direction.kForward));
        testStreamDeck9.whileTrue(lift.sysIdDynamicLift(Direction.kReverse));

        // Coral Intake Test Controls
        testStreamDeck11.whileTrue(spinCoralIntakeForward);
        testStreamDeck12.whileTrue(spinCoralIntakeBackward);

        // Arm Test Controls
        testStreamDeck13.whileTrue(tiltArmManuallyUp);
        testStreamDeck14.whileTrue(tiltArmManuallyDown);

        // Starts Telemetry
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