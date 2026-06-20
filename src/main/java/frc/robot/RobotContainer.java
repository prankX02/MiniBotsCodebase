// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.drive.TeleopDrive;
import frc.robot.subsystems.drive.Drive;

public class RobotContainer {

    // CAN IDs — match these to what you set in REV Hardware Client
    private static final int FL_ID = 1;
    private static final int FR_ID = 2;
    private static final int RL_ID = 3;
    private static final int RR_ID = 4;

    private final CommandXboxController gamepad = new CommandXboxController(0);

    private Drive drivebase;

    public RobotContainer() {
        buildRobot();
        configureBindings();
    }

    private void buildRobot() {
        if (Constants.getMode() != Mode.REPLAY) {
            switch (Constants.getRobot()) {
                case COMPETITION:
                    buildComp();
                    break;
                case SIMBOT:
                    // No sim support yet for mecanum, falls through to default
                    break;
            }
        }

        // Fallback — if drivebase wasn't built (e.g. sim or replay), 
        // create a do-nothing stub so nothing null-crashes
        if (drivebase == null) {
            drivebase = new Drive(FL_ID, FR_ID, RL_ID, RR_ID);
        }
    }

    private void buildComp() {
        drivebase = new Drive(FL_ID, FR_ID, RL_ID, RR_ID);
    }

    private void configureBindings() {
        // Default command — runs whenever no other command is using the drivebase
        drivebase.setDefaultCommand(new TeleopDrive(
            drivebase,
            () -> -gamepad.getLeftY(),   // forward/back (negated so push forward = positive)
            () -> -gamepad.getLeftX(),   // strafe
            () -> -gamepad.getRightX()   // rotation
        ));

        // Brake / stop while A button held
        gamepad.a().whileTrue(Commands.runOnce(drivebase::stop, drivebase));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}