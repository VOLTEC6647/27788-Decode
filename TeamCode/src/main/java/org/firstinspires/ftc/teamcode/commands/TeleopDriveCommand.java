package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends CommandBase {

    private final MecanumDrive drivetrain;
    private final DoubleSupplier leftStickX, leftStickY, rot, multiplier;

    public TeleopDriveCommand(MecanumDrive d, DoubleSupplier leftStickX,
                              DoubleSupplier leftStickY,
                              DoubleSupplier rot,
                              DoubleSupplier multiplier) {

        this.drivetrain = d;
        this.leftStickX = leftStickX;
        this.leftStickY = leftStickY;
        this.rot = rot;
        this.multiplier = multiplier;

        addRequirements(this.drivetrain);
    }

    public static void setDefaultCommand(TeleopDriveCommand driveCommand) {
    }

    @Override
    public void execute() {
        drivetrain.teleopDrive(
                multiplier.getAsDouble()
        );
    }
}