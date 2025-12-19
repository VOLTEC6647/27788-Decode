package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends CommandBase {

    private final MecanumDrive drivetrain;
    private final DoubleSupplier leftStickX, leftStickY, rot, multiplier;

    public TeleopDriveCommand(MecanumDrive d,
                              DoubleSupplier leftStickX,
                              DoubleSupplier leftStickY,
                              DoubleSupplier rot,
                              DoubleSupplier multiplier) {
        this.drivetrain = d;
        this.leftStickX = leftStickX;
        this.leftStickY = leftStickY;
        this.rot = rot;
        this.multiplier = multiplier;

        // Tells the scheduler that this command uses the drivetrain
        addRequirements(this.drivetrain);
    }

    @Override
    public void execute() {
        // Line 29: Ensure drivetrain.teleopDrive exists and is public
        drivetrain.teleopDrive(
                leftStickX.getAsDouble(),
                leftStickY.getAsDouble(),
                rot.getAsDouble(),
                multiplier.getAsDouble()
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Line 40: Stop motors when command ends
        drivetrain.teleopDrive(0.0, 0.0, 0.0, 0.0);
    }
}