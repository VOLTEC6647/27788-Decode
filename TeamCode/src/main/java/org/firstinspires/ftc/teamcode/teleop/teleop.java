package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Vision.Limelight;
import org.firstinspires.ftc.teamcode.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Intake;


import java.io.File;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class teleop extends CommandOpMode {
    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private MecanumDrive drive;
    private Shooter Shooter;
    private Limelight Limelight;
    private Intake Intake;


    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);

        File myFileName = AppUtil.getInstance().getSettingsFile("team.txt");
        String team = ReadWriteFile.readFile(myFileName);

        if (team.equals("blue")) {
            bot.setRotationOffset(Rotation2d.fromDegrees(0));
        }
        if (team.equals("red")) {
            bot.setRotationOffset(Rotation2d.fromDegrees(0));
        }

        Shooter = new Shooter(bot);
        Shooter.register();

        drive = new MecanumDrive(bot);
        drive.register();

        Intake = new Intake(bot);
        Intake.register();

        Limelight = new Limelight(bot);
        Limelight.register();





        TeleopDriveCommand driveCommand = new TeleopDriveCommand(
                drive,
                () -> -driverGamepad.getRightX(),
                () -> -driverGamepad.getLeftY(),
                () -> driverGamepad.getLeftX(),
                () -> bot.speed
        );
        bot.speed = 0.75;

        register(drive);
        drive.setDefaultCommand(driveCommand);


        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whileHeld(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> Shooter.setVelocity())
                        )
                );

        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenReleased(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> Shooter.setVelocity())
                        )
                );
        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whileHeld(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> Intake.setVelocity())
                        )
                );

        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenReleased(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> Intake.setVelocity( ))
                        )
                );



        while (opModeInInit()) {
            telem.update();
        }
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telem.update();
    }
}
