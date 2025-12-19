


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
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Vision.Limelight;
import org.firstinspires.ftc.teamcode.pedropathing.constants;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Intake;


import java.io.File;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class teleop extends CommandOpMode {
    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private MecanumDrive MecanumDrive;
    private Shooter Shooter;
    private Limelight Limelight;
    private Intake Intake;
    Indexer i;
    Follower f;

    Pose start = new Pose(0 ,0,0);


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


        Limelight = new Limelight(bot);
        Limelight.register();

        Intake = new Intake(bot);
        Intake.register();

        Shooter = new Shooter(bot);
        Shooter.register();

        i = new Indexer(hardwareMap, gamepad1);
        i.register();

        f = constants.createFollower(hardwareMap);
        f.setStartingPose(start);
        f.update();
        f.startTeleopDrive(true);




        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whileHeld(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> Shooter.setVelocity(1800))
                        )
                );

        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenReleased(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> Shooter.setVelocity(0))
                        )
                );
        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whileHeld(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> Intake.setVelocity(0.7))
                        )
                );

        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenReleased(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> Intake.setVelocity(0))
                        )
                );



        while (opModeInInit()) {
            telem.update();
        }
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        if (driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.3){
            Shooter.setVelocity(6000);
        }
           else{Shooter.setVelocity((0));}
        if (driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.3){
            Intake.setVelocity(0.7);
        }
        else{Intake.setVelocity((0));}
        f.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x,-gamepad1.right_stick_x , false);
        f.update();
        telem.update();
    }
}
