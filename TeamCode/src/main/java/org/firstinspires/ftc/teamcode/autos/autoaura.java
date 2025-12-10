package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedropathing.constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

// aura.

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue Wall - Full Cycle")
public class autoaura extends LinearOpMode {
    public static Pose start = new Pose(56, 8, Math.toRadians(0));
    public static Pose uno = new Pose(56, 83.5, Math.toRadians(0));

    public static Pose dos = new Pose(13, 83.5, Math.toRadians(0));

    public static Pose tres = new Pose(56, 104, Math.toRadians(0));

    public static Pose cuatro = new Pose(56, 60, Math.toRadians(0));

    public static Pose cinco = new Pose(13, 60, Math.toRadians(0));

    public static Pose seis = new Pose(56, 104, Math.toRadians(0));

    public static Pose siete = new Pose(56, 36, Math.toRadians(0));

    public static Pose ocho = new Pose(13, 36, Math.toRadians(0));

    public static Pose nueve = new Pose(56, 104, Math.toRadians(0));

    public static Pose diez = new Pose(3, 8, Math.toRadians(0));

    public static Pose once = new Pose(56, 104, Math.toRadians(0));







    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private Intake intake;
    private Shooter shooter;



    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);
        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);

        VoltageSensor vs = bot.hMap.voltageSensor.iterator().next();

        Follower f = constants.createFollower(bot.hMap);
        f.setStartingPose(start);
        f.update();

        Intake a = new Intake(bot);
        a.register();
        Shooter d = new Shooter(bot);
        d.register();





        SequentialCommandGroup wallAuto =
                new SequentialCommandGroup(
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(new BezierLine(start, uno))
                                .setLinearHeadingInterpolation(start.getHeading(), uno.getHeading())
                                .build()
                        ),
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(uno, dos))
                                        .setLinearHeadingInterpolation(uno.getHeading(), dos.getHeading())
                                        .build()
                                ),
                                new SequentialCommandGroup(
                                        new FollowPathCommand(f, f.pathBuilder()
                                                .addPath(new BezierLine(dos, tres))
                                                .setLinearHeadingInterpolation(dos.getHeading(), tres.getHeading())
                                                .build()
                                        ),
                                        new SequentialCommandGroup(
                                                new FollowPathCommand(f, f.pathBuilder()
                                                        .addPath(new BezierLine(tres, cuatro))
                                                        .setLinearHeadingInterpolation(tres.getHeading(), cuatro.getHeading())
                                                        .build()
                                                ),
                                                new SequentialCommandGroup(
                                                        new FollowPathCommand(f, f.pathBuilder()
                                                                .addPath(new BezierLine(cuatro, cinco))
                                                                .setLinearHeadingInterpolation(cuatro.getHeading(), cinco.getHeading())
                                                                .build()
                                                        ),
                                                        new SequentialCommandGroup(
                                                                new FollowPathCommand(f, f.pathBuilder()
                                                                        .addPath(new BezierLine(cinco, seis))
                                                                        .setLinearHeadingInterpolation(cinco.getHeading(), seis.getHeading())
                                                                        .build()
                                                                ),
                                                                new SequentialCommandGroup(
                                                                        new FollowPathCommand(f, f.pathBuilder()
                                                                                .addPath(new BezierLine(seis, siete))
                                                                                .setLinearHeadingInterpolation(seis.getHeading(), siete.getHeading())
                                                                                .build()
                                                                        ),
                                                                        new SequentialCommandGroup(
                                                                                new FollowPathCommand(f, f.pathBuilder()
                                                                                        .addPath(new BezierLine(siete, ocho))
                                                                                        .setLinearHeadingInterpolation(siete.getHeading(), ocho.getHeading())
                                                                                        .build()
                                                                                ),
                                                                                new SequentialCommandGroup(
                                                                                        new FollowPathCommand(f, f.pathBuilder()
                                                                                                .addPath(new BezierLine(ocho, nueve))
                                                                                                .setLinearHeadingInterpolation(ocho.getHeading(), nueve.getHeading())
                                                                                                .build()
                                                                                        ),
                                                                                        new SequentialCommandGroup(
                                                                                                new FollowPathCommand(f, f.pathBuilder()
                                                                                                        .addPath(new BezierLine(nueve, diez))
                                                                                                        .setLinearHeadingInterpolation(nueve.getHeading(), diez.getHeading())
                                                                                                        .build()
                                                                                                ),
                                                                                                new SequentialCommandGroup(
                                                                                                        new FollowPathCommand(f, f.pathBuilder()
                                                                                                                .addPath(new BezierLine(diez, once))
                                                                                                                .setLinearHeadingInterpolation(diez.getHeading(), once.getHeading())
                                                                                                                .build()
                                                                                                        )


                                                                                                )))))))))));



        waitForStart();

        f.setMaxPower(10.0 / vs.getVoltage());
        CommandScheduler.getInstance().schedule(wallAuto);

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            f.update();
            telem.addData("current pose", f.getPose());

            telem.addData("Follower Status", f.isBusy() ? "Running Path" : "Finished");
            telem.update();
        }
        CommandScheduler.getInstance().reset();
    }
}
