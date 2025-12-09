package  org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup; // Import ParallelRaceGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand; // Import WaitCommand
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


@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Paths extends LinearOpMode {

    // Scoring Poses
    public static Pose score = new Pose(55, 85, Math.toRadians(315));
    public static Pose wallStart = new Pose(55, 9.5, Math.toRadians(270));

    public static Pose preGrab1  = new Pose(55, 60, Math.toRadians(180));
    public static Pose grab1  = new Pose(25, 60, Math.toRadians(180));
    public static Pose preGrab2  = new Pose(55, 85, Math.toRadians(180));
    public static Pose grab2  = new Pose(25, 85, Math.toRadians(180));
    public static Pose preGrab3  = new Pose(70, 10, Math.toRadians(0));
    public static Pose grab3  = new Pose(134, 10, Math.toRadians(0));
    public static Pose postGrab3 = new Pose(70,20,Math.toRadians(315));
    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private Intake intake;


    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);
        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);

        VoltageSensor vs = bot.hMap.voltageSensor.iterator().next();

        Follower f = constants.createFollower(bot.hMap);
        f.setStartingPose(wallStart);
        f.update();

        intake = new Intake(bot);
        intake.register();


        SequentialCommandGroup auto =
                new SequentialCommandGroup(
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(new BezierLine(wallStart, score))
                                .setLinearHeadingInterpolation(wallStart.getHeading(), score.getHeading())
                                .build()
                        ),
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(new BezierLine(score, preGrab1))
                                .setLinearHeadingInterpolation(score.getHeading(),preGrab1.getHeading())
                                .build()
                        ),
                        new ParallelRaceGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(preGrab1, grab1))
                                        .setLinearHeadingInterpolation(preGrab1.getHeading(),grab1.getHeading())
                                        .build()
                                )//,
                                //new RunCommand(() -> intake.setPower(1.0), intake)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(200)//,
                                //new InstantCommand(()-> intake.setPower(0), intake)
                        ),
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(new BezierLine(grab1, score))
                                .setLinearHeadingInterpolation(grab1.getHeading(),score.getHeading())
                                .build()
                        ),
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(new BezierLine(score, preGrab2))
                                .setLinearHeadingInterpolation(score.getHeading(),preGrab2.getHeading())
                                .build()
                        ),
                        new ParallelRaceGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(preGrab2, grab2))
                                        .setLinearHeadingInterpolation(preGrab2.getHeading(),grab2.getHeading())
                                        .build()
                                )//,
                                //new RunCommand(() -> intake.setPower(1.0), intake)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new InstantCommand(()-> intake.setVelocity(), intake)
                        ),
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(new BezierLine(grab2, preGrab3))
                                .setLinearHeadingInterpolation(grab2.getHeading(),preGrab3.getHeading())
                                .build()
                        ),
                        new ParallelRaceGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(preGrab3, grab3))
                                        .setLinearHeadingInterpolation(preGrab3.getHeading(),grab3.getHeading())
                                        .build()
                                )//,
                                //new RunCommand(() -> intake.setPower(1.0), intake)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(200)//,
                                //new InstantCommand(()-> intake.setPower(0), intake)
                        ),
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(new BezierLine(grab3, postGrab3))
                                .setLinearHeadingInterpolation(grab3.getHeading(),postGrab3.getHeading())
                                .build()
                        ),
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(new BezierLine(postGrab3, score))
                                .setLinearHeadingInterpolation(postGrab3.getHeading(),score.getHeading())
                                .build()
                        )
                );

        waitForStart();

        f.setMaxPower(10.0 / vs.getVoltage());
        CommandScheduler.getInstance().schedule(auto);

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