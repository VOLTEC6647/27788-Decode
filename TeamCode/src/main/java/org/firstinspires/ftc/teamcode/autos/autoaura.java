

package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.constants;




@Config
@Autonomous
public class autoaura extends LinearOpMode {

    // Scoring Poses
    public static Pose startingPose = new Pose(56.000, 8.000, Math.toRadians(90));
    public static Pose second = new Pose(57.5, 85, Math.toRadians(0));
    public static Pose third = new Pose(8.5, 84, Math.toRadians(125));
    public static Pose four = new Pose(57, 85, Math.toRadians(0));
    public static Pose five = new Pose(56.5, 60, Math.toRadians(0));
    public static Pose six = new Pose(7, 59.5, Math.toRadians(0));
    public static Pose seven = new Pose(56.5, 60.5, Math.toRadians(125));
    public static Pose eigth = new Pose(57.5, 85.5, Math.toRadians(0));
    public static Pose nine = new Pose(57, 34.5, Math.toRadians(0));
    public static Pose ten = new Pose(6, 34.5, Math.toRadians(0));
    public static Pose eleven = new Pose(56.5, 35, Math.toRadians(125));
    public static Pose twelve = new Pose(57.5, 85.5, Math.toRadians(0));
    public static Pose score = new Pose(55, 85, Math.toRadians(315));







    private Follower follower;
    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;


    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();


        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);

        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        VoltageSensor vs = bot.hMap.voltageSensor.iterator().next();

        Follower f = constants.createFollower(bot.hMap);
        f.setStartingPose(startingPose);
        f.update();



        SequentialCommandGroup auto = new SequentialCommandGroup(
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        startingPose,
                                        second
                                )
                        )
                        .setLinearHeadingInterpolation(startingPose.getHeading(),second.getHeading())
                        .build()
                ),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        second,
                                        third

                                )
                        )
                        .setLinearHeadingInterpolation(second.getHeading(),third.getHeading())
                        .build()
                ),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        third,
                                        four

                                )
                        )
                        .setLinearHeadingInterpolation(third.getHeading(),four.getHeading())
                        .build()
                ),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        four,
                                        five

                                )
                        )
                        .setLinearHeadingInterpolation(four.getHeading(),five.getHeading())
                        .build()
                ),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        five,
                                        six

                                )
                        )
                        .setLinearHeadingInterpolation(five.getHeading(),six.getHeading())
                        .build()
                ),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        six,
                                        seven

                                )
                        )
                        .setLinearHeadingInterpolation(six.getHeading(),seven.getHeading())
                        .build()
                ),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        seven,
                                        eigth

                                )
                        )
                        .setLinearHeadingInterpolation(seven.getHeading(),eigth.getHeading())
                        .build()
                ),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        eigth,
                                        nine

                                )
                        )
                        .setLinearHeadingInterpolation(eigth.getHeading(),nine.getHeading())
                        .build()
                ),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        nine,
                                        ten

                                )
                        )
                        .setLinearHeadingInterpolation(nine.getHeading(),ten.getHeading())
                        .build()
                ),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        ten,
                                        eleven

                                )
                        )
                        .setLinearHeadingInterpolation(ten.getHeading(),eleven.getHeading())
                        .build()
                ),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        eleven,
                                        twelve

                                )
                        )
                        .setLinearHeadingInterpolation(eleven.getHeading(),twelve.getHeading())
                        .build()
                )
        );



        waitForStart();
        CommandScheduler.getInstance().schedule(auto);

        while (opModeIsActive()) {
            f.setMaxPower(10.0 / vs.getVoltage());
            CommandScheduler.getInstance().run();
            f.update();

        }
    }
}
