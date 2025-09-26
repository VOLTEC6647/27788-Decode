package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.utils.RobotConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPivot;
import org.firstinspires.ftc.teamcode.subsystems.ClawPivot;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;


@Autonomous
public class SpecimenAuto extends LinearOpMode {

    //region Poses
    public static Pose startingPose = new Pose(10, 71, Math.toRadians(180));
    public static Pose score1 = new Pose(38.5, 71, Math.toRadians(180));
    public static Pose pushIntermediate = new Pose(15, 36, Math.toRadians(180));
    public static Pose pushIntermediate1 = new Pose(61,36.25,Math.toRadians(180));
    public static Pose pushSample1 = new Pose(59,26,Math.toRadians(180));
    public static Pose pushSample2Intermediate1 = new Pose(23.231,16.146,Math.toRadians(180));
    public static Pose pushSample2Intermediate2 = new Pose(56,16,Math.toRadians(180));
    public static Pose pushSample2 = new Pose(55,5.6,Math.toRadians(180));
    public static Pose pushSample3Intermediate = new Pose(23.72,5.45,Math.toRadians(180));
    public static Pose pushSample3 = new Pose(6.5, 35, Math.toRadians(180));
    public static Pose endPose = new Pose(38, 74.25, Math.toRadians(180));

    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private MecanumDrive drive;

    private ShooterPivot claw;
    private ClawPivot clawPivot;
    private Arm arm;



    @Override
    public void runOpMode() {

        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        Follower f = new Follower(hardwareMap, FConstants.class, LConstants.class);

        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);


        f.setPose(startingPose);
        f.setMaxPower(0.75);



        claw = new ShooterPivot(bot);
        claw.register();

        clawPivot = new ClawPivot(bot);
        clawPivot.register();

        arm = new Arm(bot);
        arm.register();



        f.setPose(startingPose);


        //endregion

        SequentialCommandGroup auto = new SequentialCommandGroup(
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        new InstantCommand(()-> arm.setPosition(armScore)),
                        new WaitCommand(600)),
                //region Intermediate
                new ParallelCommandGroup(
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Point(startingPose),
                                                new Point(score1)
                                        )
                                )
                                .setLinearHeadingInterpolation(startingPose.getHeading(), score1.getHeading())
                                .build()
                        )

                ),
                new SequentialCommandGroup(
                        new InstantCommand(()-> arm.setPosition(armAfterScore)),
                        new WaitCommand(800)

                ),

                new SequentialCommandGroup(
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Point(score1),
                                                new Point(pushIntermediate)
                                        )
                                )
                                .setLinearHeadingInterpolation(
                                        score1.getHeading(), pushIntermediate.getHeading())
                                .build()
                        ),

                        new SequentialCommandGroup(
                                new InstantCommand(()-> arm.setPosition(armGrabWall)),
                                new WaitCommand(50) ),
                        new ParallelCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        new Point(pushIntermediate),
                                                        new Point(pushIntermediate1)
                                                )
                                        )
                                        .setLinearHeadingInterpolation(pushIntermediate.getHeading(), pushIntermediate1.getHeading())
                                        .build()
                                )

                        ),
                        new ParallelCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        new Point(pushIntermediate1),
                                                        new Point(pushSample1)
                                                )
                                        )
                                        .setLinearHeadingInterpolation(pushIntermediate1.getHeading(), pushSample1.getHeading())
                                        .build()
                                )

                        ),
                        new ParallelCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        new Point(pushSample1),
                                                        new Point(pushSample2Intermediate1)
                                                )
                                        )
                                        .setLinearHeadingInterpolation(pushSample1.getHeading(), pushSample2Intermediate1.getHeading())
                                        .build()
                                )

                        ),
                        new ParallelCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        new Point(pushSample2Intermediate1),
                                                        new Point(pushSample2Intermediate2)
                                                )
                                        )
                                        .setLinearHeadingInterpolation(pushSample2Intermediate1.getHeading(), pushSample2Intermediate2.getHeading())
                                        .build()
                                )

                        ),
                        new ParallelCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        new Point(pushSample2Intermediate2),
                                                        new Point(pushSample2)
                                                )
                                        )
                                        .setLinearHeadingInterpolation(pushSample2Intermediate2.getHeading(), pushSample2.getHeading())
                                        .build()
                                )

                        ),
                        new ParallelCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        new Point(pushSample2),
                                                        new Point(pushSample2Intermediate2)
                                                )
                                        )
                                        .setLinearHeadingInterpolation(pushSample2.getHeading(), pushSample2Intermediate2.getHeading())
                                        .build()
                                )

                        ),
                        new ParallelCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        new Point(pushSample2Intermediate2),
                                                        new Point(pushSample3Intermediate)
                                                )
                                        )
                                        .setLinearHeadingInterpolation(pushSample2Intermediate2.getHeading(), pushSample3Intermediate.getHeading())
                                        .build()
                                )

                        ),
                        new ParallelCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        new Point(pushSample3Intermediate),
                                                        new Point(pushSample3)
                                                )
                                        )
                                        .setLinearHeadingInterpolation(pushSample3Intermediate.getHeading(), pushSample3.getHeading())
                                        .build()
                                )

                        ),
                        new ParallelCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        new Point(pushSample3),
                                                        new Point(endPose)
                                                )
                                        )
                                        .setLinearHeadingInterpolation(pushSample3.getHeading(), endPose.getHeading())
                                        .build()
                                )

                        )



                )
        );

        waitForStart();
        CommandScheduler.getInstance().schedule(auto);

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            f.update();
            try {
                f.telemetryDebug(telem);
            } catch (Exception e) {

            }

        }
    }

}