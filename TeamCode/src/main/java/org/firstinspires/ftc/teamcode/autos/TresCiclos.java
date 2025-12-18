package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
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
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import static org.firstinspires.ftc.teamcode.utils.RobotConstants.*;

// aura .

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue Wall - Full Cycle")
public class TresCiclos extends LinearOpMode {
    public static Pose start = new Pose(56, 135, Math.toRadians(90));
    public static Pose Shoot0 = new Pose(56, 90, Math.toRadians(125));

    public static Pose A0 = new Pose(56, 84, Math.toRadians(180));

    public static Pose I0 = new Pose(15, 84, Math.toRadians(180));

    public static Pose Shoot1 = new Pose(56, 90, Math.toRadians(125));

    public static Pose A1 = new Pose(56, 60, Math.toRadians(180));

    public static Pose I1 = new Pose(10, 60, Math.toRadians(180));

    public static Pose Shoot2 = new Pose(56, 90, Math.toRadians(125));
    public static Pose A2 = new Pose(56, 36, Math.toRadians(180));
    public static Pose I2 = new Pose(10, 36, Math.toRadians(180));
    public static Pose Shoot3 = new Pose(56, 90, Math.toRadians(125));
    public static Pose A3 = new Pose(15, 35, Math.toRadians(270));
    public static Pose I3 = new Pose(15, 10, Math.toRadians(270));








    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private Intake intake;
    private Shooter shooter;
    private Indexer indexer;




    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);

        VoltageSensor vs = bot.hMap.voltageSensor.iterator().next();

        Follower f = constants.createFollower(bot.hMap);
        f.setStartingPose(start);
        f.update();

        Intake intake = new Intake(bot);
        intake.register();
        Shooter shooter = new Shooter(bot);
        shooter.register();
        Indexer indexer = new Indexer(bot);
        indexer.register();





        SequentialCommandGroup wallAuto =
                new SequentialCommandGroup(
                        new InstantCommand(()-> intake.setVelocity(0.8)),
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(new BezierLine(start, Shoot0))
                                .setLinearHeadingInterpolation(start.getHeading(), Shoot0.getHeading())
                                .build()
                        ),
                        new InstantCommand(()-> shooter.setVelocity(6000.00)),
                        new WaitCommand(3000),
                        new InstantCommand(()-> indexer.setPosition(InOpen)),
                        new WaitCommand(3000),
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(Shoot0, A0))
                                        .setLinearHeadingInterpolation(Shoot0.getHeading(), A0.getHeading())
                                        .build()
                                ),
                                new InstantCommand(()-> shooter.setVelocity(0.0)),
                                new InstantCommand(()-> indexer.setPosition(InClosed)),
                                new WaitCommand(1000),
                                new SequentialCommandGroup(
                                        new FollowPathCommand(f, f.pathBuilder()
                                                .addPath(new BezierLine(A0, I0))
                                                .setLinearHeadingInterpolation(A0.getHeading(), I0.getHeading())
                                                .build()
                                        ),
                                        new SequentialCommandGroup(
                                                new FollowPathCommand(f, f.pathBuilder()
                                                        .addPath(new BezierLine(I0, Shoot1))
                                                        .setLinearHeadingInterpolation(I0.getHeading(), Shoot1.getHeading())
                                                        .build()
                                                ),
                                                new InstantCommand(()-> shooter.setVelocity(6000.00)),
                                                new WaitCommand(3000),
                                                new InstantCommand(()-> indexer.setPosition(InOpen)),
                                                new WaitCommand(3000),
                                                new SequentialCommandGroup(
                                                        new FollowPathCommand(f, f.pathBuilder()
                                                                .addPath(new BezierLine(Shoot1, A1))
                                                                .setLinearHeadingInterpolation(Shoot1.getHeading(), A1.getHeading())
                                                                .build()
                                                        ),
                                                        new InstantCommand(()-> shooter.setVelocity(0.0)),
                                                        new InstantCommand(()-> indexer.setPosition(InClosed)),
                                                        new WaitCommand(1000),
                                                        new SequentialCommandGroup(
                                                                new FollowPathCommand(f, f.pathBuilder()
                                                                        .addPath(new BezierLine(A1, I1))
                                                                        .setLinearHeadingInterpolation(A1.getHeading(), I1.getHeading())
                                                                        .build()
                                                                ),
                                                                new SequentialCommandGroup(
                                                                        new FollowPathCommand(f, f.pathBuilder()
                                                                                .addPath(new BezierLine(I1, Shoot2))
                                                                                .setLinearHeadingInterpolation(I1.getHeading(), Shoot2.getHeading())
                                                                                .build()
                                                                        ),
                                                                        new InstantCommand(()-> shooter.setVelocity(1800.00)),
                                                                        new WaitCommand(3000),
                                                                        new InstantCommand(()-> indexer.setPosition(InOpen)),
                                                                        new WaitCommand(3000)
                                                                        ),
                                                                                        new SequentialCommandGroup(
                                                                                                new FollowPathCommand(f, f.pathBuilder()
                                                                                                        .addPath(new BezierLine(Shoot2, A2))
                                                                                                        .setLinearHeadingInterpolation(Shoot2.getHeading(), A2.getHeading())
                                                                                                        .build()
                                                                                                ),
                                                                                                new InstantCommand(()-> indexer.setPosition(InClosed)),
                                                                                                new SequentialCommandGroup(
                                                                                                        new FollowPathCommand(f, f.pathBuilder()
                                                                                                                .addPath(new BezierLine(A2, I2))
                                                                                                                .setLinearHeadingInterpolation(A2.getHeading(), I2.getHeading())
                                                                                                                .build()
                                                                                                        ),
                                                                                                        new SequentialCommandGroup(
                                                                                                                new FollowPathCommand(f, f.pathBuilder()
                                                                                                                        .addPath(new BezierLine(I2, Shoot3))
                                                                                                                        .setLinearHeadingInterpolation(I2.getHeading(), Shoot3.getHeading())
                                                                                                                        .build()
                                                                                                                ),
                                                                                                                new InstantCommand(()-> shooter.setVelocity(1800.00)),
                                                                                                                new WaitCommand(3000),
                                                                                                                new InstantCommand(()-> indexer.setPosition(InOpen)),
                                                                                                                new WaitCommand(3000),
                                                                                                                new SequentialCommandGroup(
                                                                                                                        new FollowPathCommand(f, f.pathBuilder()
                                                                                                                                .addPath(new BezierLine(Shoot3, A3))
                                                                                                                                .setLinearHeadingInterpolation(Shoot3.getHeading(), A3.getHeading())
                                                                                                                                .build()
                                                                                                                        ),
                                                                                                                        new InstantCommand(()-> indexer.setPosition(InClosed)),
                                                                                                                        new SequentialCommandGroup(
                                                                                                                                new FollowPathCommand(f, f.pathBuilder()
                                                                                                                                        .addPath(new BezierLine(A3, I3))
                                                                                                                                        .setLinearHeadingInterpolation(A3.getHeading(), I3.getHeading())
                                                                                                                                        .build()
                                                                                                                                )

                                                                                                                                )

                                                                                                                        )

                                                                                                                )

                                                                                               )
                                                                                )
                                                                    )
                                                        )
                                                )

                                )
                        )
                );




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
        while (opModeInInit()){
            new InstantCommand(()-> intake.setVelocity(0.8));
        }
    }
}
