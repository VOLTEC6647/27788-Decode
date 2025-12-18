package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
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

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);
        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);

        VoltageSensor vs = hardwareMap.voltageSensor.iterator().next();

        Follower f = constants.createFollower(hardwareMap);
        f.setStartingPose(start);
        f.update();

        // --- FIXED LINE 85 ---
        // Just pass hardwareMap once as required by your Intake constructor
        Intake intake = new Intake(bot);
        intake.register();}
}

