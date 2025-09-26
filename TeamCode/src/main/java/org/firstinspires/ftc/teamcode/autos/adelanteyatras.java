package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "rTeleOp", group = "TeleOp")

public class adelanteyatras extends CommandOpMode {


    public static Pose startingPose = new Pose(8, 112,0);

    //endregion


    public VisionPortal myVisionPortal;
    public Bot bot;
    public GeneratedPaths Path;
    private MultipleTelemetry telem;
    private MecanumDrive kiwiDrive;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    public void initialize() {
        CommandScheduler.getInstance().reset();

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);
        bot.getImu().resetYaw();



        Follower f = new Follower(hardwareMap, FConstants.class, LConstants.class);
        f.setPose(startingPose);
        f.setMaxPower(0.75);

        CommandScheduler.getInstance().reset();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //FtcDashboard.getInstance().startCameraStream(myVisionPortal, 30);


        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        // drive region


        //File myFileName = AppUtil.getInstance().getSettingsFile("team.txt");
        //String team = ReadWriteFile.readFile(myFileName);


        waitForStart();
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowPathCommand(f,GeneratedPaths.builder.build())
                )
        );



    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telem.addData("status","start");
        telem.update();
    }
}