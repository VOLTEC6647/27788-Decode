package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Vision.Vision;

@Config
@TeleOp(name = "LimelightTest")
public class VisionTest extends LinearOpMode {
    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;


    @Override
    public void runOpMode(){
        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);
        bot.getImu().resetYaw();


        Vision vision = new Vision(bot);

        vision.initializeCamera();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            CommandScheduler.getInstance().run();

            telemetry.addData("Distance", vision.getDistance());
            telemetry.addData("TX", vision.getTx(0.0));
            telemetry.addData("TY", vision.getTy(0.0));
            telemetry.addData("Area", vision.isTargetVisible());
            telemetry.update();

        }

        CommandScheduler.getInstance().reset();
    }
}