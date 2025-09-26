package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.utils.ImuGlobal;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ResetIMU", group = "Tools")
public class resetImu extends CommandOpMode {
    public void initialize() {
        CommandScheduler.getInstance().reset();
        ImuGlobal.getImu(hardwareMap).resetYaw();
        MultipleTelemetry telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telem.addData("IMU", "Reset");
        telem.update();
    }
}