package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class ImuGlobal extends Robot {

    private static IMU imu;
    public static IMU getImu(HardwareMap hMap) {
        if (imu == null) {
            imu = hMap.get(IMU.class, "imu");
            imu.initialize(
                    new IMU.Parameters(
                            new RevHubOrientationOnRobot(
                                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                            )
                    )
            );
            imu.resetYaw();
        }
        return imu;
    }

}