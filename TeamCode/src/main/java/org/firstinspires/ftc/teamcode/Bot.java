package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.ImuGlobal;

import lombok.Getter;
import lombok.Setter;

public class Bot extends Robot {
    private final IMU imu;
    public final Telemetry telem;
    public final HardwareMap hMap;
    public final GamepadEx driver;
    public final GamepadEx opertator;
    public double speed = 1;
    public double rotMultiplier = 1;



    public final ElapsedTime timer;

    public Bot(Telemetry telem, HardwareMap hMap, GamepadEx gamepad, GamepadEx gamepad2) {
        this.telem = telem;
        this.hMap = hMap;
        this.driver = gamepad;
        this.opertator = gamepad2;

        this.timer = new ElapsedTime();

        imu = ImuGlobal.getImu(hMap);
    }
    /**
     * Get the IMU object for the robot
     * @return the IMU object
     */
    public IMU getImu() { return imu; }

    private Rotation2d rotationOffset = Rotation2d.fromDegrees(0);

    public void setRotationOffset(Rotation2d rotationOffset) {
        this.rotationOffset = rotationOffset;
    }

    public Rotation2d getYaw(){
        return new Rotation2d(getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)).rotateBy(rotationOffset);
    }

    /**
     * Get the MecanumDrivetrain subsystem of the robot
     * @return the mecanum subsystem of the robot
     */

    public double getTimestamp() {
        return timer.seconds();
    }
}