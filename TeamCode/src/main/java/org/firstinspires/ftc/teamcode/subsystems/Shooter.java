package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Bot;

@Config
public class Shooter extends SubsystemBase {
    private final Bot bot;

    private IMU imu = null;

    private final DcMotorEx shooter;

    public static Pose pose;


    public Shooter(Bot bot) {
        this.bot = bot;

        shooter = bot.hMap.get(DcMotorEx.class, "Shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);



    }





    @Override
    public void periodic() {


        if (bot.opertator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5){
            shooter.setPower(1);
        } else {
            shooter.setPower(0);
        }


    }
}