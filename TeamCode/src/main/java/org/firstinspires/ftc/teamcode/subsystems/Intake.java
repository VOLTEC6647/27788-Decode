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
public class Intake extends SubsystemBase {
    private final Bot bot;

    private IMU imu = null;

    private final DcMotorEx intake;

    public static Pose pose;


    public Intake(Bot bot) {
        this.bot = bot;

        intake = bot.hMap.get(DcMotorEx.class, "Intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);



    }





    @Override
    public void periodic() {


        bot.telem.addData("ola", "si funciona");

        if (bot.opertator.getButton(GamepadKeys.Button.LEFT_BUMPER)) {

            intake.setMotorEnable();
        } else if (!bot.opertator.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            intake.setMotorDisable();


            //bot.telem.addData("Pose",
            //        "X: " + pose.getX(DistanceUnit.MM) +
            //                ", Y: " + pose.getY(DistanceUnit.MM) +
            //                ", Heading: " + pose.getHeading(AngleUnit.DEGREES));
        }


    }}