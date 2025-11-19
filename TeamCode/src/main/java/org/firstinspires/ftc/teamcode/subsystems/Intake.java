package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Bot;

public class Intake implements Subsystem {

    private DcMotorEx Intake;
    @Config
    public static class ShooterPIDF{
        public static double kp = 10;
        public static double ki = 0;
        public static double kd = 0.05;
        public static double kf = -0.005;
    }
    public static double targetVelocity = 10000;
    private Bot bot;


    public Intake(Bot bot) {
        this.bot = bot;

        Intake = bot.hMap.get(DcMotorEx.class, "Intake");


        Intake.setDirection(DcMotorSimple.Direction.REVERSE);

        Intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        Intake.setVelocityPIDFCoefficients(
                ShooterPIDF.kp,
                ShooterPIDF.ki,
                ShooterPIDF.kd,
                ShooterPIDF.kf
        );


    }

    @Override
    public void periodic(){
        double currentVelocity = Intake.getVelocity();

        bot.telem.addData("Target Velocity", targetVelocity);
        bot.telem.addData("Current Velocity", currentVelocity);


    }
    public void setVelocity(){
        Intake.setVelocity(targetVelocity);



    }
}