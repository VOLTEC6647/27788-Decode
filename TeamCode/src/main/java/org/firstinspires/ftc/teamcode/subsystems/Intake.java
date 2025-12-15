package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Bot;

public class Intake implements Subsystem {

    private DcMotorEx Intake;

    public static double targetVelocity = 1800;
    private Bot bot;


    public Intake(Bot bot) {
        this.bot = bot;

        Intake = bot.hMap.get(DcMotorEx.class, "Intake");


        Intake.setDirection(DcMotorSimple.Direction.FORWARD);

        Intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void periodic(){
        double currentVelocity = Intake.getVelocity();

        bot.telem.addData("Target Velocity", targetVelocity);
        bot.telem.addData("Current Velocity", currentVelocity);


    }
    public void setVelocity(double velocity){
        Intake.setPower(velocity);



    }
}