package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Bot;

public class Intake implements Subsystem {

    private DcMotorEx intake;

    private Bot bot;
    public double power = 0;


    public Intake(Bot bot) {
        this.bot = bot;

        intake = bot.hMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

    }


    @Override
    public void periodic(){

    }
    public void setPower(double power){
        power = power;
        intake.setPower(power);


    }
}