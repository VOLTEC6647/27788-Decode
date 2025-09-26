package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Bot;

public class Arm implements Subsystem {
    //private Servo clawServo;

    private Servo arm;
    private Bot bot;
    public double setPoint = 0;

    public Arm(Bot bot) {
        this.bot = bot;

        arm = bot.hMap.get(Servo.class,"arm");
        arm.setDirection(Servo.Direction.FORWARD);

        //arm.setPosition(0.13);


    }
    @Override
    public void periodic(){
        //double currentPosition = clawServo.getPosition();



    }
    public void setPosition(double setpoint){
        setPoint = setpoint;
        arm.setPosition(setpoint);


    }
}
