package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Bot;

public class ClawPivot implements Subsystem {
    //private Servo clawServo;

    private Servo clawPivot;
    private Bot bot;
    public double setPoint = 0;

    public ClawPivot(Bot bot) {
        this.bot = bot;

        clawPivot = bot.hMap.get(Servo.class,"cPivot");
        clawPivot.setDirection(Servo.Direction.FORWARD);

    }
    @Override
    public void periodic(){
        //double currentPosition = clawServo.getPosition();





    }
    public void setPosition(double setpoint){
        setPoint = setpoint;
        clawPivot.setPosition(setpoint);


    }
}
