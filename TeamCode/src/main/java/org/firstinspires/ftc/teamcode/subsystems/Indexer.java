
package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Bot;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.utils.RobotConstants.*;

public class Indexer implements Subsystem {

    private Servo Indexer;
    private Bot bot;
    HardwareMap h;
    Gamepad g;
    public double setPoint = 0;


    public Indexer (Bot bot){
        this.bot = bot;

        Indexer = h.get(Servo.class, "Indexer");
    }

    public void setSetpoint (double setPoint) {
        this.setPoint = setPoint;
        Indexer.setPosition(setPoint);
    }


    public void periodic(){
        if(g.x) {
            Indexer.setPosition(InOpen);
        } else {
            Indexer.setPosition(InClosed);
        }
    }
    public void setPosition(double setpoint){
        setPoint = setpoint;
        Indexer.setPosition(setPoint);


    }
}
