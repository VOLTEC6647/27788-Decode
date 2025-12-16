package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Indexer implements Subsystem {

    private Servo Indexer;
    HardwareMap h;
    Gamepad g;


    public Indexer (HardwareMap h, Gamepad g){
        this.h = h;
        this.g = g;

        Indexer = h.get(Servo.class, "Indexer");
    }


    public void periodic(){
        if(g.x) {
            Indexer.setPosition(0.2);
        } else {
            Indexer.setPosition(0);
        }
    }
}

