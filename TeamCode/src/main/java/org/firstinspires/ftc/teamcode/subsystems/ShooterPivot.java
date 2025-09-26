package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Bot;

public class ShooterPivot implements Subsystem {
    //private Servo clawServo;

    private Servo clawServo;
    private Bot bot;
    public double setPoint = 0;
    boolean garraAbierta = true;

    // Variable para evitar cambios múltiples con una sola pulsación
    boolean botonPresionadoAnteriormente = false;

    public ShooterPivot(Bot bot) {
        this.bot = bot;

        clawServo = bot.hMap.get(Servo.class,"claw");
        clawServo.setDirection(Servo.Direction.FORWARD);

    }
    public void setSetpoint (double setPoint) {
        this.setPoint = setPoint;
        clawServo.setPosition(setPoint);
    }


    @Override
    public void periodic(){
        //double currentPosition = clawServo.getPosition();

        boolean botonPresionadoAhora = bot.opertator.gamepad.a;

        if (botonPresionadoAhora && !botonPresionadoAnteriormente) {
            if (garraAbierta) {
                clawServo.setPosition(0);
                garraAbierta = false;
            } else {
                clawServo.setPosition(0.3);
                garraAbierta = true;
            }
        }

        botonPresionadoAnteriormente = botonPresionadoAhora;


    }
    public void setPosition(double setpoint){
        setPoint = setpoint;
        clawServo.setPosition(setPoint);


    }
}
