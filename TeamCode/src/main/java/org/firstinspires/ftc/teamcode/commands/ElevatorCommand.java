package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.AnyArm;

public class ElevatorCommand extends CommandBase {
    private final AnyArm elevator;
    private int setPoint;
    public ElevatorCommand(AnyArm elevator, int setPoint){
        this.elevator = elevator;
        this.setPoint = setPoint;
        addRequirements(elevator);
    }

    public void execute() {
        elevator.setSetpoint(setPoint);
    }
    @Override
    public boolean isFinished(){
        return Math.abs(setPoint - elevator.getMotorRotate().getCurrentPosition()) < 20;
    }
}
