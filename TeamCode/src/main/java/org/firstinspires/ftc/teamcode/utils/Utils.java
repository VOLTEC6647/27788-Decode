package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

public class Utils {
    public static double getRotationTarget(MecanumDrive drivetrain, Rotation2d currentRotation){
        double robotRotation = drivetrain.getRobotOrientation().getDegrees();
        double closest;
        if(Math.abs(currentRotation.getDegrees()-robotRotation) < 180){
            closest = currentRotation.getDegrees();
        } else {
            if(currentRotation.getDegrees() > robotRotation){
                closest = currentRotation.getDegrees() - 360;
            } else {
                closest = currentRotation.getDegrees() + 360;
            }
        }
        return closest;
    }
    public static double getRotationTargetTank(MecanumDrive drivetrain, Rotation2d currentRotation){
        double robotRotation = drivetrain.getRobotOrientation().getDegrees();
        double closest;
        if(Math.abs(currentRotation.getDegrees()-robotRotation) < 180){
            closest = currentRotation.getDegrees();
        } else {
            if(currentRotation.getDegrees() > robotRotation){
                closest = currentRotation.getDegrees() - 360;
            } else {
                closest = currentRotation.getDegrees() + 360;
            }
        }
        return closest;
    }
}
