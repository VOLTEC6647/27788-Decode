package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sun.tools.javac.util.Pair;


import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.ArrayList;
import java.util.List;

public class PoseEstimatorAprilTagProcessor {

    AprilTagProcessor aprilTag;
    Camera camera;
    LinearOpMode linearOpMode;
    List<AprilTagDetection> currentDetections = new ArrayList<>();

    public PoseEstimatorAprilTagProcessor(Camera camera, LinearOpMode linearOpMode) {
        this.camera = camera;
        this.linearOpMode = linearOpMode;
    }

    /**
     * Also update currentDetections
     * @return  Null if no result or result too unreliable
     * */
    public Vector2d getCurrentPosFromAprilTag(double camAngleDeg) {
        //Get april tag detection
        currentDetections = aprilTag.getDetections();
        if (currentDetections.isEmpty()) return null;
        float decisionMarginSum = 0;
        ArrayList<Pair<Vector2d, Float>> robotPoseResult = new ArrayList<>();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata == null) continue;
            decisionMarginSum += detection.decisionMargin;
            //Storing the pose from the detection and its decision margin
            robotPoseResult.add(new Pair<>(getCameraPoseFromAprilTagDetection(detection, camAngleDeg), detection.decisionMargin));
        }
        if (decisionMarginSum < 0.1) return null;
        Vector2d currentPose = new Vector2d(0, 0);
        //Weighted average of all pose from apriltag
        for (Pair<Vector2d, Float> result : robotPoseResult) {
            currentPose = currentPose.plus(result.fst.times(result.snd / decisionMarginSum));
        }
        return currentPose;
    }

    private Vector2d getCameraPoseFromAprilTagDetection(AprilTagDetection detection, double camAngleDeg) {
        VectorF _pos = detection.metadata.fieldPosition;

        //Global Position of apriltag
        Vector2d aprilTagPos = new Vector2d(_pos.get(0), _pos.get(1));

        //Convert unit if needed, TODO: Determine unit and convert beforehand
        float conversionFactor = detection.metadata.distanceUnit == DistanceUnit.METER ? (float) 5000/127 : 1;
        Vector2d cameraToApriltag = new Vector2d((float) detection.ftcPose.x * conversionFactor, (float) detection.ftcPose.y * conversionFactor);

        //XY swapped between april tag reference frame and FTC reference frame
        double radians = Math.toRadians(-90 + wrapAngle(camAngleDeg, WRAP_ANGLE_TYPE.zeroTo360));
        double cos = Math.cos(radians);
        double sin = Math.sin(radians);


        //Rotate cameraToApriltag vector from the robot perspective (robot based) to
        //global perspective (field based)
        Vector2d cameraToAprilTagWorld = new Vector2d(
                cameraToApriltag.getX() * cos - cameraToApriltag.getY() * sin,
                cameraToApriltag.getX() * sin + cameraToApriltag.getY() * cos
        );
        return aprilTagPos.minus(cameraToAprilTagWorld);
    }

    public void initAprilTagProcessor() {

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal inside wrapper class
        camera.addProcessorToQueue(aprilTag);
    }
    public List<AprilTagDetection> getCurrentDetections()
    {
        return currentDetections;
    }
    public enum WRAP_ANGLE_TYPE {
        zeroTo360, minus180To180, zeroTo2Pi, minusPiToPi
    }
    public static double wrapAngle(double angle, WRAP_ANGLE_TYPE type) {
        switch (type) {
            default:
            case zeroTo360:
                while (angle >= 360) angle -= 360;
                while (angle < 0) angle += 360;
                return angle;
            case minus180To180:
                while (angle >= 180) angle -= 360;
                while (angle < -180) angle += 360;
                return angle;
            case zeroTo2Pi:
                while (angle >= 2 * Math.PI) angle -= Math.PI;
                while (angle < 0) angle += Math.PI;
                return angle;
            case minusPiToPi:
                while (angle >= Math.PI) angle -= Math.PI;
                while (angle < -Math.PI) angle += Math.PI;
                return angle;
        }
    }
}
