package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Autonomous
public class RightAuto extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int one = 1;
    int two = 2;
    int three = 3;

    AprilTagDetection tagOfInterest = null;



    @Override
    public void runOpMode()
    {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        Lift lift = new Lift();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        Pose2d startPose = new Pose2d(-40, 63, Math.toRadians(270));
        telemetry.addData( "Passed", "1");
        telemetry.update();
        int position = 2;
        robot.setPoseEstimate(startPose);
        TrajectorySequence preload = robot.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(1);
                    lift.backArmSensor();
                    robot.vee.setPosition(1);
                })
                .splineToConstantHeading(new Vector2d(-36, 50), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-36, 6), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-36, 12), Math.toRadians(90))
                .addTemporalMarker(() -> {
                    lift.setHeight(390);
                })
                .turn(Math.toRadians(-135))
//                .splineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(135)), Math.toRadians(315))
//                .splineToConstantHeading(new Vector2d(-36, -2), Math.toRadians(90),
//                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*0.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.5))
//                .turn(Math.toRadians(-90))
//                .back(7)
                .lineToConstantHeading(new Vector2d(-30, 4))
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(0.65);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(1);
                    lift.armAngle(0);
                })
                .forward(6)
//                .lineToConstantHeading(new Vector2d(-36, 11.5))
                .addTemporalMarker(() -> {
                    lift.armAngle(0);
                    robot.claw.setPosition(0.4);
                    lift.slidesensor();
                })
                .build();
        telemetry.addData( "Passed", "2");
        telemetry.update();
        TrajectorySequence cycle = robot.trajectorySequenceBuilder(preload.end())
                .splineToLinearHeading(new Pose2d(-45, 13, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    lift.setHeight(140);
                })
                .splineToConstantHeading(new Vector2d(-65, 13), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(1);
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    lift.setHeight(390);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    lift.backArmSensor();
                })
                .splineToConstantHeading(new Vector2d(-45, 13), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-30, 4, Math.toRadians(135)), Math.toRadians(315))
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(0.65);
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(1);
                    lift.armAngle(0);
                })
                .forward(6)
                .addTemporalMarker(() -> {
                    lift.slidesensor();
                })
                .build();
        telemetry.addData( "Passed", "3");
        telemetry.update();
        TrajectorySequence cycle2 = robot.trajectorySequenceBuilder(cycle.end())
                .splineToLinearHeading(new Pose2d(-45, 13, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    lift.setHeight(95);
                    robot.claw.setPosition(0.1);
                })
                .splineToConstantHeading(new Vector2d(-65, 13), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(1);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    lift.setHeight(390);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    lift.backArmSensor();
                })
                .splineToConstantHeading(new Vector2d(-45, 13), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-30, 4, Math.toRadians(135)), Math.toRadians(315))
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(0.65);
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(1);
                    lift.armAngle(0);
                })
                .forward(6)
                .addTemporalMarker(() -> {
                    lift.slidesensor();
                })
                .build();
        telemetry.addData( "Passed", "4");
        telemetry.update();
        TrajectorySequence parkright = robot.trajectorySequenceBuilder(cycle2.end())
                .splineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(180)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    lift.setHeight(0);
                    robot.vee.setPosition(0.6);
                    lift.armAngle(0);
                })
                .waitSeconds(15)
                .build();
        telemetry.addData( "Passed", "5");
        telemetry.update();
        TrajectorySequence parkleft = robot.trajectorySequenceBuilder(cycle2.end())
                .splineToLinearHeading(new Pose2d(-12, 12, Math.toRadians(180)), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    lift.slidesensor();
                    robot.vee.setPosition(0.6);
                    lift.armAngle(0);
                })
                .waitSeconds(15)
                .build();
        telemetry.addData( "Passed", "6");
        telemetry.update();
        TrajectorySequence middle = robot.trajectorySequenceBuilder(cycle2.end())
                .addTemporalMarker(() -> {
                    lift.slidesensor();
                    robot.vee.setPosition(0.6);
                })
                .splineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(180)), Math.toRadians(45))
                .build();
        telemetry.addData( "Passed", "7");
        telemetry.update();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        telemetry.addData( "ajkdh", "Got through trajectories");
        telemetry.update();
        lift.init(hardwareMap);
        robot.claw.setPosition(0.7);
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == one || tag.id == two || tag.id == three)
                    {
                        tagOfInterest = tag;
                        telemetry.addData("Tag #", tagOfInterest.id);
                        tagFound = true;
                        break;
                    }
                }
            }
            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest != null){
            position = tagOfInterest.id;
        }
        robot.followTrajectorySequence(preload);
        robot.followTrajectorySequence(cycle);
        robot.followTrajectorySequence(cycle2);
        if (position == 3){
            robot.followTrajectorySequence(parkright);
        }
        else if (position == 1){
            robot.followTrajectorySequence(parkleft);
        }
        else{
            robot.followTrajectorySequence(middle);
        }
    }

}
