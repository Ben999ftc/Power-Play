package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class DrivingWill extends OpMode
{
    Lift lift = new Lift();
    //Declare motors and variables//


    //Motors: 4 wheels, Duckydropper
    //Servos: none
    private boolean isPressed = false;
    private int height_count = 0;

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo claw = null;
    private DigitalChannel button = null;
    private Servo vee = null;
    private RevBlinkinLedDriver lights = null;



    final double INTAKE_SPIN_SPEED = 1.0;

    @Override
    public void init() {
        //Declare variables for phone to recognise//

        //names on the config
//
        lift.init(hardwareMap);

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        claw = hardwareMap.get(Servo.class, "claw");
        button = hardwareMap.get(DigitalChannel.class, "button");
        vee = hardwareMap.get(Servo.class, "vee");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");


        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);







//Set the Direction for the motors to turn when the robot moves forward//

//        Rotator1.setDirection(DcMotorSimple.Direction.FORWARD);
//        Rotator2.setDirection(DcMotorSimple.Direction.FORWARD);
//        Slides.setDirection(DcMotorSimple.Direction.FORWARD);


        telemetry.addData("status", "Initialized");
    }


    //Set variables//
    @Override
    public void loop() {
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;


        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        leftFrontPower = Range.clip(drive + turn + strafe, -1, 1);
        rightFrontPower = Range.clip(drive - turn - strafe, -1, 1);
        leftBackPower = Range.clip(drive + turn - strafe, -1, 1);
        rightBackPower = Range.clip(drive - turn + strafe, -1, 1);


        // not locking the wheels while turning
        if (gamepad1.right_stick_x >= 0.1 && gamepad1.left_stick_y <= -0.1) {
            rightFrontPower = 0.5;  // was 0.2
            rightBackPower = 0.5;   // was 0.2
            leftFrontPower = -0.5;  //was -0.3
            leftBackPower = -0.5;   //was -0.3
        } else if (gamepad1.right_stick_x <= -0.1 && gamepad1.left_stick_y <= -0.1) {
            leftFrontPower = 0.5;   //was 0.2
            leftBackPower = 0.5;    //was 0.2
            rightFrontPower = -0.5; //was -0.3
            rightBackPower = -0.5;  //was -0.3
        }

        if(gamepad1.right_bumper){
            leftFrontPower /= 2;
            leftBackPower /= 2;
            rightFrontPower /= 2;
            rightBackPower /= 2;
        }

        if (gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0){
            leftFrontPower /= 2;
            leftBackPower /= 2;
            rightFrontPower /= 2;
            rightBackPower /= 2;
        }


        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        if(height_count > 2){
            height_count = 2;
        }
        else if(height_count < 0){
            height_count = 0;
        }
        else if(gamepad2.right_bumper && !isPressed){
            height_count++;
            isPressed = true;

        }
        else if(gamepad2.left_bumper && !isPressed){
            height_count--;
            isPressed = true;
        }
        else{
            isPressed = false;
        }

        if(gamepad2.right_trigger == 1.0){
            lift.setHeight(165);
        }
        else if(gamepad2.right_bumper){
            lift.setHeight(380);
        }
        else if(gamepad2.left_bumper){
            lift.setHeight(0);
        }

        if(gamepad2.dpad_up){
            lift.armAngle(160);
        }
        else if(gamepad2.dpad_down){
            lift.armAngle(0);
        }
        else if(gamepad2.dpad_right){
            lift.armAngle(190);
        }
        else if(gamepad2.dpad_left){
            lift.armAngle(101);
        }


        if (lift.getAngle() > 160 && gamepad2.x){
            claw.setPosition(0.25);
        }
        else if(gamepad2.x) {
            claw.setPosition(0.1);
        }
        else {
            claw.setPosition(0.5);
        }

        telemetry.addData("Arm Position:", lift.getAngle());
        telemetry.update();

        if (gamepad1.right_trigger > 0.1){
            vee.setPosition(1);
        }
        else{
            vee.setPosition(0.6);
        }

        if(button.getState() == true){
            gamepad1.rumble(100);
            gamepad2.rumble(100);
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
        else{
            gamepad1.stopRumble();
            gamepad2.stopRumble();
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }

        //}
        //telemetry.addData("Arm position", lift.getAngle());
    }

    @Override
    public void stop() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);


    }

}