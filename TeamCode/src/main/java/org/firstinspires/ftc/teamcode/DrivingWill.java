package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DrivingWill extends OpMode
{
    Lift lift = new Lift();
    //Declare motors and variables//

    private boolean isPressed = false;
    private boolean cone = false;
    private boolean stack = false;

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo vee = null;
    private ColorSensor colour = null;

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
        vee = hardwareMap.get(Servo.class, "vee");
        colour = hardwareMap.get(ColorSensor.class, "colour");



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

        if(gamepad1.right_bumper){
            leftFrontPower /= 2;
            leftBackPower /= 2;
            rightFrontPower /= 2;
            rightBackPower /= 2;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        if(gamepad2.right_trigger >= 0.5){
            lift.setHeight(165);
        }
        else if(gamepad2.right_bumper){
            lift.setHeight(380);
        }
        else if(gamepad2.left_bumper){
            if(lift.slide_stop.isPressed()){
                lift.stopslide();
            }
            else {
                lift.setHeight(0);
            }
        }
        else if (gamepad2.y) {
            lift.setHeight(140);
        }
        else if (gamepad2.b) {
            lift.setHeight(95);
        }
        else if (gamepad2.a) {
            lift.setHeight(80);
        }
        else if (gamepad2.left_trigger > 0.5){
            lift.setHeight(45);
        }

        if(gamepad2.dpad_up){
            lift.armAngle(160);
        }
        else if(gamepad2.dpad_down){
            lift.armAngle(0);
        }
        else if(gamepad2.dpad_left){
            lift.armAngle(101);
        }
        else if(gamepad2.dpad_right){
            if (lift.magnet.isPressed()){
                lift.stopArm();
            }
            else {

                lift.pControl();
            }
        } else {
            lift.stopArm();
        }

        if(gamepad2.left_stick_button){
            if(!isPressed){
                if (!stack){
                    stack = true;
                }
                else {
                    stack = false;
                }
                isPressed = true;
            }
        }
        else {
            isPressed = false;
        }

    //Claw Code: Opens with GP2 X and opens less when past vertical position
    // BIGGER CLOSES MORE*********************
        if (!stack) {
            if (gamepad2.x) {
                lift.openClaw();
            } else {
                lift.closeClaw();
            }
        }
        else {
            if (cone == true) {
                if (gamepad2.x) {
                    lift.openClaw();
                } else {
                    lift.closeClaw();
                }
            } else {
                if (gamepad2.x) {
                    lift.closeClaw();
                } else {
                    lift.openClaw();
                }
            }
        }


        if (gamepad1.left_trigger > 0.1){
            vee.setPosition(1);
        }
        else{
            vee.setPosition(0.6);
        }

        if((colour.red() >= colour.blue() + 20 && colour.red() >= colour.blue() +20) || (colour.blue() >= colour.red() + 20 && colour.blue() >= colour.green() + 20)){
            cone = true;
        }
        else{
            cone = false;
        }

        telemetry.addData("Activated", lift.getState());
        telemetry.addData("Left Height", lift.getHeight());
        telemetry.addData("Right Height", lift.getRightHeight());
        telemetry.addData("Arm Angle", lift.getAngle());
        telemetry.addData("Red", colour.red());
        telemetry.addData("Blue", colour.blue());
        telemetry.addData("Green", colour.green());
        telemetry.update();

    }

    @Override
    public void stop() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);


    }

}