package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Lift {
    private DcMotorEx lift_left;
    private DcMotorEx lift_right;
    public DcMotorEx arm;
    double lastError = 0;
    public TouchSensor magnet;
    public TouchSensor slide_stop;
    public Servo claw;
    public ColorSensor colour;
    ElapsedTime timer = new ElapsedTime();


    private static void setTimeout(Runnable runnable, int delay){
        new Thread(() -> {
            try {
                Thread.sleep(delay);
                runnable.run();
            }
            catch (Exception e){
                System.err.println(e);
            }
        }).start();
    }

    public void init(HardwareMap hardwareMap){

        lift_left = hardwareMap.get(DcMotorEx.class, "left_lift");
        lift_right = hardwareMap.get(DcMotorEx.class, "right_lift");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        magnet = hardwareMap.get(TouchSensor.class, "magnet");
        slide_stop = hardwareMap.get(TouchSensor.class, "slide_stop");
        claw = hardwareMap.get(Servo.class, "claw");
        colour = hardwareMap.get(ColorSensor.class, "colour");


        lift_left.setTargetPosition(0);
        lift_right.setTargetPosition(0);
        arm.setTargetPosition(0);
        lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift_left.setDirection(DcMotorSimple.Direction.REVERSE);
        lift_right.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        lift_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void setHeight(double height){
        double ticks_per_mm = (384.5 / 112);
        double value = height * ticks_per_mm;
        int target = (int)value;
        lift_left.setTargetPosition(target);
        lift_right.setTargetPosition(target);
        lift_left.setVelocity(2785);
        lift_right.setVelocity(2785);
    }
    public void armAngle(double angle){
        double ticks_per_degree = (2786.2 / 360);
        double  value = (angle * ticks_per_degree);
        int target = (int)value;
        arm.setTargetPosition(target);
        arm.setVelocity(2000);
    }
    public void backArm(){
        armAngle(250);
    }
    public void stopArm(){
        arm.setVelocity(0);
    }
    public void stopslide(){
        lift_left.setVelocity(0);
        lift_right.setVelocity(0);
    }
    public void stopIfPressed() {
        if (magnet.isPressed()) {
            stopArm();
        } else {
            setTimeout(() -> this.stopIfPressed(), 25);
        }
    }
    public void resetSlides() {
        if (slide_stop.isPressed()){
            stopslide();
        }
        else {
            setTimeout(() -> this.resetSlides(), 20);
        }
    }
    public void backArmSensor() {
        pControl();
        setTimeout(() -> stopIfPressed(), 25);
    }
    public void slidesensor() {
        setHeight(0);
        setTimeout(() -> resetSlides(), 20);
    }
    public boolean detectCone(){
        if (colour.red() + 20 > colour.blue() && colour.red() + 20 > colour.green()){
            return true;
        }
        else if (colour.blue() + 20 > colour.red() && colour.blue() + 20 > colour.green()){
            return true;
        }
        else {
            return false;
        }
    }
    public void resetArm(){
        if(getAngle() >= 180){
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            double value = (2786.2 / 360) * 190 * -1;
            int target = (int)value;
            arm.setTargetPosition(target);
        }
        else {
            arm.setTargetPosition(0);
        }
        arm.setVelocity(2000);
    }
    public void PIDarm(double angle){
        double ticksPerDegree = (2786.2 / 360);
        double reference = angle * ticksPerDegree;

        double Kp = 0.0008;
        double Ki = 0;
        double Kd = 0;

        double error = reference - arm.getCurrentPosition();
        double integral = error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        arm.setPower((error * Kp) + (integral * Ki) + (derivative * Kd));
    }
    public void pControl(){
        armAngle(250);
        double ticksPerDegree = (2786.2/360);
        double proportion = (250 - (arm.getCurrentPosition() / ticksPerDegree)) / 250;
        double velocity = 2786.2 * proportion;
        arm.setVelocity(velocity);
    }
    public void openClaw(){
        if (getAngle() >= 100) {
            claw.setPosition(0.82);
        } else {
            claw.setPosition(0.65);
        }
    }
    public void closeClaw(){
        claw.setPosition(0.95);
    }
    public double getAngle(){
        return (arm.getCurrentPosition() / (2786.2 / 360));
    }

    public double getHeight(){return lift_left.getCurrentPosition() / (384.5/112);}
    public double getRightHeight(){return lift_right.getCurrentPosition() / (384.5/112);}

    public boolean getState(){return magnet.isPressed();}

}
