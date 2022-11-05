package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp
public class MotorTest extends OpMode {
    private DcMotor Port0C = null;
    private DcMotor Port1C = null;
    private DcMotor Port2C = null;
    private DcMotor Port3C = null;
    private DcMotor Port0E = null;
    private DcMotor Port1E = null;
    private DcMotor Port2E = null;

    @Override
    public void init(){
        Port0C = hardwareMap.get(DcMotor.class, "00");
        Port1C = hardwareMap.get(DcMotor.class, "01");
        Port2C = hardwareMap.get(DcMotor.class, "02");
        Port3C = hardwareMap.get(DcMotor.class, "03");
        Port0E = hardwareMap.get(DcMotor.class, "10");
        Port1E = hardwareMap.get(DcMotor.class, "11");
        Port2E = hardwareMap.get(DcMotor.class, "12");

        Port0C.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Port1C.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Port3C.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            Port0C.setPower(0.5);
        }
        if (gamepad1.b){
            Port1C.setPower(0.5);
        }
        if (gamepad1.x){
            Port2C.setPower(0.5);
        }
        if (gamepad1.y){
            Port3C.setPower(0.5);
        }
        telemetry.addData("Port 0", Port0C.getCurrentPosition());
        telemetry.addData("Port 1", Port1C.getCurrentPosition());
        telemetry.addData("Port 3", Port3C.getCurrentPosition());
        telemetry.update();
    }
}
