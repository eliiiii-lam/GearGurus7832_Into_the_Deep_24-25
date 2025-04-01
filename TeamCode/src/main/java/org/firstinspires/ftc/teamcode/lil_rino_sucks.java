package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled

@TeleOp public class lil_rino_sucks extends LinearOpMode {

    public DcMotor fR;
    public DcMotor fL;
    public DcMotor bR;
    public DcMotor bL;
// declaring the four motors

    @Override

    public void runOpMode(){

        // declaring eli chill sucks
        fR = hardwareMap.get(DcMotor.class, "fR");
        fL = hardwareMap.get(DcMotor.class, "fL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        bL = hardwareMap.get(DcMotor.class, "bL");


        //reversing motors
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){

        if (Math.abs(gamepad1.right_stick_y) > 0.1){
            fR.setPower(gamepad1.right_stick_y * 1);
            bR.setPower(gamepad1.right_stick_y * 1);
        } else {
            fR.setPower(0);
            bR.setPower(0);
        }

            if (Math.abs(gamepad1.right_stick_y) > 0.1){
                fL.setPower(gamepad1.right_stick_y * 1);
                bL.setPower(gamepad1.right_stick_y * 1);
            } else {
                fR.setPower(0);
                bR.setPower(0);
            }


        }







    }



}
