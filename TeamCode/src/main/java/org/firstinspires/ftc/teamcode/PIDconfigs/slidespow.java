package org.firstinspires.ftc.teamcode.PIDconfigs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled

@TeleOp public class slidespow extends LinearOpMode {

    public DcMotor uppies;
    public DcMotor uppies1;
// declaring the four motors

    @Override

    public void runOpMode(){

        // declaring eli chill sucks


        uppies = hardwareMap.get(DcMotor.class, "uppies");
        uppies1 = hardwareMap.get(DcMotor.class, "uppies1");


        //reversing motors

        waitForStart();

        while (opModeIsActive()){

            if (Math.abs(gamepad1.right_stick_y) > 0.1){
                //uppies.setPower(gamepad1.right_stick_y * 1);
                uppies1.setPower(gamepad1.right_stick_y * -1);

            } else {
              //  uppies.setPower(0);
                uppies1.setPower(0);

            }




        }







    }



}
