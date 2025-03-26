package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "The Holy Grail")
public class TheHolyGrail extends LinearOpMode {
    @Override
    public void runOpMode() {
        //Start position for RED
        Pose2d initialPose = new Pose2d(-10, 61.5, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Arm arm = new Arm(hardwareMap);
        //  resetRuntime();

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "Arm");

        if (gamepad2.right_stick_button){

            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        }

        waitForStart();

        TrajectoryActionBuilder spec1 = drive.actionBuilder(initialPose)

                .stopAndAdd(arm.closeClaw())
                .waitSeconds(0.5)
                .stopAndAdd(arm.closeClaw())
                .stopAndAdd(arm.armUp())
                .afterTime(0,arm.SetPosition(-440))
                .stopAndAdd(arm.closeClaw())
                .lineToY(31.5)
                .afterTime(0,arm.SetPosition(-800))
                .waitSeconds(0.1)
                .stopAndAdd(arm.openClaw())
                .afterTime(0.2,arm.SetPosition(20))
                .stopAndAdd(arm.armDown())
                .setReversed(true)

                .splineToConstantHeading(new Vector2d(16.5,34), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(16.5,25), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(16.5,18), Math.toRadians(270))

                .splineToConstantHeading(new Vector2d(33,12), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(35 ,45), Math.toRadians(270))
               /// /////////////////////////////////////////////////////////////////////////
                .setTangent(-60)
                .splineToConstantHeading(new Vector2d(23,35), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(25,30), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(38,15), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(38.1,45), Math.toRadians(270))
/// /////////////////////////////////////////////////////////////////////////////////////////
                .setTangent(-60)
                .splineToConstantHeading(new Vector2d(31,35), Math.toRadians(250))
                .splineToConstantHeading(new Vector2d(32,30), Math.toRadians(250))
                .splineToConstantHeading(new Vector2d(52,17), Math.toRadians(250))
                .splineToConstantHeading(new Vector2d(60,50), Math.toRadians(250))
                /////////////////////////////////////////////////////////////////////////////////////////////
                .strafeToLinearHeading(new Vector2d(19,56), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .strafeToLinearHeading(new Vector2d(19,61), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))


                //////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
                .stopAndAdd(arm.closeClaw())
                .waitSeconds(0.15)
                .afterTime(0,arm.SetPosition(-440))
                .stopAndAdd(arm.armUp())
                .strafeToLinearHeading(new Vector2d(-23,31), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .afterTime(0,arm.SetPosition(-800))
                .waitSeconds(0.1)
                .stopAndAdd(arm.openClaw())
                .afterTime(0.8,arm.SetPosition(0))
                .stopAndAdd(arm.armDown())
////////////////////////////////////////////////////////////////////////////////////////
                .strafeToLinearHeading(new Vector2d(19,56), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .strafeToLinearHeading(new Vector2d(19,61), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))

                .stopAndAdd(arm.closeClaw())
                .waitSeconds(0.15)
                .afterTime(0,arm.SetPosition(-440))
                .stopAndAdd(arm.armUp())
                .strafeToLinearHeading(new Vector2d(-21,35), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .afterTime(0,arm.SetPosition(-800))
                .waitSeconds(0.1)
                .stopAndAdd(arm.openClaw())
                .afterTime(0.8,arm.SetPosition(0))
                .stopAndAdd(arm.armDown())
////////////////////////////////////////////////////////////////////////////////////////
                .strafeToLinearHeading(new Vector2d(19,56), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .strafeToLinearHeading(new Vector2d(19,61), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))

                .stopAndAdd(arm.closeClaw())
                .waitSeconds(0.15)
                .afterTime(0,arm.SetPosition(-440))
                .stopAndAdd(arm.armUp())
                .strafeToLinearHeading(new Vector2d(-19,35), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .afterTime(0,arm.SetPosition(-800))
                .waitSeconds(0.1)
                .stopAndAdd(arm.openClaw())
                .afterTime(0.8,arm.SetPosition(0))
                .stopAndAdd(arm.armDown())
////////////////////////////////////////////////////////////////////////////////////////
                .strafeToLinearHeading(new Vector2d(19,56), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .strafeToLinearHeading(new Vector2d(19,61.5), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .stopAndAdd(arm.closeClaw())
                .waitSeconds(0.15)
                .afterTime(0,arm.SetPosition(-440))
                .stopAndAdd(arm.armUp())
                .strafeToLinearHeading(new Vector2d(-18,35), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .afterTime(0,arm.SetPosition(-800))
                .waitSeconds(0.1)
                .stopAndAdd(arm.openClaw())
                .afterTime(1.2,arm.SetPosition(0))
                .stopAndAdd(arm.armDown())
                .setTangent(45)
                .strafeToLinearHeading(new Vector2d(50,60), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 7))


////////////////////////////////////////////////////////////////////////////////////////





                ;
















        if (isStopRequested()) return;




        Actions.runBlocking(


                new ParallelAction(
                        arm.lockIntake(),
                        spec1.build(),
                        arm.UpdatePID()

                )


        );
    }
}