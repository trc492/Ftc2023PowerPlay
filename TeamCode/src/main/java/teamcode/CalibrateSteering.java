/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Calibrate Steering Servos", group="Test")
public class CalibrateSteering extends LinearOpMode {
   private static final double DEF_90STEP = 0.340;
   private static final double LF_ZERO = 0.538;
   private static final double RF_ZERO = 0.500;
   private static final double LB_ZERO = 0.538;
   private static final double RB_ZERO = 0.475;
   private static final String[] wheelNames = {"lfWheel", "rfWheel", "lbWheel", "rbWheel"};
   private static final String[] posNames = {"Zero", "Plus90", "Minus90"};
   private double[][] servoPositions = {
      {LF_ZERO, LF_ZERO + DEF_90STEP, LF_ZERO - DEF_90STEP},   // lfSteerServo
      {RF_ZERO, RF_ZERO + DEF_90STEP, RF_ZERO - DEF_90STEP},   // rfSteerServo
      {LB_ZERO, LB_ZERO + DEF_90STEP, LB_ZERO - DEF_90STEP},   // lbSteerServo
      {RB_ZERO, RB_ZERO + DEF_90STEP, RB_ZERO - DEF_90STEP}    // rbSteerServo
   };
   private double stepSize = 0.01;
   private int wheelIndex = 0;
   private int posIndex = 0;

   // A, B, DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT
   private boolean[] buttonWasPressed = {false, false, false, false, false, false};

   // Declare OpMode members.
   private ElapsedTime runtime = new ElapsedTime();
   private Servo lfSteerServo1, lfSteerServo2, rfSteerServo1, rfSteerServo2;
   private Servo lbSteerServo1, lbSteerServo2, rbSteerServo1, rbSteerServo2;

   private void setServoPosition()
   {
      lfSteerServo1.setPosition(servoPositions[0][posIndex]);
      lfSteerServo2.setPosition(servoPositions[0][posIndex]);
      rfSteerServo1.setPosition(servoPositions[1][posIndex]);
      rfSteerServo2.setPosition(servoPositions[1][posIndex]);
      lbSteerServo1.setPosition(servoPositions[2][posIndex]);
      lbSteerServo2.setPosition(servoPositions[2][posIndex]);
      rbSteerServo1.setPosition(servoPositions[3][posIndex]);
      rbSteerServo2.setPosition(servoPositions[3][posIndex]);
   }  //setServoPosition

   private boolean buttonPressed(int index, boolean buttonState)
   {
      boolean buttonEvent = !buttonWasPressed[index] && buttonState || buttonWasPressed[index] && !buttonState;

      if (buttonEvent) buttonWasPressed[index] = buttonState;

      return buttonEvent && buttonState;
   }  //buttonPressed

   @Override
   public void runOpMode() {
      telemetry.addData("Status", "Initialized");
      telemetry.update();

      // Initialize the hardware variables. Note that the strings used here as parameters
      // to 'get' must correspond to the names assigned during the robot configuration
      // step (using the FTC Robot Controller app on the phone).
      lfSteerServo1 = hardwareMap.get(Servo.class, "lfSteerServo1");
      lfSteerServo2 = hardwareMap.get(Servo.class, "lfSteerServo2");
      rfSteerServo1 = hardwareMap.get(Servo.class, "rfSteerServo1");
      rfSteerServo2 = hardwareMap.get(Servo.class, "rfSteerServo2");
      lbSteerServo1 = hardwareMap.get(Servo.class, "lbSteerServo1");
      lbSteerServo2 = hardwareMap.get(Servo.class, "lbSteerServo2");
      rbSteerServo1 = hardwareMap.get(Servo.class, "rbSteerServo1");
      rbSteerServo2 = hardwareMap.get(Servo.class, "rbSteerServo2");
      setServoPosition();

      // Wait for the game to start (driver presses PLAY)
      waitForStart();
      runtime.reset();

      // run until the end of the match (driver presses STOP)
      while (opModeIsActive()) {

         if (buttonPressed(0, gamepad1.a))
         {
            posIndex = (posIndex + 1) % servoPositions[0].length;
         }

         if (buttonPressed(1, gamepad1.b))
         {
            wheelIndex = (wheelIndex + 1) % servoPositions.length;
         }

         if (buttonPressed(2, gamepad1.dpad_up))
         {
            if (servoPositions[wheelIndex][posIndex] + stepSize <= 1.0)
            {
               servoPositions[wheelIndex][posIndex] += stepSize;
            }
         }

         if (buttonPressed(3, gamepad1.dpad_down))
         {
            if (servoPositions[wheelIndex][posIndex] - stepSize >= 0.0)
            {
               servoPositions[wheelIndex][posIndex] -= stepSize;
            }
         }

         if (buttonPressed(4, gamepad1.dpad_left))
         {
            if (stepSize * 10.0 <= 0.1)
            {
               stepSize *= 10.0;
            }
         }

         if (buttonPressed(5, gamepad1.dpad_right))
         {
            if (stepSize / 10.0 >= 0.001)
            {
               stepSize /= 10.0;
            }
         }

         setServoPosition();

         // Show the elapsed game time and wheel power.
         telemetry.addData("Status", "Run Time: " + runtime.toString());
         telemetry.addData(
             "Selections", "Wheel: %s, Pos: %s, Step: %.3f",
             wheelNames[wheelIndex], posNames[posIndex], stepSize);
         telemetry.addData(
             "FrontServos", "lfPos=%.3f, rfPos=%.3f", servoPositions[0][posIndex], servoPositions[1][posIndex]);
         telemetry.addData(
             "BackServos", "lbPos=%.3f, rbPos=%.3f", servoPositions[2][posIndex], servoPositions[3][posIndex]);
         telemetry.update();
      }
   }
}
