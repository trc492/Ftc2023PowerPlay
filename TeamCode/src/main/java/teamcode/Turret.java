/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcFtcLib.ftclib.FtcMotorActuator;

public class Turret
{
    //state machine if we need it in the future
    private enum State
    {

        DONE
    }   //enum ZeroCalStates


    private final Robot robot;
    private final TrcPidActuator pidTurret;

    private final TrcEvent armEvent;
    private final TrcEvent elevatorEvent;
    private final TrcEvent turretEvent;
    private final TrcStateMachine<State> sm;
    private double turretTargetPosition;

    //state machine where both setPosition and setPower use it. check if elevator and arm below level, otherwise move it up above safe level
    //once you are above elevator and arm, then get out of the safemachine

    public Turret(Robot robot)
    {
        this.robot = robot;

        final FtcMotorActuator.MotorParams motorParams = new FtcMotorActuator.MotorParams(
            RobotParams.TURRET_MOTOR_INVERTED,
            RobotParams.TURRET_HAS_LOWER_LIMIT_SWITCH, RobotParams.TURRET_LOWER_LIMIT_INVERTED,
            RobotParams.TURRET_HAS_UPPER_LIMIT_SWITCH, RobotParams.TURRET_UPPER_LIMIT_INVERTED);
        final TrcPidActuator.Parameters turretParams = new TrcPidActuator.Parameters()
            .setPosRange(RobotParams.TURRET_MIN_POS, RobotParams.TURRET_MAX_POS)
            .setScaleOffset(RobotParams.TURRET_DEG_PER_COUNT, RobotParams.TURRET_OFFSET)
            .setPidParams(new TrcPidController.PidParameters(
                RobotParams.TURRET_KP, RobotParams.TURRET_KI, RobotParams.TURRET_KD,
                RobotParams.TURRET_TOLERANCE))
            .setStallProtectionParams(
                RobotParams.TURRET_STALL_MIN_POWER, RobotParams.TURRET_STALL_TOLERANCE,
                RobotParams.TURRET_STALL_TIMEOUT, RobotParams.TURRET_RESET_TIMEOUT)
            .setZeroCalibratePower(RobotParams.TURRET_CAL_POWER)
            .setPosPresets(RobotParams.TURRET_PRESET_LEVELS);
        pidTurret = new FtcMotorActuator(
            RobotParams.HWNAME_TURRET, motorParams, turretParams).getPidActuator();

        armEvent = new TrcEvent("armEvent");
        elevatorEvent = new TrcEvent("elevatorEvent");
        turretEvent = new TrcEvent("turretEvent");
        sm = new TrcStateMachine<>("turret state machine");


    }   //Turret

    // Since there are only 3 steps, we should use notifier callback instead. In this method, we should:
    // 1. zero calibrate the elevator.
    // 2. zero calibrate the arm and chain to a armZeroCalDone callback.
    // 3. In the armZeroCalDone callback, do a fire and forget zero calibrate on the turret.
    public void zeroCalibrate()
    {
        robot.elevator.zeroCalibrate();
        robot.arm.zeroCalibrate(RobotParams.ARM_CAL_POWER, this::armZeroCalDoneCallback);

    }
    public void armZeroCalDoneCallback(Object context){
        robot.turret.zeroCalibrate();
    }
    //before it calls, setTarget, check if arm above a certain threshold
    //if they are below threshold, raise them before turning
    public void setTarget(double position)
    {
//        if(elevator.getPosition() < Robotparams.)
        if (robot.arm.getPosition() >= RobotParams.ARM_MIN_POS_FOR_TURRET)
        {
            // We are safe to turn the turret, do it.
            pidTurret.setTarget(position);
        }
        else
        {
            turretTargetPosition = position;
            robot.arm.setTarget(RobotParams.ARM_MIN_POS_FOR_TURRET, false, 1.0, null, this::armRaiseDoneCallback );
        }
    }
    //if arm isn't high enough to set power to turret, just set the arm target, no callback for doing turret power
    public void setPower(double power){
        if (robot.arm.getPosition() < RobotParams.ARM_MIN_POS_FOR_TURRET){
            robot.arm.setTarget(RobotParams.ARM_MIN_POS_FOR_TURRET);
        }
        else{
            pidTurret.setPower(power);
        }
    }
    public void armRaiseDoneCallback(Object context){
        pidTurret.setTarget(turretTargetPosition);
        turretTargetPosition = 0.0;
    }

}
