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

import TrcCommonLib.trclib.TrcAnalogSensorTrigger;
import TrcCommonLib.trclib.TrcIntake;
import TrcFtcLib.ftclib.FtcDcMotor;
import TrcFtcLib.ftclib.FtcDistanceSensor;

class Intake
{
    private final TrcIntake.Parameters params;
    private final TrcIntake motorIntake;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param params specifies the parameters for the Intake subsystem.
     */
    public Intake(String instanceName, TrcIntake.Parameters params)
    {
        this.params = params;
        FtcDcMotor motor = new FtcDcMotor(instanceName + ".motor");
        TrcAnalogSensorTrigger<FtcDistanceSensor.DataType> analogTrigger = null;

        if (RobotParams.Preferences.hasGrabberSensor)
        {
            FtcDistanceSensor sensor = new FtcDistanceSensor(instanceName + ".sensor");
            analogTrigger = new TrcAnalogSensorTrigger<>(
                    instanceName + ".analogTrigger", sensor, 0, FtcDistanceSensor.DataType.DISTANCE_CM,
                    new double[]{params.analogThreshold}, this::analogTriggerEvent, false);
        }

        motorIntake = new TrcIntake(instanceName, motor, params, analogTrigger);
    }   //Intake

    /**
     * This method returns the TrcIntake object.
     *
     * @return TrcIntake object.
     */
    public TrcIntake getMotorIntake()
    {
        return motorIntake;
    }   //getMotorIntake

    /**
     * This method is called when an analog sensor threshold has been crossed.
     *
     * @param currZone specifies the zone it is going into.
     * @param prevZone specifies the zone it is coming out of.
     * @param value specifies the actual sensor value.
     */
    private void analogTriggerEvent(int currZone, int prevZone, double value)
    {
        final String funcName = "analogTriggerEvent";

        if (params.msgTracer != null)
        {
            params.msgTracer.traceInfo(funcName, "Zone=%d->%d, value=%.3f", prevZone, currZone, value);
        }

        if (motorIntake.isAutoAssistActive() && prevZone != -1)
        {
            if (params.msgTracer != null)
            {
                params.msgTracer.traceInfo(funcName, "Trigger: hasObject=%s", motorIntake.hasObject());
            }

            motorIntake.finishAutoAssist(null);
        }
    }   //analogTriggerEvent

}   //class Intake