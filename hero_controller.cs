using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;
using Microsoft.SPOT;
using System;
using System.IO.Ports;
using System.Text;
using System.Threading;

namespace HERO_UART_Smooth_Control
{
    public class Program
    {
        static TalonSRX rightSlave = new TalonSRX(4);
        static TalonSRX right = new TalonSRX(3);
        static TalonSRX leftSlave = new TalonSRX(2);
        static TalonSRX left = new TalonSRX(1);

        static StringBuilder uartBuffer = new StringBuilder();
        static GameController _gamepad = null;
        static SerialPort _uart = null;

        static float targetV = 0.0f;
        static float targetW = 0.0f;
        static float smoothedV = 0.0f;
        static float smoothedW = 0.0f;

        static float alpha = 0.6f;              // faster smoothing (was 0.3)
        static float maxChangePerLoop = 0.15f;   // faster max velocity delta (was 0.05)

        public static void Main()
        {
            byte[] data = new byte[1024];
            int bytesReceived = 0;

            bool useUART = false;
            bool lastButton6State = false;

            while (true)
            {
                if (_gamepad == null)
                    _gamepad = new GameController(UsbHostDevice.GetInstance());

                // Toggle mode on Button 6 press
                bool currentButton6State = _gamepad.GetButton(6);
                if (currentButton6State && !lastButton6State)
                {
                    useUART = !useUART;
                    Debug.Print("Toggled mode: " + (useUART ? "UART" : "Joystick"));

                    if (useUART)
                    {
                        if (_uart == null)
                        {
                            _uart = new SerialPort(CTRE.HERO.IO.Port1.UART, 115200);
                            _uart.Open();
                            uartBuffer.Clear();
                        }
                    }
                    else
                    {
                        if (_uart != null)
                        {
                            _uart.Flush();
                            _uart.Close();
                            _uart = null;
                        }
                    }
                }
                lastButton6State = currentButton6State;

                if (!useUART)
                {
                    // Manual joystick control
                    float v = -1 * _gamepad.GetAxis(1);  // joystick Y-axis needs flip
                    float w = _gamepad.GetAxis(2);       // joystick X-axis normal
                    Drive(v, w, false);                  // drive in joystick mode
                }
                else
                {
                    // UART control mode
                    if (_uart != null && _uart.BytesToRead > 0)
                    {
                        bytesReceived = _uart.Read(data, 0, data.Length);
                        for (int i = 0; i < bytesReceived; i++)
                        {
                            char c = (char)data[i];
                            uartBuffer.Append(c);

                            if (c == '#')
                            {
                                string msg = uartBuffer.ToString();
                                uartBuffer.Clear();

                                int startIdx = msg.LastIndexOf('!');
                                int midIdx = msg.IndexOf('@', startIdx);
                                int endIdx = msg.IndexOf('#', midIdx);

                                if (startIdx != -1 && midIdx != -1 && endIdx != -1)
                                {
                                    try
                                    {
                                        string vStr = msg.Substring(startIdx + 1, midIdx - startIdx - 1);
                                        string wStr = msg.Substring(midIdx + 1, endIdx - midIdx - 1);

                                        float v = (float)Convert.ToDouble(vStr);
                                        float w = (float)Convert.ToDouble(wStr);

                                        targetV = LimitChange(targetV, v, maxChangePerLoop);
                                        targetW = LimitChange(targetW, w, maxChangePerLoop);

                                        Debug.Print("Received v=" + v + ", w=" + w);
                                    }
                                    catch
                                    {
                                        Debug.Print("UART parse error");
                                    }
                                }
                            }

                            if (uartBuffer.Length > 100)
                                uartBuffer.Clear();
                        }
                    }

                    // Always apply smoothing
                    smoothedV = alpha * targetV + (1 - alpha) * smoothedV;
                    smoothedW = alpha * targetW + (1 - alpha) * smoothedW;

                    // ✅ Flip v here for UART
                    Drive(smoothedV, smoothedW, true);
                }

                CTRE.Phoenix.Watchdog.Feed();
                Thread.Sleep(20);  // ~50Hz loop
            }
        }

        static void Drive(float v, float w, bool isUART)
        {
            if (isUART)
                v = -v;  // Flip v direction only for UART control

            float leftThrot = v + w;
            float rightThrot = v - w;

            leftThrot = Clamp(leftThrot, -1.0f, 1.0f);
            rightThrot = Clamp(rightThrot, -1.0f, 1.0f);

            left.Set(ControlMode.PercentOutput, leftThrot);
            leftSlave.Set(ControlMode.PercentOutput, leftThrot);
            right.Set(ControlMode.PercentOutput, -rightThrot);   // Invert right motors
            rightSlave.Set(ControlMode.PercentOutput, -rightThrot);
        }

        static float Clamp(float value, float min, float max)
        {
            if (value < min) return min;
            if (value > max) return max;
            return value;
        }

        static float LimitChange(float current, float target, float maxDelta)
        {
            float delta = target - current;
            if (delta > maxDelta) return current + maxDelta;
            if (delta < -maxDelta) return current - maxDelta;
            return target;
        }
    }
}



