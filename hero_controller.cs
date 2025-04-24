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

        static float currentV = 0;
        static float currentW = 0;

        // Smoothed values and smoothing factor
        static float smoothedV = 0.0f;
        static float smoothedW = 0.0f;
        static float alpha = 0.3f;  // 0.3 = medium smoothing

        public static void Main()
        {
            byte[] data = new byte[1024];
            int bytesReceived = 0;

            while (true)
            {
                if (_gamepad == null)
                    _gamepad = new GameController(UsbHostDevice.GetInstance());

                if (!_gamepad.GetButton(6))
                {
                    // Manual control via joystick
                    if (_uart != null)
                    {
                        _uart.Flush();
                        _uart.Close();
                        _uart = null;
                    }

                    float v = -1 * _gamepad.GetAxis(1);
                    float w = _gamepad.GetAxis(2);
                    Drive(v, w);  // immediate response for manual mode
                }
                else
                {
                    // UART control mode
                    if (_uart == null)
                    {
                        _uart = new SerialPort(CTRE.HERO.IO.Port1.UART, 9600);
                        _uart.Open();
                        uartBuffer.Clear();
                    }

                    bytesReceived = _uart.Read(data, 0, data.Length);
                    for (int i = 0; i < bytesReceived; i++)
                    {
                        char c = (char)data[i];
                        uartBuffer.Append(c);

                        if (c == '#')  // full message end
                        {
                            string msg = uartBuffer.ToString();
                            uartBuffer.Clear();

                            int startIdx = -1;
                            int midIdx = -1;
                            int endIdx = msg.Length - 1;

                            for (int j = 0; j < msg.Length; j++)
                            {
                                if (msg[j] == '!')
                                    startIdx = j;
                                else if (msg[j] == '@')
                                    midIdx = j;
                            }

                            if (startIdx != -1 && midIdx != -1 && midIdx > startIdx && endIdx > midIdx)
                            {
                                try
                                {
                                    string vStr = msg.Substring(startIdx + 1, midIdx - startIdx - 1);
                                    string wStr = msg.Substring(midIdx + 1, endIdx - midIdx - 1);

                                    float v = (float)Convert.ToDouble(vStr);
                                    float w = (float)Convert.ToDouble(wStr);

                                    currentV = v;
                                    currentW = w;

                                    Debug.Print("Received v=" + v + ", w=" + w);
                                }
                                catch (Exception)
                                {
                                    Debug.Print("UART parse error");
                                }
                            }
                        }
                    }

                    // Always drive with the most recent command (smoothed)
                    Drive(currentV, currentW);
                }

                CTRE.Phoenix.Watchdog.Feed();
                Thread.Sleep(20);  // ~50Hz control loop
            }
        }

        static void Drive(float v, float w)
        {
            // Apply exponential smoothing
            smoothedV = alpha * v + (1 - alpha) * smoothedV;
            smoothedW = alpha * w + (1 - alpha) * smoothedW;

            float leftThrot = smoothedV + smoothedW;
            float rightThrot = smoothedV - smoothedW;

            // Optionally clamp
            leftThrot = Clamp(leftThrot, -1.0f, 1.0f);
            rightThrot = Clamp(rightThrot, -1.0f, 1.0f);

            left.Set(ControlMode.PercentOutput, leftThrot);
            leftSlave.Set(ControlMode.PercentOutput, leftThrot);
            right.Set(ControlMode.PercentOutput, -rightThrot);
            rightSlave.Set(ControlMode.PercentOutput, -rightThrot);
        }

        static float Clamp(float value, float min, float max)
        {
            if (value < min) return min;
            if (value > max) return max;
            return value;
        }
    }
}



