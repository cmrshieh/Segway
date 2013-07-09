using System;
using System.Diagnostics;
using System.IO.Ports;
using System.Text;
using System.Threading;

namespace Calibrator
{
	public class Program
	{
		private static void Main(string[] args)
		{
			long[] gyroSums = new long[3];
			long[] accelSums = new long[3];
			int vals = 0;

			Stopwatch outputTimer = Stopwatch.StartNew();

			using (SerialPort port = new SerialPort("COM3", 9600))
			{
				port.NewLine = "\r\n";
				port.Open();
				while (true)
				{
					string header;
					do
					{
						header = port.ReadLine();
					} while (header != "MPU-6050");

					string info = port.ReadLine();
					string accel = port.ReadLine();
					string temperature = port.ReadLine();
					string gyro = port.ReadLine();

					string[] accelSplit = accel.Substring(13).Split(new[] {", "}, StringSplitOptions.None);
					int accelX = int.Parse(accelSplit[0]);
					int accelY = int.Parse(accelSplit[1]);
					int accelZ = int.Parse(accelSplit[2]);

					string[] gyroSplit = gyro.Substring(13).Split(new[] { ", " }, StringSplitOptions.None);
					int gyroX = int.Parse(gyroSplit[0]);
					int gyroY = int.Parse(gyroSplit[1]);
					int gyroZ = int.Parse(gyroSplit[2]);

					vals++;

					accelSums[0] += accelX;
					accelSums[1] += accelY;
					accelSums[2] += accelZ;

					gyroSums[0] += gyroX;
					gyroSums[1] += gyroY;
					gyroSums[2] += gyroZ;


					if (outputTimer.ElapsedMilliseconds <= 5000)
						continue;

					Console.WriteLine("Accel averages:");
					Console.WriteLine("X: {0}", Math.Round(accelSums[0] / (double)vals));
					Console.WriteLine("Y: {0}", Math.Round(accelSums[1] / (double)vals));
					Console.WriteLine("Z: {0}", Math.Round(accelSums[2] / (double)vals));

					Console.WriteLine("Gyro averages:");
					Console.WriteLine("X: {0}", Math.Round(gyroSums[0] / (double)vals));
					Console.WriteLine("Y: {0}", Math.Round(gyroSums[1] / (double)vals));
					Console.WriteLine("Z: {0}", Math.Round(gyroSums[2] / (double)vals));

					Console.WriteLine();
					outputTimer.Restart();
				}
			}
		}
	}
}
