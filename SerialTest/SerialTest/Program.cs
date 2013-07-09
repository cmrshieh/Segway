using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace SerialTest
{
	class Program
	{
		private static bool FindDataStart(BinaryReader br)
		{
			byte[] bytes = Encoding.ASCII.GetBytes("Data");

			for (int i = 0; i < bytes.Length; i++)
			{
				byte b = br.ReadByte();
				if (b != bytes[i])
				{
					if (b == bytes[0])
					{
						i = 0;
						continue; // Continue as if we received 'A'
					}

					return false;
				}
			}

			return true;
		}

		static void Main(string[] args)
		{
			Console.Write("Please enter COM port: ");
			string com = Console.ReadLine();

			using (SerialPort sp = new SerialPort(com, 115200))
			{
				sp.Open();

				BinaryReader br = new BinaryReader(sp.BaseStream);

				while (true)
				{
					while (true)
					{
						if (FindDataStart(br))
							break;
					}

					float accelAngle = br.ReadSingle();
					float angularVelocity = br.ReadSingle();
					float angle = br.ReadSingle();
					short motorSpeed = br.ReadInt16();
					float kp = br.ReadSingle();
					float ki = br.ReadSingle();
					float kd = br.ReadSingle();

					ushort sensorReadTime = br.ReadUInt16();
					ushort angleTime = br.ReadUInt16();
					ushort regulateMotorTime = br.ReadUInt16();
					ushort printDebugInfoTime = br.ReadUInt16();
					ushort totalTime = br.ReadUInt16();
					bool stalled = br.ReadInt16() != 0;

					Console.WriteLine("Regulation");
					Console.WriteLine("Accel angle: {0}", accelAngle);
					Console.WriteLine("Angular velocity: {0}", angularVelocity);
					Console.WriteLine("Angle: {0}", angle);
					Console.WriteLine("Motor speed: {0}", motorSpeed);
					Console.WriteLine("Kp: {0}", kp);
					Console.WriteLine("Ki: {0}", ki);
					Console.WriteLine("Kd: {0}", kd);

					Console.WriteLine("Timings");
					Console.WriteLine("Sensor read time: {0}", sensorReadTime);
					Console.WriteLine("Angle time: {0}", angleTime);
					Console.WriteLine("Regulate motor time: {0}", regulateMotorTime);
					Console.WriteLine("Print debug info time: {0}", printDebugInfoTime);
					Console.WriteLine("Total time: {0}", totalTime);
					Console.WriteLine("Stalled: {0}", stalled);

					if (Console.KeyAvailable)
					{
						string line = Console.ReadLine();
						string[] split = line.Split(' ');
						float newKp, newKi, newKd;
						if (split.Length < 3 || !float.TryParse(split[0], NumberStyles.Float, CultureInfo.InvariantCulture, out newKp)
							 || !float.TryParse(split[1], NumberStyles.Float, CultureInfo.InvariantCulture, out newKi)
							 || !float.TryParse(split[2], NumberStyles.Float, CultureInfo.InvariantCulture, out newKd))
						{
							Console.WriteLine("Invalid");

						}
						else
						{
							MemoryStream ms = new MemoryStream();
							using (BinaryWriter bw = new BinaryWriter(ms))
							{
								bw.Write(new byte[] { (byte)'K', (byte)'S', (byte)'E', (byte)'G' });
								bw.Write(newKp);
								bw.Write(newKi);
								bw.Write(newKd);

								byte[] bytes = ms.ToArray();
								sp.Write(bytes, 0, bytes.Length);
							}
						}
					}
				}
			}
		}
	}
}