using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using ZedGraph;

namespace PCDebugger
{
	public partial class FormMonitor : Form
	{
		public FormMonitor()
		{
			InitializeComponent();
		}

		private Thread _monitorThread;
		private readonly Stopwatch _timer = new Stopwatch();
		private bool _firstResult = true;
		private void FormMonitor_Load(object sender, EventArgs e)
		{
			var gyroPane = _graphAngularVelocity.GraphPane;
			gyroPane.Title.Text = "Vinkelhastighed";
			gyroPane.XAxis.Title.Text = "Tid [s]";
			gyroPane.YAxis.Title.Text = "Vinkelhastighed [grader/s]";
			gyroPane.YAxis.MajorGrid.IsZeroLine = false;
			gyroPane.AddCurve("Gyroskop", _gyroPoints, Color.Red, SymbolType.None);
			gyroPane.XAxis.Scale.Min = 0;
			gyroPane.XAxis.Scale.Max = 30;
			gyroPane.XAxis.Scale.MinorStep = 1;
			gyroPane.XAxis.Scale.MajorStep = 2;

			gyroPane.YAxis.Scale.Min = -250;
			gyroPane.YAxis.Scale.Max = 250;
			_graphAngularVelocity.AxisChange();
			_graphAngularVelocity.Invalidate();

			var accelPane = _graphAccelAngle.GraphPane;
			accelPane.Title.Text = "Accelerometer vinkel";
			accelPane.XAxis.Title.Text = "Tid [s]";
			accelPane.YAxis.Title.Text = "Vinkel [grader]";
			accelPane.YAxis.MajorGrid.IsZeroLine = false;
			accelPane.AddCurve("Accelerometer", _accelPoints, Color.Blue, SymbolType.None);
			accelPane.XAxis.Scale.Min = 0;
			accelPane.XAxis.Scale.Max = 30;
			accelPane.XAxis.Scale.MinorStep = 1;
			accelPane.XAxis.Scale.MajorStep = 2;

			accelPane.YAxis.Scale.Min = -100;
			accelPane.YAxis.Scale.Max = 100;
			_graphAccelAngle.AxisChange();
			_graphAccelAngle.Invalidate();

			var compPane = _graphAngle.GraphPane;
			compPane.Title.Text = "Komplementær vinkel";
			compPane.XAxis.Title.Text = "Tid [s]";
			compPane.YAxis.Title.Text = "Vinkel [grader]";
			compPane.YAxis.MajorGrid.IsZeroLine = false;
			compPane.AddCurve("Vinkel", _anglePoints, Color.Green, SymbolType.None);
			compPane.XAxis.Scale.Min = 0;
			compPane.XAxis.Scale.Max = 30;
			compPane.XAxis.Scale.MinorStep = 1;
			compPane.XAxis.Scale.MajorStep = 2;

			compPane.YAxis.Scale.Min = -100;
			compPane.YAxis.Scale.Max = 100;
			_graphAngle.AxisChange();
			_graphAngle.Invalidate();

			_monitorThread = new Thread(Monitor);
			_monitorThread.Start();
		}

		private readonly RollingPointPairList _gyroPoints = new RollingPointPairList(500);
		private readonly RollingPointPairList _accelPoints = new RollingPointPairList(500);
		private readonly RollingPointPairList _anglePoints = new RollingPointPairList(500);
		private void AddResult(float accel, float gyro, float angle)
		{
			float t;
			if (_firstResult)
			{
				t = 0;
				_timer.Start();
				_firstResult = false;
			}
			else
			{
				t = (float)_timer.Elapsed.TotalSeconds;
			}

			_gyroPoints.Add(new PointPair(t, gyro));
			_accelPoints.Add(new PointPair(t, accel));
			_anglePoints.Add(new PointPair(t, angle));

			foreach (ZedGraphControl graph in new[] { _graphAngularVelocity, _graphAccelAngle, _graphAngle })
			{
				if (t > graph.GraphPane.XAxis.Scale.Max - _graphAngularVelocity.GraphPane.XAxis.Scale.MajorStep)
				{
					graph.GraphPane.XAxis.Scale.Min = t - 30;
					graph.GraphPane.XAxis.Scale.Max = t + _graphAngularVelocity.GraphPane.XAxis.Scale.MajorStep;
				}

				graph.AxisChange();
				graph.Invalidate();
			}
		}

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

		private void Monitor()
		{
			string portName = "COM38";
			if (File.Exists("Com port.txt"))
				portName = File.ReadAllText("COM port.txt");

			using (SerialPort port = new SerialPort(portName, 115200))
			{
				port.Open();

				BinaryReader br = new BinaryReader(port.BaseStream);

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

					try
					{Invoke(new Action<float, float, float>(AddResult), accelAngle, angularVelocity, angle);
					}
					catch (InvalidOperationException)
					{
						break;
					}
					catch (FormatException)
					{
						Thread.Sleep(1000);
					}
				}
			}
		}
	}
}
