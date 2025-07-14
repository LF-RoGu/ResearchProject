using System;
using System.IO;
using System.Net.Sockets;
using System.Threading.Tasks;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;

namespace _01_pointCloud
{
    public partial class Form1 : Form
    {
        private TcpClient tcpClient;
        private StreamReader reader;

        private FrameAggregator aggregator = new(10);
        private List<ImuSample> imuSamples = new();
        private readonly object lockObj = new();

        public Form1()
        {
            InitializeComponent();
            ConnectToServer();
        }

        private void ConnectToServer()
        {
            try
            {
                tcpClient = new TcpClient();
                tcpClient.Connect("192.168.63.97", 9000);
                reader = new StreamReader(tcpClient.GetStream());
                Task.Run(ReadLoop);
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Connection error: {ex.Message}");
            }
        }

        private async Task ReadLoop()
        {
            var currentFrame = new List<RadarPoint>();

            while (true)
            {
                string? line = await reader.ReadLineAsync();
                if (line == null) continue;

                if (line.StartsWith("[RADAR]"))
                {
                    var pt = ParseRadar(line);
                    currentFrame.Add(pt);
                }
                else if (line.StartsWith("[IMU]"))
                {
                    var imu = ParseImu(line);
                    lock (lockObj)
                    {
                        imuSamples.Add(imu);
                        if (imuSamples.Count > 20) imuSamples.RemoveAt(0);
                    }
                }

                if (currentFrame.Count >= 5)
                {
                    aggregator.AddFrame(new List<RadarPoint>(currentFrame));
                    currentFrame.Clear();
                }

                await Task.Delay(10);
            }
        }

        private RadarPoint ParseRadar(string line)
        {
            var pt = new RadarPoint();
            foreach (var token in line.Split(' '))
            {
                if (token.StartsWith("frame=")) pt.FrameId = int.Parse(token[6..]);
                else if (token.StartsWith("pt=")) pt.PointId = int.Parse(token[3..]);
                else if (token.StartsWith("x=")) pt.X = float.Parse(token[2..]);
                else if (token.StartsWith("y=")) pt.Y = float.Parse(token[2..]);
                else if (token.StartsWith("z=")) pt.Z = float.Parse(token[2..]);
                else if (token.StartsWith("dop=")) pt.Doppler = float.Parse(token[4..]);
                else if (token.StartsWith("snr=")) pt.SNR = int.Parse(token[4..]);
                else if (token.StartsWith("noise=")) pt.Noise = int.Parse(token[6..]);
            }
            return pt;
        }

        private ImuSample ParseImu(string line)
        {
            var imu = new ImuSample();
            imu.Quaternion = new float[4];
            foreach (var token in line.Split(' '))
            {
                if (token.StartsWith("frame=")) imu.FrameId = int.Parse(token[6..]);
                else if (token.StartsWith("idx=")) imu.Index = int.Parse(token[4..]);
                else if (token.StartsWith("quat=("))
                {
                    var q = token.Substring(6).TrimEnd(')');
                    var parts = q.Split(',');
                    for (int i = 0; i < 4; i++)
                        imu.Quaternion[i] = float.Parse(parts[i]);
                }
            }
            return imu;
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            panelRaw.Invalidate();
            panelImu.Invalidate();
        }

        private void panelRaw_Paint(object sender, PaintEventArgs e)
        {
            lock (lockObj)
            {
                foreach (var p in aggregator.GetAllPoints())
                {
                    float scale = 20.0f;
                    float cx = 200;
                    float cy = 200;
                    float x = cx + p.X * scale;
                    float y = cy - p.Y * scale;

                    e.Graphics.FillEllipse(Brushes.Gray, x, y, 4, 4);
                    e.Graphics.DrawString($"{p.Doppler:F2}",
                        SystemFonts.DefaultFont, Brushes.Black, x + 5, y);
                }
            }
        }

        private void panelImu_Paint(object sender, PaintEventArgs e)
        {
            lock (lockObj)
            {
                foreach (var imu in imuSamples)
                {
                    var (vx, vy) = GetYawVector(imu.Quaternion);
                    DrawImuVector(e.Graphics, vx, vy);
                }
            }
        }

        private (float vx, float vy) GetYawVector(float[] q)
        {
            float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
            float yaw = (float)Math.Atan2(2.0 * (q0 * q3 + q1 * q2),
                                          1.0 - 2.0 * (q2 * q2 + q3 * q3));
            float vx = (float)Math.Cos(yaw);
            float vy = (float)Math.Sin(yaw);
            return (vx, vy);
        }

        private void DrawImuVector(Graphics g, float vx, float vy)
        {
            float scale = 50.0f;
            float cx = 200;
            float cy = 100;
            float x2 = cx + vx * scale;
            float y2 = cy - vy * scale;

            g.DrawLine(Pens.Red, cx, cy, x2, y2);
            g.FillEllipse(Brushes.Red, x2 - 3, y2 - 3, 6, 6);
        }
    }
}
