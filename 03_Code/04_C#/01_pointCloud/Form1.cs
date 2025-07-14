using System;
using System.IO;
using System.Net.Sockets;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace _01_pointCloud
{
    public partial class Form1 : Form
    {
        private TcpClient tcpClient;
        private NetworkStream stream;
        private StreamReader reader;

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
                tcpClient.Connect("192.168.63.97", 9000);  // Connect directly to your RPi
                stream = tcpClient.GetStream();
                reader = new StreamReader(stream);

                Task.Run(ReadLoop); // Run reader in background
                textBoxOutput.AppendText("Connected to RPi!\r\n");
            }
            catch (Exception ex)
            {
                textBoxOutput.AppendText($"Connection error: {ex.Message}\r\n");
            }
        }

        private async Task ReadLoop()
        {
            while (true)
            {
                string? line = await reader.ReadLineAsync();
                if (line != null)
                {
                    textBoxOutput.Invoke(() =>
                    {
                        textBoxOutput.AppendText(line + "\r\n");
                    });
                }
                await Task.Delay(100); // 100 ms pause between lines
            }
        }
    }
}
