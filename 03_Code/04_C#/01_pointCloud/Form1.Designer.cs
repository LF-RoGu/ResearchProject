namespace _01_pointCloud;

partial class Form1
{
    private System.ComponentModel.IContainer components = null;

    private System.Windows.Forms.Panel panelRaw;
    private System.Windows.Forms.Panel panelImu;
    private System.Windows.Forms.Timer timer1;

    protected override void Dispose(bool disposing)
    {
        if (disposing && (components != null)) components.Dispose();
        base.Dispose(disposing);
    }

    private void InitializeComponent()
    {
        this.components = new System.ComponentModel.Container();
        this.panelRaw = new System.Windows.Forms.Panel();
        this.panelImu = new System.Windows.Forms.Panel();
        this.timer1 = new System.Windows.Forms.Timer(this.components);

        // panelRaw
        this.panelRaw.Location = new System.Drawing.Point(20, 20);
        this.panelRaw.Size = new System.Drawing.Size(400, 400);
        this.panelRaw.Name = "panelRaw";
        this.panelRaw.BackColor = System.Drawing.Color.White; // ✅ White background!
        this.panelRaw.Paint += new System.Windows.Forms.PaintEventHandler(this.panelRaw_Paint);

        // panelImu
        this.panelImu.Location = new System.Drawing.Point(20, 440);
        this.panelImu.Size = new System.Drawing.Size(400, 200);
        this.panelImu.Name = "panelImu";
        this.panelImu.BackColor = System.Drawing.Color.White; // ✅ White background!
        this.panelImu.Paint += new System.Windows.Forms.PaintEventHandler(this.panelImu_Paint);

        // timer1
        this.timer1.Interval = 100; // ~10Hz refresh
        this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
        this.timer1.Start();

        // Form1
        this.ClientSize = new System.Drawing.Size(450, 670);
        this.Controls.Add(this.panelRaw);
        this.Controls.Add(this.panelImu);
        this.Name = "Form1";
        this.Text = "Radar & IMU Viewer";
    }
}
