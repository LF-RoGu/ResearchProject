namespace _01_pointCloud;

partial class Form1
{
    private System.ComponentModel.IContainer components = null;

    private System.Windows.Forms.TextBox textBoxOutput;

    protected override void Dispose(bool disposing)
    {
        if (disposing && (components != null))
        {
            components.Dispose();
        }
        base.Dispose(disposing);
    }

    #region Windows Form Designer generated code

    private void InitializeComponent()
    {
        this.textBoxOutput = new System.Windows.Forms.TextBox();
        this.SuspendLayout();
        // 
        // textBoxOutput
        // 
        this.textBoxOutput.Location = new System.Drawing.Point(12, 12);
        this.textBoxOutput.Multiline = true;
        this.textBoxOutput.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
        this.textBoxOutput.Size = new System.Drawing.Size(776, 426);
        this.textBoxOutput.Name = "textBoxOutput";
        // 
        // Form1
        // 
        this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
        this.ClientSize = new System.Drawing.Size(800, 450);
        this.Controls.Add(this.textBoxOutput);
        this.Name = "Form1";
        this.Text = "PointCloud TCP Debug";
        this.ResumeLayout(false);
        this.PerformLayout();
    }

    #endregion
}
