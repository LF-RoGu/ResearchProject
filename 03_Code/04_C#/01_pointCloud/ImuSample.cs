public class ImuSample
{
    public int FrameId { get; set; }
    public int Index { get; set; }
    public float[] Quaternion { get; set; } = new float[4];
    public float[] Accel { get; set; } = new float[3];
    public int PacketCounter { get; set; }
}
