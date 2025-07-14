public class RadarPoint
{
    public int FrameId { get; set; }
    public int PointId { get; set; }
    public float X { get; set; }
    public float Y { get; set; }
    public float Z { get; set; }
    public float Doppler { get; set; }
    public int SNR { get; set; }
    public int Noise { get; set; }
}
