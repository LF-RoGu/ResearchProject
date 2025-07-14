using System.Collections.Generic;
using System.Linq;

public class FrameAggregator
{
    private readonly int capacity;
    private readonly Queue<List<RadarPoint>> frames = new();
    private readonly object lockObj = new();

    public FrameAggregator(int capacity = 10)
    {
        this.capacity = capacity;
    }

    public void AddFrame(List<RadarPoint> frame)
    {
        lock (lockObj)
        {
            frames.Enqueue(frame);
            if (frames.Count > capacity)
                frames.Dequeue();
        }
    }

    public List<RadarPoint> GetAllPoints()
    {
        lock (lockObj)
        {
            return frames.SelectMany(f => f).ToList();
        }
    }
}
