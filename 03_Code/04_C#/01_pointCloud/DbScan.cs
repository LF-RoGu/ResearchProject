using System;
using System.Collections.Generic;
using System.Linq;

public static class DbScan
{
    public static List<List<RadarPoint>> Cluster(List<RadarPoint> points, float eps, int minPts)
    {
        var clusters = new List<List<RadarPoint>>();
        var visited = new HashSet<RadarPoint>();
        var noise = new List<RadarPoint>();

        foreach (var point in points)
        {
            if (visited.Contains(point)) continue;

            visited.Add(point);
            var neighbors = RegionQuery(points, point, eps);

            if (neighbors.Count < minPts)
            {
                noise.Add(point);
            }
            else
            {
                var cluster = new List<RadarPoint>();
                ExpandCluster(point, neighbors, cluster, points, visited, eps, minPts);
                clusters.Add(cluster);
            }
        }

        return clusters;
    }

    private static void ExpandCluster(
        RadarPoint point,
        List<RadarPoint> neighbors,
        List<RadarPoint> cluster,
        List<RadarPoint> points,
        HashSet<RadarPoint> visited,
        float eps,
        int minPts)
    {
        cluster.Add(point);

        for (int i = 0; i < neighbors.Count; i++)
        {
            var p = neighbors[i];
            if (!visited.Contains(p))
            {
                visited.Add(p);
                var neighbors2 = RegionQuery(points, p, eps);
                if (neighbors2.Count >= minPts)
                {
                    neighbors.AddRange(neighbors2.Where(n => !neighbors.Contains(n)));
                }
            }

            if (!cluster.Contains(p))
            {
                cluster.Add(p);
            }
        }
    }

    private static List<RadarPoint> RegionQuery(List<RadarPoint> points, RadarPoint point, float eps)
    {
        var neighbors = new List<RadarPoint>();
        foreach (var p in points)
        {
            if (Distance(point, p) <= eps)
            {
                neighbors.Add(p);
            }
        }
        return neighbors;
    }

    private static float Distance(RadarPoint p1, RadarPoint p2)
    {
        float dx = p1.X - p2.X;
        float dy = p1.Y - p2.Y;
        return (float)Math.Sqrt(dx * dx + dy * dy);
    }
}
