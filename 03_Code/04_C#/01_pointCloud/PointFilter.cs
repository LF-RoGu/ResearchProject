using System.Collections.Generic;
using System.Linq;

public static class PointFilter
{
    public static IEnumerable<RadarPoint> FilterSNRmin(IEnumerable<RadarPoint> pts, int minSNR)
        => pts.Where(p => p.SNR >= minSNR);

    public static IEnumerable<RadarPoint> FilterZ(IEnumerable<RadarPoint> pts, float minZ, float maxZ)
        => pts.Where(p => p.Z >= minZ && p.Z <= maxZ);

    public static IEnumerable<RadarPoint> FilterY(IEnumerable<RadarPoint> pts, float minY, float maxY)
        => pts.Where(p => p.Y >= minY && p.Y <= maxY);

    public static IEnumerable<RadarPoint> FilterPhi(IEnumerable<RadarPoint> pts, float minPhi, float maxPhi)
    {
        return pts.Where(p =>
        {
            var phi = Math.Atan2(p.Y, p.X) * 180 / Math.PI;
            return phi >= minPhi && phi <= maxPhi;
        });
    }

    public static IEnumerable<RadarPoint> FilterDoppler(IEnumerable<RadarPoint> pts, float minDop, float maxDop)
        => pts.Where(p => p.Doppler >= minDop && p.Doppler <= maxDop);
}
