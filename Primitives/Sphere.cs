using System;
using System.Security.Permissions;
using MathNet.Numerics.Integration;

namespace PathTracer
{
  class Sphere : Shape
  {
    public double Radius { get; set; }
    public Sphere(double radius, Transform objectToWorld)
    {
      Radius = radius;
      ObjectToWorld = objectToWorld;
    }

    public override (double?, SurfaceInteraction) Intersect(Ray r)
    {
      Ray ray = WorldToObject.Apply(r);

      var ox = ray.o.x;
      var oy = ray.o.y;
      var oz = ray.o.z;

      var dx = ray.d.x;
      var dy = ray.d.y;
      var dz = ray.d.z;
      
      var a = dx * dx + dy * dy + dz * dz;
      var b = 2 * (dx * ox + dy * oy + dz * oz);
      var c = ox * ox + oy * oy + oz * oz - Radius * Radius;
      
      var (solvable, t0, t1) = Utils.Quadratic(a, b, c);
      if (!solvable) return (null, null);

      var shapeHit = t0 <= 0 ? t1 : t0;

      var hit = ray.Point(shapeHit);
      hit *= Radius / hit.Length();

      if (Math.Abs(hit.x) < Double.Epsilon && Math.Abs(hit.y) < double.Epsilon)
      {
        hit.x = 1e-5 * Radius;
      }

      var phi = Math.Atan2(hit.y, hit.x);
      if (phi < 0) phi += 2 * Math.PI;

      var dpdu = new Vector3(-phi * hit.y, phi * hit.x, 0);

      var interaction = new SurfaceInteraction(hit, hit, -ray.d, dpdu, this);
      return (shapeHit, ObjectToWorld.Apply(interaction));
    }

    public override (SurfaceInteraction, double) Sample()
    {
      // TODO: Implement Sphere sampling
      var center = new Vector3(0, 0, 0);
      var point = center + Radius * Samplers.UniformSampleSphere();

      var normal = ObjectToWorld.ApplyPoint(point).Normalize();
      point *= Radius / (center - point).Length();
      return (new SurfaceInteraction(point, normal, normal, normal, this), 0);
    }

    public override double Area() { return 4 * Math.PI * Radius * Radius; }

    public override double Pdf(SurfaceInteraction si, Vector3 wi)
    {
      return 1 / Area();
    }

  }
}
