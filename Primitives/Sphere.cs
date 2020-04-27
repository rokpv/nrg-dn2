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

      // TODO: Compute quadratic sphere coefficients
      // TODO: Initialize _double_ ray coordinate values
      var ox = ray.o.x;
      var oy = ray.o.y;
      var oz = ray.o.z;

      var dx = ray.d.x;
      var dy = ray.d.y;
      var dz = ray.d.z;
      
      var a = dx * dx + dy * dy + dz * dz;
      var b = 2 * (dx * ox + dy * oy + dz * oz);
      var c = ox * ox + oy * oy + oz * oz - Radius * Radius;
      
      // TODO: Solve quadratic equation for _t_ values
      var (solvable, t0, t1) = Utils.Quadratic(a, b, c);
      if (!solvable) return (null, null);
      // TODO: Check quadric shape _t0_ and _t1_ for nearest intersection
      var shapeHit = t0 <= 0 ? t1 : t0;
      
      // TODO: Compute sphere hit position and $\phi$
      var hit = ray.Point(shapeHit);
      hit *= Radius / hit.Length();

      if (Math.Abs(hit.x) < Double.Epsilon && Math.Abs(hit.y) < double.Epsilon)
      {
        hit.x = 1e-5 * Radius;
      }

      var phi = Math.Atan2(hit.y, hit.x);
      var theta = Math.Acos((hit.z / Radius).Clamp(-1, 1));
      if (phi < 0) phi += 2 * Math.PI;
      
      // TODO: Return shape hit and surface interaction
      var zRadius = Math.Sqrt(hit.x * hit.x + hit.y * hit.y);
      var invZRadius = 1 / zRadius;
      var cosPhi = hit.x * invZRadius;
      var sinPhi = hit.y * invZRadius;
      
      var dpdu = new Vector3(-phi * hit.y, phi * hit.x, 0);
      var dpdv = new Vector3(hit.z * cosPhi, hit.z * sinPhi, -Radius * Math.Sin(theta));
      var normal = Vector3.Cross(dpdu, dpdv).Normalize();
      
      var interaction = new SurfaceInteraction(hit, normal, -ray.d, dpdu, this);
      return (shapeHit, interaction);
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
      var radiusSq = Radius * Radius;
      var center = ObjectToWorld.ApplyPoint(new Vector3(0, 0, 0));
      var origin = si.Point;
      var diff = origin - center;
      if (diff.LengthSquared() <= radiusSq)
      {
        return base.Pdf(si, wi);
      }

      // TODO: change
      return base.Pdf(si, wi);
    }

  }
}
