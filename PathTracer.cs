using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static PathTracer.Samplers;

namespace PathTracer
{
  class PathTracer
  {
    public Spectrum Li(Ray ray, Scene scene)
    {
      var L = Spectrum.ZeroSpectrum;
      var beta = Spectrum.Create(1f);

      for (int bounces = 0; bounces <= 20; ++bounces)
      {
        var (d, intersection) = scene.Intersect(ray);
        
        if (intersection == null) break;

        var wo = -ray.d;
        if (intersection.Obj is Light)
        {
          if (bounces == 0)
          {
            L.AddTo(beta * intersection.Le(wo));
          }
          break;
        }

        L.AddTo(beta * Light.UniformSampleOneLight(intersection, scene));

        if (intersection.Obj is Shape objectHit)
        {
          var (f, wi, pdf, isSpecular) = objectHit.BSDF.Sample_f(wo, intersection);

          if (f.IsBlack() || Math.Abs(pdf) < double.Epsilon) break;

          beta *= f * Vector3.AbsDot(wi, intersection.Normal) / pdf;

          ray = intersection.SpawnRay(wi);
        }

        if (bounces > 3)
        {
          var q = 1 - beta.Max();
          if (ThreadSafeRandom.NextDouble() < q)
          {
            break;
          }

          beta /= 1.0 - q;
        }

      }

      return L;
    }

  }
}
