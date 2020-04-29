namespace PathTracer
{
    public class SphericalLight : Light
    {
        private Spectrum lemit;
        private Sphere sphere;

        public SphericalLight(double radius, Transform objectToWorld, Spectrum l, bool innerEmision = false, double intensity = 1)
        {
            lemit = l * intensity;
            sphere = new Sphere(radius, objectToWorld, innerEmision);
        }

        public override (double?, SurfaceInteraction) Intersect(Ray r)
        {
            (double ?t, SurfaceInteraction si) = sphere.Intersect(r);
            if (si != null)
                si.Obj = this;
            return (t, si);
        }

        public override (SurfaceInteraction, double) Sample()
        {
            return sphere.Sample();
        }

        public override (Spectrum, Vector3, double, Vector3) Sample_Li(SurfaceInteraction source)
        {
            (SurfaceInteraction pShape, double pdf) = sphere.Sample(source);

            if (pdf == 0 || (pShape.Point - source.Point).LengthSquared() < Renderer.Epsilon)
            {
                return (Spectrum.ZeroSpectrum, Vector3.ZeroVector, 0, Vector3.ZeroVector);
            }

            var wi = (pShape.Point - source.Point).Normalize();
            var Li = L(pShape, -wi);
            return (Li, wi, pdf, pShape.Point);
        }

        public override Spectrum L(SurfaceInteraction si, Vector3 w)
        {
            return (Vector3.Dot(si.Normal, w) > 0) ? lemit : Spectrum.ZeroSpectrum;
        }

        public override double Pdf_Li(SurfaceInteraction si, Vector3 wi)
        {
            return sphere.Pdf(si, wi);
        }
    }
}