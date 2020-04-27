namespace PathTracer
{
    public class OrenNayar : BxDF
    {
        public override Spectrum f(Vector3 wo, Vector3 wi)
        {
            throw new System.NotImplementedException();
        }

        public override (Spectrum, Vector3, double) Sample_f(Vector3 woL)
        {
            throw new System.NotImplementedException();
        }

        public override double Pdf(Vector3 wo, Vector3 wi)
        {
            throw new System.NotImplementedException();
        }
    }
}