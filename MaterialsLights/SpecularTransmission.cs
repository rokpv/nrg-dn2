using System;

namespace PathTracer
{
    public class SpecularTransmission : BxDF
    {
        private Spectrum r;
        private FresnelDielectric fresnel;

        public override bool IsSpecular => true;

        public SpecularTransmission(Spectrum r, double fresnel1, double fresnel2)
        {
            this.r = r;
            fresnel = new FresnelDielectric(fresnel1, fresnel2);
        }

        /// <summary>
        /// f of perfect specular transmission is zero (probability also)
        /// </summary>
        /// <param name="wo"></param>
        /// <param name="wi"></param>
        /// <returns></returns>
        public override Spectrum f(Vector3 wo, Vector3 wi)
        {
            return Spectrum.ZeroSpectrum;
        }

        /// <summary>
        /// Sample returns a single possible direction
        /// </summary>
        /// <param name="woL"></param>
        /// <returns></returns>
        public override (Spectrum, Vector3, double) Sample_f(Vector3 wo)
        {
            var F = fresnel.Evaluate(Utils.CosTheta(wo));

            bool entering = Utils.CosTheta(wo) > 0;
            var etaI = entering ? fresnel.EtaI : fresnel.EtaT;
            var etaT = entering ? fresnel.EtaT : fresnel.EtaI;

            var n = new Vector3(0, 0, 1);
            n = Vector3.Dot(n, wo) < 0 ? -n : n;

            var (refracted, wt) = Refract(wo, n, etaI / etaT);
            
            if (!refracted) return (Spectrum.ZeroSpectrum, null, 0);

            Spectrum ft = r * (1 - F.Max());

            var pdf = 1 - F.Max();
            return (ft / Utils.AbsCosTheta(wt), wt, pdf);
        }

        /// <summary>
        /// Probability is 0
        /// </summary>
        /// <param name="wo"></param>
        /// <param name="wi"></param>
        /// <returns></returns>
        public override double Pdf(Vector3 wo, Vector3 wi)
        {
            return 0;
        }
        
        private (bool, Vector3) Refract(Vector3 wi, Vector3 n, double eta)
        {
            var cosThetaI = Vector3.Dot(n, wi);
            var sin2ThetaI = Math.Max(0, 1 - cosThetaI * cosThetaI);
            var sin2ThetaT = eta * eta * sin2ThetaI;
            
            if (sin2ThetaT >= 1) return (false, null);

            var cosThetaT = Math.Sqrt(1 - sin2ThetaT);

            var wt = eta * -wi + (eta * cosThetaI - cosThetaT) * n;
            return (true, wt);
        }
    }
}