using System;

namespace PathTracer
{
    public class OrenNayar : BxDF
    {
        private Spectrum kd;
        private double a;
        private double b;

        private static double Phi(double x, double y)
        {
            return Math.Atan(y / x);
        }

        private static double Theta(double x, double y, double z)
        {
            return Math.Atan(Math.Sqrt(x * x + y * y) / z);
        }

        public OrenNayar(Spectrum kd, double roughness)
        {
            this.kd = kd;
            var roughnessSq = roughness * roughness;
            a = 1 - roughnessSq / (2 * (roughnessSq + 0.33));
            b = 0.45 * roughnessSq / (roughnessSq + 0.09);
        }

        public override Spectrum f(Vector3 wo, Vector3 wi)
        {
            // var phiWi = Phi(wi.x, wi.y);
            // var phiWo = Phi(wo.x, wo.y);
            // var thetaWi = Theta(wi.x, wi.y, wi.z);
            // var thetaWo = Theta(wo.x, wo.y, wo.z);
            //
            // var alpha = Math.Max(thetaWi, thetaWo);
            // var beta = Math.Min(thetaWi, thetaWo);
            //
            // return kd * Utils.PiInv * (a + b * Math.Max(0, Math.Cos(phiWi - phiWo))) * Math.Sin(alpha) * Math.Tan(beta);
            
            var sinThetaI = Utils.SinTheta(wi);
            var sinThetaO = Utils.SinTheta(wo);
            var maxCos = 0.0;
            if (sinThetaI > 1e-4 && sinThetaO > 1e-4)
            {
                var sinPhiI = Utils.SinPhi(wi);
                var cosPhiI = Utils.CosPhi(wi);
                var sinPhiO = Utils.SinPhi(wo);
                var cosPhiO = Utils.CosPhi(wo);
                var dCos = cosPhiI * cosPhiO + sinPhiI * sinPhiO;
                maxCos = Math.Max(0, dCos);
            }

            double sinAlpha;
            double tanBeta;
            if (Utils.AbsCosTheta(wi) > Utils.AbsCosTheta(wo)) {
                sinAlpha = sinThetaO;
                tanBeta = sinThetaI / Utils.AbsCosTheta(wi);
            } else {
                sinAlpha = sinThetaI;
                tanBeta = sinThetaO / Utils.AbsCosTheta(wo);
            }

            return kd * Utils.PiInv * (a + b * maxCos * sinAlpha * tanBeta);
        }

        public override (Spectrum, Vector3, double) Sample_f(Vector3 wo)
        {
            var wi = Samplers.CosineSampleHemisphere();
            if (wo.z < 0)
                wi.z *= -1;
            double pdf = Pdf(wo, wi);
            return (f(wo, wi), wi, pdf);
        }

        public override double Pdf(Vector3 wo, Vector3 wi)
        { 
            if (!Utils.SameHemisphere(wo, wi))
                return 0;

            return Math.Abs(wi.z) * Utils.PiInv; // wi.z == cosTheta
        }
    }
}