# [Package] mc_localizer

Monte Carlo Localization Module

## Functions

Logic

|Index|Function|Description|
|---|---|---|
|2|updateParticlesByMotionModel||
|5|calculateLikelihoodsByMeasurementModel||
||calculateLikelihoodsByDecisionModel||
|6|calculateGLSampledPosesLikelihood||
|7|estimateRobotPose||
|8|resampleParticles||

**estimateRobotPose**

$$
\mathbf{x}_t
= \sum_{i=1}^{\; {}^P\!M \;} {}^P\!\omega_t^{[i]}\, {}^P\mathbf{x}_t^{[i]}
+ \sum_{i=1}^{ \;{}^G\!M \;} {}^G\!\omega_t^{[i]}\, {}^G\mathbf{x}_t^{[i]}
$$
