#include <mitsuba/render/scene.h>
#include <mitsuba/core/statistics.h>
#include <regex>

MTS_NAMESPACE_BEGIN

static StatsCounter avgPathLength("shadow tracer", "Average path length", EAverage);

class mask_integrator: public MonteCarloIntegrator {
public:
	MTS_DECLARE_CLASS()

public:
	mask_integrator(const Properties& props) :MonteCarloIntegrator(props) {}

	// unserialize, in order
	mask_integrator(Stream* stream, InstanceManager* manager) :
		MonteCarloIntegrator(stream, manager) {}

	// serialize
	void serialize(Stream* stream, InstanceManager* manager) const {
		SamplingIntegrator::serialize(stream, manager);
	}

	std::string toString() const {
		std::ostringstream oss;
		oss << "Mask Integrator[" << endl
			<< "  maxDepth = " << m_maxDepth << "," << endl
			<< "  rrDepth = " << m_rrDepth << "," << endl
			<< "  strictNormals = " << m_strictNormals << endl
			<< "]";
		return oss.str();
	}

	inline Float miWeight(Float pdfA, Float pdfB) const {
		pdfA *= pdfB;
		pdfB *= pdfB;
		return pdfA / (pdfA + pdfB);
	}

	/// Query for an unbiased estimate of the radiance along r
	Spectrum Li(const RayDifferential& r, RadianceQueryRecord& rRec) const { 
		const Scene *scene = rRec.scene;
		Intersection &its = rRec.its;
		RayDifferential ray(r);
		Spectrum Li(0.0f);
		bool scattered = false;

		// Perform the first ray intersection
		rRec.rayIntersect(ray);
		ray.mint = Epsilon;

		Spectrum throughput(1.0f);
		Float eta = 1.0f;

		//auto check_is_occlusion = [](const Intersection &its) {
		//	its.shape->
		//};

		
		if (!its.isValid()) {
			return Li;
		}

		if (its.shape->get_is_render_target()) {
			Spectrum mask_color;
			mask_color.fromLinearRGB(1.0f, 1.0f, 1.0f);
			Li = mask_color;
		}

		return Li;
	}


	bool preprocess(const Scene* scene, RenderQueue* queue, const RenderJob* job, int sceneResID, int cameraResID, int samplerResID) {
		SamplingIntegrator::preprocess(scene, queue, job, sceneResID, cameraResID, samplerResID);
		return true;
	}

private:
};

MTS_IMPLEMENT_CLASS_S(mask_integrator, false, SamplingIntegrator);
MTS_EXPORT_PLUGIN(mask_integrator, "A object mask integrator");
MTS_NAMESPACE_END