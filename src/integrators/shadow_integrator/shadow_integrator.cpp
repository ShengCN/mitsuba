#include <mitsuba/render/scene.h>
#include <mitsuba/core/statistics.h>
#include <regex>

MTS_NAMESPACE_BEGIN

static StatsCounter avgPathLength("shadow tracer", "Average path length", EAverage);

class shadow_integrator : public MonteCarloIntegrator {
public:
	MTS_DECLARE_CLASS()

public:
	shadow_integrator(const Properties& props) :MonteCarloIntegrator(props) {}

	// unserialize, in order
	shadow_integrator(Stream* stream, InstanceManager* manager) :
		MonteCarloIntegrator(stream, manager) {}

	// serialize
	void serialize(Stream* stream, InstanceManager* manager) const {
		SamplingIntegrator::serialize(stream, manager);
	}

	std::string toString() const {
		std::ostringstream oss;
		oss << "Shadow Detection[" << endl
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

		while(rRec.depth <= m_maxDepth || m_maxDepth < 0) {
			// hit nothing
			if (!its.isValid()) {
				Li = throughput;
				break;
			}

			// when hit render target, ignore this hit, go straight
			while(its.shape->get_is_render_target()) {
				ray = Ray(its.p + ray.d * Epsilon, ray.d, ray.time);
				if (!scene->rayIntersect(ray, its))
					break;
			}


			// if not hit shadow receiver
			if(!its.shape->get_is_render_ground()) {
				//std::string log_str = shape_name + " is not ground";
				Li = throughput;
				break;
			}

			const BSDF *bsdf = its.getBSDF(ray);

			/* Possibly include emitted radiance*/
			if(its.isEmitter() && (rRec.type & RadianceQueryRecord::EEmittedRadiance) && (!m_hideEmitters || scattered)) {
				/*Li += throughput * its.Le(-ray.d);*/
				break;
			}
			
			if((rRec.depth >= m_maxDepth && m_maxDepth > 0) || (m_strictNormals && dot(ray.d, its.geoFrame.n) * Frame::cosTheta(its.wi) >=0)) {
				/*
					Only continue if: 
						1. The current path lenght is below the specified maximum
						2. If 'strictNormals' = true, when the geometric and shading 
						   normals classify the incident direction to the same side
				*/

				break;
			}

			/* ==================================================================== */
			/*                     Direct illumination sampling                     */
			/* ==================================================================== */
			/* estimate the direct illumination if this is requested */
			DirectSamplingRecord dRec(its);
			if(rRec.type  & RadianceQueryRecord::EDirectSurfaceRadiance && (bsdf->getType() & BSDF::ESmooth)) {
				Spectrum value = scene->sampleEmitterDirect(dRec, rRec.nextSample2D());

				if(!value.isZero()) {
					/* one more condition here. compute if the occluder is the render target */
					Ray occlusion_test_ray(dRec.ref, dRec.d, 0.0f);
					Intersection occlusion_test_its;
					if (scene->rayIntersect(occlusion_test_ray, occlusion_test_its)) {
						if (occlusion_test_its.shape->get_is_render_target()) {
							Li += throughput * value;
							break;
						}
					}

					Li = throughput;
					break;
				}

			}

			break;
		} 

		return Li;
	}


	bool preprocess(const Scene* scene, RenderQueue* queue, const RenderJob* job, int sceneResID, int cameraResID, int samplerResID) {
		SamplingIntegrator::preprocess(scene, queue, job, sceneResID, cameraResID, samplerResID);

		const AABB& scene_aabb = scene->getAABB();
		Point camera_position = scene->getSensor()->getWorldTransform()->eval(0).transformAffine(Point(0.0f));
		m_max_dist = -std::numeric_limits<Float>::infinity();

		for (int i = 0; i < 8; ++i) {
			m_max_dist = std::max(m_max_dist, (camera_position - scene_aabb.getCorner(i)).length());
		}

		return true;
	}

private:
	Spectrum m_color;
	Float m_max_dist;
};

MTS_IMPLEMENT_CLASS_S(shadow_integrator, false, SamplingIntegrator);
MTS_EXPORT_PLUGIN(shadow_integrator, "A contrived integrator");
MTS_NAMESPACE_END