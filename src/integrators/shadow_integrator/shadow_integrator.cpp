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
			if (!its.isValid()) {
				break;
			}

			if(!its.shape->get_is_render_ground()) {
				//std::string log_str = shape_name + " is not ground";
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

				//if(!value.isZero()) {
				//	const Emitter *emitter = static_cast<const Emitter *>(dRec.object);
				//	
				//	/* Allocate a record for querying the BSDF */
				//	BSDFSamplingRecord bRec(its, its.toLocal(dRec.d), ERadiance);

				//	/* Evaluate BSDF * cos(theta) */
				//	const Spectrum bsdfVal = bsdf->eval(bRec);

				//	/* Prevent light leaks due ot the use of shading normals */
				//	if(!bsdfVal.isZero() && (!m_strictNormals || dot(its.geoFrame.n, dRec.d) * Frame::cosTheta(bRec.wo) > 0)) {
				//		/* Calculate prob. of having generated that direction using BSDF sampling */
				//		Float bsdfPdf = (emitter->isOnSurface() && dRec.measure == ESolidAngle) ? bsdf->pdf(bRec) : 0;
				//		Float weight = miWeight(dRec.pdf, bsdfPdf);
				//		
				//		Li += throughput * value * bsdfVal * weight;
				//	}
				//}

				if(value.isZero()) {
					Spectrum shadow_value;
					shadow_value.fromLinearRGB(1.0f, 1.0f, 1.0f);
					Li += throughput * shadow_value;
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