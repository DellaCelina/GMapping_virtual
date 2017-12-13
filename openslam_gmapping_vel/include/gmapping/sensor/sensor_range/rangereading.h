#ifndef RANGEREADING_H
#define RANGEREADING_H

#include <vector>
#include <gmapping/sensor/sensor_base/sensorreading.h>
#include <gmapping/sensor/sensor_range/rangesensor.h>

namespace GMapping{

class RangeReading: public SensorReading, public std::vector<double>{
	public:
		RangeReading(const RangeSensor* rs, double time=0);
		//RangeReading(unsigned int n_beams, double min_angle, double max_angle, const double* d, const RangeSensor* rs, double time=0);
		RangeReading(unsigned int n_beams, unsigned int min_beam_index, unsigned int max_beam_index, unsigned int min_virtual_beam_index, unsigned int max_virtual_beam_index, const double* d, const RangeSensor* rs, double time=0);
		virtual ~RangeReading();
		inline const OrientedPoint& getPose() const {return m_pose;}
		inline void setPose(const OrientedPoint& pose) {m_pose=pose;}
		unsigned int rawView(double* v, double density=0.) const;
		std::vector<Point> cartesianForm(double maxRange=1e6) const;
		unsigned int activeBeams(double density=0.) const;
		inline unsigned int getMinBeamIdx() const {return m_min_beam_index;}
		inline unsigned int getMaxBeamIdx() const {return m_max_beam_index;}
		inline unsigned int getMinVirtualBeamIdx() const {return m_min_virtual_beam_index;}
		inline unsigned int getMaxVirtualBeamIdx() const {return m_max_virtual_beam_index;}
	protected:
		OrientedPoint m_pose;
		unsigned int m_min_beam_index;
		unsigned int m_max_beam_index;
		unsigned int m_min_virtual_beam_index;
		unsigned int m_max_virtual_beam_index;
};

};

#endif
