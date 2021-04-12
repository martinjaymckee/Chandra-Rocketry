#ifndef CHANDRA_ROCKETRY_FLIGHT_EVENT_DETECTORS_H
#define CHANDRA_ROCKETRY_FLIGHT_EVENT_DETECTORS_H

//
// Chandra-HAL Includes
//
#include <chrono.h>
#include <circular_buffer.h>
#include <gravity.h>
#include <quantity.h>
#include <quantity_utils.h>
#include <units.h>

namespace chandra
{
namespace rocketry
{

template<class TimestampClock>
class FlightEventResult
{
  public:
    using clock_t = TimestampClock;
    using time_point_t = typename clock_t::time_point_t;

    FlightEventResult(
      bool _detected,
      time_point_t _timestamp
    ) : detected(_detected), timestamp(_timestamp) {}

    FlightEventResult( const bool& _detected = false )
      : FlightEventResult(_detected, clock_t::now()) {}

    bool detected = false;
    time_point_t timestamp;
    operator bool() const { return detected; }
};

namespace internal
{
template<class Value> // TODO: THIS IS NOT THE ACTUAL FILTER IMPLEMENTATION THAT SHOULD BE USED...
struct DummyFilter{  //    LOOK THROUGH THE AVAILABLE FILTERS IN CHANDRA-HAL AND NORMALIZE THEM, THEN USE THOSE AND ADD A DUMMY FILTER
  using value_t = Value;
  value_t operator () (const value_t& _value) const {
    return _value;
  }
};

} /*namespace internal*/

template<
  class Value,
  uint8_t BufferSize=32,
  template<class> class AccelerationFilter = internal::DummyFilter,
  class TimestampClock = chandra::chrono::timestamp_clock,
  class TimeUnits = chandra::units::mks::s,
  class VelocityUnits = chandra::units::mks::m_per_s,
  class AccelerationUnits = chandra::units::mks::m_per_s2
>
class LiftoffDetector
{
  public:
    using value_t = Value;
    using clock_t = TimestampClock;
    using time_t = chandra::units::mks::Q_s<value_t>;
    using velocity_t = chandra::units::Quantity<value_t, VelocityUnits>;
    using buffer_t = chandra::FixedCircularBuffer<velocity_t, BufferSize>;
    using acceleration_t = chandra::units::Quantity<value_t, VelocityUnits>;
    using acceleration_filter_t = AccelerationFilter<acceleration_t>;
    using event_result_t = FlightEventResult<clock_t>;

    template<class... Args>
    LiftoffDetector(const velocity_t& _v_thresh = velocity_t{15}, Args... args)
      : a_filter_{args...}, v_thresh_{_v_thresh} {}

    bool reset() {
      detected_ = false;
      v_est_ = velocity_t{0};
      return true;
    }

    template<class ChronoTime>
    event_result_t operator () (const ChronoTime& _dt, acceleration_t _a) {
      if(detected_) return {}; // If already detected, never return detection again

      const auto a_smooth = a_filter_(_a);
      if(units::isNonnegative(a_smooth)){
        if(dv_buffer_.full()) {
            v_est_ -= dv_buffer_[0];
        }

        if(dv.value() < value_t{0}) {
          dv_buffer_ << velocity_t{0};
        } else {
          const time_t dt = units::chronoToQuantity<time_t>(_dt);
          const velocity_t dv{dt * _a};
          dv_buffer_ << dv;
          v_est_ += dv;
        }

        if(v_est_ > v_thresh_) {
          detected_ = true;
          // TODO: ESTIMATE LIFTOFF TIME
          const auto t = clock_t::now();
          return {true, t};
        }
      }

      return {false};
    }

  protected:
    bool detected_ = false;
    acceleration_filter_t a_filter_;
    buffer_t dv_buffer_;
    velocity_t v_est_;
};

template<
  class Value,
  template<class> class DerivitiveEstimator = internal::DummyFilter,
  template<class> class JerkFilter = internal::DummyFilter,
  class TimestampClock = chandra::chrono::timestamp_clock,
  class TimeUnits = chandra::units::mks::s,
  class AccelerationUnits = chandra::units::mks::m_per_s2,
  class JerkUnits = chandra::units::mks::m_per_s3
>
class BurnoutDetector
{

};

} /*namespace rocketry*/
} /*namespace chandra*/

#endif /*CHANDRA_ROCKETRY_FLIGHT_EVENT_DETECTORS_H*/
