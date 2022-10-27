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
#include <windowed_filters.h>

namespace chandra
{
namespace rocketry
{
template<class TimeType>
class FlightEventResult
{
  public:
    using time_t = TimeType;

    FlightEventResult(
      bool _detected,
      time_t _latency
    ) : detected(_detected), latency(_latency) {}

    FlightEventResult( const bool& _detected = false )
        : FlightEventResult(_detected, time_t{}) {}

    bool detected = false;
    time_t latency;
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
  class TimestampClock = chandra::chrono::timestamp_clock,
  uint8_t BufferSize=32,
  template<class> class AccelerationFilter = internal::DummyFilter,
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


  protected:
      struct VelocitySample
      {
          VelocitySample() {}

          VelocitySample(time_t _dt, velocity_t _dv)
              : dt(_dt), dv(_dv) {}

          time_t dt;
          velocity_t dv;
      };

  public:
      using buffer_t = chandra::NonblockingFixedCircularBuffer<VelocitySample, BufferSize>;
      using acceleration_t = chandra::units::Quantity<value_t, AccelerationUnits>;
      using acceleration_filter_t = AccelerationFilter<acceleration_t>;
      using event_result_t = FlightEventResult<time_t>;

      template<class... Args>
        LiftoffDetector(const velocity_t& _v_thresh = velocity_t{15}, const acceleration_t& _a_thresh = 3_gees_, Args... args)
          : a_filter_{args...}, v_thresh_{_v_thresh}, a_thresh_{_a_thresh} {
            reset();
        }

        bool reset() {
          detected_ = false;
          v_est_ = velocity_t{0};
          dv_buffer_.clear();
          return true;
        }

        event_result_t operator () (const time_t _dt, const acceleration_t _a) {
          if(detected_) return {}; // If already detected, never return detection again

          const auto a_smooth = a_filter_(_a);
          if(units::isNonnegative(a_smooth)){
            const velocity_t dv{_dt * a_smooth};

            if(dv_buffer_.full()) {
                v_est_ -= dv_buffer_[0].dv;
            }

            if(dv.value() < value_t{0}) {
              dv_buffer_ << VelocitySample(_dt, velocity_t{0});
            } else {
              dv_buffer_ << VelocitySample(_dt, dv);
              v_est_ += dv;
            }

            if( (v_est_ >= v_thresh_) &&
                (a_smooth >= a_thresh_) ) {
              detected_ = true;
              const time_t t_latency = time_t{v_est_ * _dt / (2 * dv)};
              return event_result_t(true, t_latency);
            }
          }

          return event_result_t(false);
        }

    private:
        bool detected_ = false;
        acceleration_filter_t a_filter_;
        buffer_t dv_buffer_;
        velocity_t v_thresh_;
        velocity_t v_est_{0};
        acceleration_t a_thresh_;
};


template<
  class Value,
  class TimestampClock = chandra::chrono::timestamp_clock,
  template<class> class AccelerationFilter = internal::DummyFilter,
  template<class> class JerkFilter = internal::DummyFilter,
  class TimeUnits = chandra::units::mks::s,
  class AccelerationUnits = chandra::units::mks::m_per_s2,
  class JerkUnits = chandra::units::mks::m_per_s3
>
class BurnoutDetector
{
  public:
    using value_t = Value;
    using clock_t = TimestampClock;
    using time_t = chandra::units::mks::Q_s<value_t>;
    using acceleration_t = chandra::units::Quantity<value_t, AccelerationUnits>;
    using jerk_t = chandra::units::Quantity<value_t, JerkUnits>;
    using acceleration_filter_t = AccelerationFilter<acceleration_t>;
    using jerk_filter_t = JerkFilter<jerk_t>;
    using event_result_t = FlightEventResult<time_t>;

    event_result_t operator () (const time_t _dt, const acceleration_t _a) {
      event_result_t result;
      return result;
    }

  private:

};


template<
  class Value,
  class TimestampClock = chandra::chrono::timestamp_clock,
  template<class> class LengthFilter = internal::DummyFilter,
  template<class, class> class VelocityFilter = chandra::signal::ExponentialWeightedAverage,
  class TimeUnits = chandra::units::mks::s,
  class LengthUnits = chandra::units::mks::m,
  class VelocityUnits = chandra::units::mks::m_per_s
>
class ApogeeDetector
{
  public:
    using value_t = Value;
    using clock_t = TimestampClock;
    using time_t = chandra::units::mks::Q_s<value_t>;
    using length_t = chandra::units::Quantity<value_t, LengthUnits>;
    using velocity_t = chandra::units::Quantity<value_t, VelocityUnits>;
    using velocity_filter_t = VelocityFilter<velocity_t, clock_t>;
    using event_result_t = FlightEventResult<time_t>;

    template<class... Args>
    ApogeeDetector(Args... args) : v_filt_{ args... }, v_thresh_{ 1.5_m_per_s_ } { // TODO: FIGURE OUT THE BEST API FOR USING THIS....
        const auto g{ 9.80655_m_per_s2_ }; // TODO: THIS NEEDS TO E MADE CONSTEXPR
        t_latency_ = v_thresh_ / g;
        h_thresh_ = length_t((t_latency_ * t_latency_) * g);
        reset();
    }

    bool reset() {
        initialized_ = false;
        armed_ = false;
        detected_ = false;
        v_est_ = velocity_t{ 0 };
        return true;
    }

    event_result_t operator () (const time_t _dt, const length_t _h) {
        event_result_t result;
        if (detected_) return result;

        if (!initialized_) {
            initialized_ = true;
            h_last_ = _h;
            h_max_ = _h;
        }

        if (_h > h_max_) {
            h_max_ = _h;
        }

        const velocity_t v_est((h_last_ - _h) / _dt );
        v_est_ = v_filt_(v_est);

        if (armed_) {
            if (((h_max_ - _h) > h_thresh_) && (v_est_ >= v_thresh_)) {
                detected_ = true;
                result.detected = true;
                result.latency = t_latency_;
            }
        }
        else {
            if (v_est_ < -15_m_per_s_) armed_ = true;
        }

        h_last_ = _h;

        return result;
    }

  private:
      bool initialized_ = false;
      bool armed_ = false;
      bool detected_ = false;
      velocity_filter_t v_filt_;
      velocity_t v_thresh_;
      velocity_t v_arm_;
      time_t t_latency_;
      length_t h_thresh_;
      velocity_t v_est_;
      length_t h_last_;
      length_t h_max_;
};


template<
  class Value,
  class TimestampClock = chandra::chrono::timestamp_clock,
  template<class, class> class VelocityFilter = chandra::signal::ExponentialWeightedAverage,
  class TimeUnits = chandra::units::mks::s,
  class LengthUnits = chandra::units::mks::m,
  class VelocityUnits = chandra::units::mks::m_per_s
>
class LandingDetector
{
    public:
        using value_t = Value;
        using pos_t = chandra::aero::ENU<value_t>;
        using clock_t = TimestampClock;
        using time_t = chandra::units::mks::Q_s<value_t>;
        using length_t = typename pos_t::value_t;
        using velocity_t = chandra::units::Quantity<value_t, VelocityUnits>;
        using velocity_filter_t = VelocityFilter<velocity_t, clock_t>;
        using event_result_t = FlightEventResult<time_t>;

        template<class... Args>
        LandingDetector(value_t _e, Args... args)
            : v_thresh_{ 0.5_m_per_s_ }, v_arm_(15_m_per_s_), v_filt_z_{ _e, args... } {}

        bool reset() {
            armed_ = false;
            initialized_ = false;
            detected_ = false;
            return true;
        }

        event_result_t operator () (const time_t _dt, const length_t _h) {
            event_result_t result;
            if (detected_) return result;

            const auto t = clock_t::now();

            if (!initialized_) {
                h_last_ = _h;
                h_max_ = _h;
                initialized_ = true;
            }

            if (_h > h_max_) {
                h_max_ = _h;
            }

            const auto v_z_est_ = (h_last_ - _h) / _dt;
            v_z_ = v_filt_z_(v_z_est_);
            h_last_ = _h;

            if (armed_) {
                if (
                    (std::abs(v_z_) <= v_thresh_) &&
                    (_h <= (h_max_ / 4))
                ) {
                    detected_ = true;
                    result.detected = true;
                    result.latency = 3_s_;
                }
            }
            else {
                armed_ = (std::abs(v_z_) > v_arm_);
            }

            return result;
        }

  private:
      bool initialized_ = false;
      bool armed_ = false;
      bool detected_ = false;
      velocity_t v_thresh_;
      velocity_t v_arm_;
      length_t h_last_;
      length_t h_max_;
      velocity_filter_t v_filt_z_;
      velocity_t v_z_;
};
} /*namespace rocketry*/
} /*namespace chandra*/

#endif /*CHANDRA_ROCKETRY_FLIGHT_EVENT_DETECTORS_H*/
