#ifndef CHANDRA_ROCKETRY_LOOP_CONTROL_H
#define CHANDRA_ROCKETRY_LOOP_CONTROL_H

#include <chrono>
using namespace std::literals::chrono_literals;

//
// Chandra-HAL Includes
//
#include <chrono.h>

namespace chandra
{
namespace rocketry
{

template<
  class SystemClock = chandra::chrono::timestamp_clock,
  class HILClock = chandra::chrono::mock_clock
>
class LoopControl
{
  public:
    using loop_clock_t = chandra::chrono::compound_clock<SystemClock, HILClock>;
    using hil_clock_t = HILClock;
    using duration_t = typename loop_clock_t::duration;
    using timepoint_t = typename loop_clock_t::time_point;

    struct LoopTriggerEvent
    {
      LoopTriggerEvent() : trigger{0}, dt{0} {}
      LoopTriggerEvent(const bool& _trigger, duration_t _dt)
        : trigger{_trigger}, dt{_dt} {}

      constexpr operator bool () const { return trigger; }

      bool trigger = false;
      duration_t dt;
    };

    LoopControl(const duration_t& _t_loop)
      : t_last_{duration_t{0}}, t_loop_{_t_loop} {}

    bool init() {
      loop_clock_t::init();
      reset();
      return true;
    }

    bool reset() {
      loop_clock_t::reset();
      t_last_ = loop_clock_t::now();
      return true;
    }

    bool mode(const bool& _sys) {
      system_active_ = _sys;
      loop_clock_t::source(_sys);
      return mode();
    }

    constexpr bool mode() const { return system_active_; }

    template<class Rep, class Period>
    duration_t loop_time(const std::chrono::duration<Rep, Period>& _t) {
      t_loop_ = _t;
      return loop_time();
    }

    constexpr duration_t loop_time() const {
      return t_loop_;
    }

    template<class Rep, class Period>
    bool hil_update(const std::chrono::duration<Rep, Period>& _dt) {
        if(!system_active_) {
          hil_trigger_ = true;
          loop_clock_t::advance(_dt);
          return true;
        }
        return false;
    }

    LoopTriggerEvent trigger() noexcept {
      const auto t = loop_clock_t::now();
      duration_t dt = t_loop_;

      if(triggered_) sync();

      if(system_active_) {
        // Advance to nearset timestep to avoid multiple loop triggers in rapid succession
        while(chandra::chrono::after(t_loop_, t_last_, t)) {
          t_last_ += t_loop_;
          triggered_ = true;
        }
      } else {
        if(hil_trigger_) {
          hil_trigger_ = false;
          triggered_ = true;
          dt = t - t_last_;
          t_last_ = t;
        }
      }

      return {triggered_, dt};
    }

    template<class Func>
    bool sync(const Func& _func) {
      if(triggered_) {
        _func(!system_active_, t_last_);
        triggered_ = false;
        return true;
      }
      return false;
    }

    bool sync() noexcept {
      return sync([](bool, auto){});
    }

  protected:
    timepoint_t t_last_;
    duration_t t_loop_;
    bool system_active_ = true;
    bool hil_trigger_ = false;
    bool triggered_ = false;
};

} /*namespace rocketry*/
} /*namespace chandra*/

#endif /*CHANDRA_ROCKETRY_LOOP_CONTROL_H*/
