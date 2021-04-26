#ifndef CHANDRA_ROCKETRY_LOOP_CONTROL_H
#define CHANDRA_ROCKETRY_LOOP_CONTROL_H

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
    using duration_t = typename loop_clock_t::duration;
    using timepoint_t = typename loop_clock_t::time_point;

    struct LoopTriggerEvent
    {
      LoopTriggerEvent() : trigger{0}, dt{0} {}
      LoopTriggerEvent(const bool& _trigger, duration_t& _dt)
        : trigger{_trigger}, dt{_dt} {}

      bool trigger = false;
      duration_t dt;
    };

    LoopControl(const duration_t& _t_loop)
      : t_last_{duration_t{0}}, t_loop_{_t_loop} {}

    bool init() {
      return loop_clock_t::init();
    }

    bool mode(const bool& _sys) const {
      system_active_ = _sys;
      return mode();
    }

    constexpr bool mode() const { return system_active_; }

    LoopTriggerEvent trigger() noexcept {
      const auto t = loop_clock_t::now();
      const auto dt = t - t_last_;

      if(triggered_) sync();

      if(system_active_) {
        if(chandra::chrono::after(t_loop_, t_last_, t)) {
          t_last_ += t_loop_;
          triggered_ = true;
        }
      } else {
        if(hil_trigger_) {
          hil_trigger_ = false;
          triggered_ = true;
          t_last_ = t;
        }
      }

      return {triggered_, dt};
    }

    template<class Func>
    bool sync(Func& _func) {
      if(triggered_) {
        _func(t_last_);
        triggered_ = false;
        return true;
      }
      return false;
    }

    bool sync() noexcept {
      return sync([](auto){});
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
