// ConsoleApplication1.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <iostream>
#include <fstream>
#include <random>
#include <string>
#include <vector>

//
// Include Chandra-HAL
//
#include <chrono.h>
#include <coordinates.h>
#include <gnss_tracker_protocol.h>
#include <matrix.h>
#include <timing.h>
#include <units.h>
using namespace chandra::units::mks::literals;

template<class T>
struct TD;

//
// Include Chandra-Rocketry
//
#include <flight_event_detectors.h>

//
// Application Types (Basic)
//
using value_t = double;
using system_clock_t = chandra::chrono::mock_clock;

//
// Chandra-HAL Variable Definitions
//
system_clock_t::duration chandra::chrono::mock_clock::t_offset_{ 0 };

//
// Application Utility Functions
// 


//
// Application Types (Advanced)
// 
enum class FlightEvent {
	None,
	Liftoff,
	Burnout,
	Apogee,
	Landing
};

template<class Stream>
Stream& operator << (Stream& _stream, const FlightEvent& _evt) {
	switch (_evt) {
		default:
		case FlightEvent::None:
			_stream << "None";
			break;

		case FlightEvent::Liftoff:
			_stream << "Liftoff";
			break;

		case FlightEvent::Burnout:
			_stream << "Burnout";
			break;

		case FlightEvent::Apogee:
			_stream << "Apogee";
			break;

		case FlightEvent::Landing:
			_stream << "Landing";
			break;

	}

	return _stream;
}


template<
	class QuantityType
>
class QuantityRandomWalk
{
	public:
		using quantity_t = QuantityType;
		using time_t = decltype(1_s_);
		using rate_t = decltype(quantity_t() / 1_s_);
		using value_t = typename chandra::scalar_of<quantity_t>::type;

		constexpr QuantityRandomWalk(std::mt19937& _gen) : gen_{ _gen }, rate_{ rate_t{1} } {}
		constexpr QuantityRandomWalk(std::mt19937& _gen, rate_t _rate) : rate_{ _rate } {}

		quantity_t random_offset(quantity_t _sd) {
			std::normal_distribution<value_t> init_dist(0, chandra::units::scalar_cast(_sd));
			val_ = quantity_t{ init_dist(gen_) };
			return val_;
		}

		constexpr quantity_t offset(quantity_t _val) {
			val_ = _val;
			return val_;
		}

		constexpr quantity_t offset() const {
			return val_;
		}

		constexpr rate_t rate(rate_t _rate) {
			rate_ = _rate;
			return rate_;
		}

		constexpr rate_t rate() const {
			return rate_;
		}

		quantity_t operator () (time_t _dt) {
//			std::cout << "dt = " << _dt << ", rate = " << rate_ << "dt * rate = " << _dt * rate_ << "\n";
			std::normal_distribution<value_t> step_dist(0, chandra::units::scalar_cast(rate_));
			val_ = (val_ + quantity_t{ _dt * rate_t{ step_dist(gen_) } });

			if ((val_ > quantity_t{ 100 }) || (val_ < quantity_t{ -100 }) ) {
				std::cout << "dt = " << _dt << ", dt*rate = " << _dt * rate_ << "val = " << val_ << "\n";
			}

			return val_;
		}

	private:
		std::mt19937& gen_;
		rate_t rate_;
		quantity_t val_;
};

template<
	class VelocityType,
	class AngleType
>
class WindGenerator
{
	public:
		using time_t = decltype(1_s_);
		using value_t = typename VelocityType::value_t;
		using velocity_t = VelocityType;
		using velocity_rate_t = decltype(velocity_t() / 1_s_);
		using angle_t = AngleType;
		using angle_rate_t = decltype(angle_t() / 1_s_);
		using velocity_vector_t = chandra::math::Vector3D<velocity_t, true, chandra::aero::frames::ENU>;

		WindGenerator(std::mt19937& _gen, velocity_t _magnitude, angle_t _heading, const value_t& _e = value_t(0.65))
			: gen_(_gen), magnitude_(_magnitude), magnitude_random_walk_(_gen), heading_(_heading), heading_random_walk_(_gen), e_(_e) {
			magnitude_sd_ = magnitude_ / 100;
			heading_sd_ = 25_deg_ / 100;
			magnitude_rate_ = (magnitude_ / 5_s_);
			heading_rate_ = (360_deg_ / 10_s_);
			magnitude_random_walk_.rate(magnitude_rate_);
			heading_random_walk_.rate(heading_rate_);
			std::normal_distribution<value_t> mag_dist(scalar_cast(magnitude_), scalar_cast(magnitude_ / 3));
			std::normal_distribution<value_t> heading_dist(scalar_cast(heading_), scalar_cast(heading_ / 3));
			last_mag_val_ = mag_dist(gen_);
			last_heading_val_ = heading_dist(gen_);
			magnitude_random_walk_.offset(magnitude_);
			heading_random_walk_.offset(heading_);
		}

		constexpr auto operator() (time_t _dt) { // TODO: THIS NEEDS TO TAKE A TIME VALUE OF SOME SORT....
			using namespace chandra::units;
			velocity_vector_t vec;
			magnitude_ = magnitude_random_walk_(_dt);
			heading_ = heading_random_walk_(_dt);
			//std::cout << "magnitude = " << magnitude_ << ", heading = " << heading_ << "\n";
			std::normal_distribution<value_t> mag_dist(scalar_cast(magnitude_), scalar_cast(magnitude_sd_));
			std::normal_distribution<value_t> heading_dist(scalar_cast(heading_), scalar_cast(heading_sd_));
			const auto new_mag_val = mag_dist(gen_);
			const auto new_heading_val = heading_dist(gen_);
			last_mag_val_ = (e_ * last_mag_val_) + ((1 - e_) * new_mag_val);
			last_heading_val_ = (e_ * last_heading_val_) + ((1 - e_) * new_heading_val);
			//std::cout << "magnitude = " << last_mag_val_ << ", heading = " << last_heading_val_ << "\n";
			vec.x = velocity_t{ std::sin(last_heading_val_) * last_mag_val_ }; // TODO: FIGURE OUT IF THIS IS WORKING CORRECTLY WITH HEADING RELATED TO NORTH....
			vec.y = velocity_t{ std::cos(last_heading_val_) * last_mag_val_ };
			//std::cout << "\t wind = " << vec << "\n";
			return vec;
		}

	private:
		std::mt19937& gen_;
		velocity_t magnitude_;
		velocity_t magnitude_sd_;
		velocity_rate_t magnitude_rate_;
		QuantityRandomWalk<velocity_t> magnitude_random_walk_;
		angle_t heading_;
		angle_t heading_sd_;
		angle_rate_t heading_rate_;
		QuantityRandomWalk<angle_t> heading_random_walk_;
		value_t last_mag_val_;
		value_t last_heading_val_;
		value_t e_;
};

template<class LengthType>
class GNSSErrorModel
{
public:
	using value_t = typename LengthType::value_t;
	using length_t = LengthType;
	using length_vector_t = chandra::math::Vector3D<length_t, true, chandra::aero::frames::ENU>;

	GNSSErrorModel(std::mt19937& _gen, length_t _xy_sd, length_t _z_sd, const value_t& _e = value_t(0.99))
		: gen_(_gen), xy_sd_(_xy_sd), z_sd_(_z_sd), e_(_e) {
		std::normal_distribution<value_t> xy_dist(0, scalar_cast(xy_sd_));
		std::normal_distribution<value_t> z_dist(0, scalar_cast(z_sd_));
		last_offset_.x = length_t{ xy_dist(gen_) };
		last_offset_.y = length_t{ xy_dist(gen_) };
		last_offset_.z = length_t{ z_dist(gen_) };
	}

	constexpr auto operator() () {
		using namespace chandra::units;
		length_vector_t vec;
		std::normal_distribution<value_t> xy_dist(0, scalar_cast(xy_sd_));
		std::normal_distribution<value_t> z_dist(0, scalar_cast(z_sd_));
		vec.x = length_t{ xy_dist(gen_) };
		vec.y = length_t{ xy_dist(gen_) };
		vec.z = length_t{ z_dist(gen_) };
		last_offset_ = vec;
		return vec;
	}

private:
	std::mt19937& gen_;
	length_t xy_sd_;
	length_t z_sd_;
	length_vector_t last_offset_;
	value_t e_;
};


template<class Value, class Clock>
class UptimeTracker
{
	public:
		using time_t = chandra::units::mks::Q_s<Value>;
		using timepoint_t = typename Clock::time_point;

		constexpr auto operator() () const {
			const auto dt = system_clock_t::now() - t_start_;
			return chandra::units::conversions::chronoToQuantity < time_t >(dt);
		}

	private:
		timepoint_t t_start_;
};

template<
	class TimeType,
	class PositionType,
	class VelocityVectorType,
	class AccelerationVectorType,
	class ForceType,
	class AccelerationType
>
struct FlightState
{
	using time_t = TimeType;
	using pos_vec_t = PositionType;
	using vel_vec_t = VelocityVectorType;
	using accel_vec_t = AccelerationVectorType;
	using force_t = ForceType;

	FlightState() {}

	FlightState(const FlightState& _other)
		: t(_other.t), pos(_other.pos), vel(_other.vel), accel(_other.accel),
		thrust(_other.thrust), drag(_other.drag), wind_vel(_other.wind_vel), g(_other.g), flight_mode(_other.flight_mode) {}

	TimeType t;
	PositionType pos;
	VelocityVectorType vel;
	AccelerationVectorType accel;
	ForceType thrust;
	ForceType drag;
	VelocityVectorType wind_vel;
	AccelerationType g;
	chandra::aero::protocol::TrackerFlightMode flight_mode = chandra::aero::protocol::TrackerFlightMode::Preflight;
};

template< class Stream >
Stream& operator << (Stream& _stream, const chandra::aero::protocol::TrackerFlightMode& _flight_mode) {
	switch (_flight_mode) {
		default:
			_stream << "<default>";
			break;

		case chandra::aero::protocol::TrackerFlightMode::Preflight:
			_stream << "Preflight";
			break;


		case chandra::aero::protocol::TrackerFlightMode::Ascent:
			_stream << "Ascent";
			break;


		case chandra::aero::protocol::TrackerFlightMode::Descent:
			_stream << "Descent";
			break;


		case chandra::aero::protocol::TrackerFlightMode::Postflight:
			_stream << "Postflight";
			break;
	}

	return _stream;
}

template<
	class Stream,
	class TimeType,
	class PositionType,
	class VelocityVectorType,
	class AccelerationVectorType,
	class ForceType,
	class AccelerationType
>
Stream& operator << (Stream& _stream, FlightState<TimeType, PositionType, VelocityVectorType, AccelerationVectorType, ForceType, AccelerationType>& _state) {
	std::cout << "State( ";
	std::cout << "t = " << _state.t;
	std::cout << ", pos = " << _state.pos;
	std::cout << ", vel = " << _state.vel;
	std::cout << ", accel = " << _state.accel;
	std::cout << ", thrust = " << _state.thrust;
	std::cout << ", drag = " << _state.drag;
	std::cout << ", wind_vel = " << _state.wind_vel;
	std::cout << ", g = " << _state.g;
	std::cout << ", mode = " << _state.flight_mode;
	std::cout << " )";
	return _stream;
}

template<
	class Value
>
struct LaunchConfig
{
	using pos_t = chandra::aero::LLH<Value>;
	using time_t = chandra::units::mks::Q_s<Value>;

	pos_t pad_position;
	time_t launch_delay_time;
};


template<
	class Value,
	class LengthType,
	class MassType,
	class VelocityType
>
struct RocketConfig
{
	Value cd;
	Value lateral_cd;
	Value parachute_cd;
	LengthType diameter;
	LengthType length;
	MassType mass;
	LengthType parachute_diameter;
	VelocityType descent_terminal;
};


template<
	class TimeType,
	class ForceType
>
struct MotorConfig
{
	TimeType burn_time;
	ForceType average_thrust;
};


template<
	class TimeType,
	class LengthType,
	class VelocityType,
	class AccelerationType,
	class VelocityVectorType,
	class AccelerationVectorType
>
class FlightStatistics
{
	public:
		using time_t = TimeType;
		using length_t = LengthType;
		using velocity_t = VelocityType;
		using acceleration_t = AccelerationType;
		using velocity_vector_t = VelocityVectorType;
		using acceleration_vector_t = AccelerationVectorType;

		time_t t_launch{ 0 };
		time_t t_apogee{ 0 };
		length_t maximum_altitude{ 0 };
		velocity_t maximum_velocity{ 0 };
		acceleration_t maximum_acceleration{ 0 };

		bool update_max_alt(const length_t& _h) {
			if (_h > maximum_altitude) {
				maximum_altitude = _h;
				return true;
			}
			return false;
		}

		bool update_max_vel(const velocity_vector_t& _vel) {
			const auto v_mag = chandra::math::magnitude(_vel);
			if (v_mag > maximum_velocity) {
				maximum_velocity = v_mag;
				return true;
			}
			return false;
		}

		bool update_max_accel(const acceleration_vector_t& _accel) {
			const auto a_mag = chandra::math::magnitude(_accel);
			if (a_mag > maximum_acceleration) {
				maximum_acceleration = a_mag;
				return true;
			}
			return false;

		}
};

template<
	class Stream,
	class TimeType,
	class LengthType,
	class VelocityType,
	class AccelerationType,
	class VelocityVectorType,
	class AccelerationVectorType
>
Stream& operator << (Stream& _stream, const FlightStatistics<TimeType, LengthType, VelocityType, AccelerationType, VelocityVectorType, AccelerationVectorType>& _statistics) {
	_stream << "Statistics( t_launch = " << _statistics.t_launch;
	_stream << ", t_apogee = " << _statistics.t_apogee;
	_stream << ", max(altititude) = " << _statistics.maximum_altitude;
	_stream << ", max(velocity) = " << _statistics.maximum_velocity;
	_stream << ", max(acceleration) = " << _statistics.maximum_acceleration;
	_stream << " )";
	return _stream;
}


template<
	class Value,
	class LengthUnits = chandra::units::mks::m,
	class VelocityUnits = chandra::units::mks::m_per_s,
	class AccelerationUnits = chandra::units::mks::m_per_s2,
	class ForceUnits = chandra::units::mks::N,
	class MassUnits = chandra::units::mks::kg,
	class AngleUnits = chandra::units::mks::rad,
	class TimeUnits = chandra::units::mks::s,
	class DensityUnits = chandra::units::mks::kg_per_m3
>
class LaunchSimulator
{
public:
	using value_t = Value;
	using length_t = chandra::units::Quantity<value_t, LengthUnits>;
	using area_t = decltype(std::declval<length_t>()* std::declval<length_t>());
	using velocity_t = chandra::units::Quantity<value_t, VelocityUnits>;
	using acceleration_t = chandra::units::Quantity<value_t, AccelerationUnits>;
	using force_t = chandra::units::Quantity<value_t, ForceUnits>;
	using mass_t = chandra::units::Quantity<value_t, MassUnits>;
	using angle_t = chandra::units::Quantity<value_t, AngleUnits>;
	using time_t = chandra::units::Quantity<value_t, TimeUnits>;
	using pos_t = chandra::aero::ENU<value_t>;
	using velocity_vector_t = chandra::math::Vector3D<velocity_t, true, typename pos_t::frame_t>;
	using acceleration_vector_t = chandra::math::Vector3D<acceleration_t, true, typename pos_t::frame_t>;
	using force_vector_t = chandra::math::Vector3D<force_t, true, typename pos_t::frame_t>;
	using density_t = chandra::units::Quantity<value_t, DensityUnits>;

	using launch_config_t = LaunchConfig<value_t>;
	using rocket_config_t = RocketConfig<value_t, length_t, mass_t, velocity_t>;
	using motor_config_t = MotorConfig<time_t, force_t>;
	using state_t = FlightState<time_t, pos_t, velocity_vector_t, acceleration_vector_t, force_t, acceleration_t>;
	using statistics_t = FlightStatistics<time_t, length_t, velocity_t, acceleration_t, velocity_vector_t, acceleration_vector_t>;

	LaunchSimulator(const launch_config_t& _launch_config, const rocket_config_t& _rocket_config, const motor_config_t& _motor_config, WindGenerator<velocity_t, angle_t>& _wind_gen) 
		: launch_config_(_launch_config), rocket_config_(_rocket_config), motor_config_(_motor_config), wind_gen_(_wind_gen) {}

	bool init(time_t _t) {
		state_.t = _t;
		reference_position_ = chandra::aero::LLHToECEF(launch_config_.pad_position);
		state_.pos = pos_t{ 0_m_, 0_m_, 0_m_ };
		state_.g = 9.80665_m_per_s2_;
		state_.flight_mode = chandra::aero::protocol::TrackerFlightMode::Preflight;
		const auto r = rocket_config_.diameter / 2;
		reference_area_ = 3.1415926 * r * r;
		lateral_reference_area_ = rocket_config_.diameter * rocket_config_.length;
		const auto r_parachute = rocket_config_.parachute_diameter / 2;
		parachute_area_ = 3.1415926 * r_parachute * r_parachute;
		return true;
	}

	state_t update(time_t _t) {
		auto new_state = state_;
		const auto dt = (_t - state_.t);
		new_state.t = _t;

		const auto lateral_force_constant = 0.5 * rho_ * lateral_reference_area_ * rocket_config_.lateral_cd;

		auto wind = wind_gen_(dt);
		new_state.wind_vel = wind;

		const force_vector_t total_forces;

		acceleration_vector_t dv;
		const auto v_mag = chandra::math::magnitude(new_state.vel);
		const auto dv_x = wind.x - state_.vel.x;
		const auto dv_y = wind.y - state_.vel.y;
		const value_t sign_x = (wind.x > velocity_t{ 0 }) ? 1.0 : -1.0;
		const value_t sign_y = (wind.y > velocity_t{ 0 }) ? 1.0 : -1.0;

		switch (new_state.flight_mode) {
			default:
			case chandra::aero::protocol::TrackerFlightMode::Preflight:
				if (new_state.t >= launch_config_.launch_delay_time) {
					statistics_.t_launch = new_state.t;
					new_state.flight_mode = chandra::aero::protocol::TrackerFlightMode::Ascent;
				}
				break;

			case chandra::aero::protocol::TrackerFlightMode::Ascent:
				new_state.thrust = ( (new_state.t - launch_config_.launch_delay_time) < motor_config_.burn_time) ? motor_config_.average_thrust : force_t{ 0 };
				new_state.drag = force_t(0.5 * rho_ * (v_mag * v_mag) * reference_area_ * rocket_config_.cd);

				total_forces.x = (state_.pos.z < 1.5_m_) ? force_t{ 0 } : force_t(sign_x * lateral_force_constant * (dv_x * dv_x));
				total_forces.y = (state_.pos.z < 1.5_m_) ? force_t{ 0 } : force_t(sign_y * lateral_force_constant * (dv_y * dv_y));
				total_forces.z = (new_state.thrust - new_state.drag);
				
				new_state.accel = total_forces / rocket_config_.mass;
				new_state.accel.z = new_state.accel.z - new_state.g;

				statistics_.update_max_accel(new_state.accel);

				new_state.vel = new_state.vel + dt*new_state.accel;
				statistics_.update_max_vel(new_state.vel);

				new_state.pos = new_state.pos + pos_t(dt * new_state.vel); // TODO: FIGURE OUT WHY THESE CASTS ARE NECESSARY... IT PROBABLY HAS TO DO WITH FRAME NOT PROPOGATIN CORRECTLY
				statistics_.update_max_alt(new_state.pos(2));

				if ( (new_state.flight_mode == chandra::aero::protocol::TrackerFlightMode::Ascent) &&
					(state_.pos(2) < new_state.pos(2) ) ) {	// Have not reached apogee
					statistics_.maximum_altitude = new_state.pos(2);
				} else { // Apogee Detected
					statistics_.t_apogee = new_state.t;
					new_state.flight_mode = chandra::aero::protocol::TrackerFlightMode::Descent;
				}
				break;

			case chandra::aero::protocol::TrackerFlightMode::Descent:
				new_state.drag = force_t(0.5 * rho_ * (v_mag * v_mag) * reference_area_ * rocket_config_.cd);

				total_forces.x = sign_x * lateral_force_constant * (dv_x * dv_x);
				total_forces.y = sign_y * lateral_force_constant * (dv_y * dv_y);
				total_forces.z = force_t(0.5 * rho_ * (state_.vel.z * state_.vel.z) * parachute_area_ * rocket_config_.parachute_cd);

				if (new_state.pos.z < 150_m_) { // Simulate Dual Deploy
					total_forces.z = total_forces.z * 5;
				}

				new_state.accel = total_forces / rocket_config_.mass;
				new_state.accel.z = new_state.accel.z - new_state.g;

				statistics_.update_max_accel(new_state.accel);

				new_state.vel = new_state.vel + dt*new_state.accel;
				new_state.pos = new_state.pos + pos_t(dt * new_state.vel); // TODO: FIGURE OUT WHY THESE CASTS ARE NECESSARY... IT PROBABLY HAS TO DO WITH FRAME NOT PROPOGATIN CORRECTLY

				if (new_state.pos(2) <= 0_m_) { // Detect landing
					new_state.pos(2) = 0_m_;
					new_state.flight_mode = chandra::aero::protocol::TrackerFlightMode::Postflight;
				}
				break;

			case chandra::aero::protocol::TrackerFlightMode::Postflight:
				new_state.vel = velocity_vector_t{ 0_m_per_s_, 0_m_per_s_, 0_m_per_s_ };
				new_state.accel = acceleration_vector_t{ 0_gees_, 0_gees_, -1_gees_ };
				break;
		}


		state_ = new_state;
		return state_;
	}

	auto statistics() const {
		return statistics_;
	}

	auto reference_position() const {
		return reference_position_;
	}

private:
	launch_config_t launch_config_;
	rocket_config_t rocket_config_;
	motor_config_t motor_config_;
	WindGenerator<velocity_t, angle_t>& wind_gen_;
	state_t state_;
	statistics_t statistics_;
	pos_t reference_position_;
	density_t rho_ = 1.225_kg_per_m3_;
	area_t reference_area_;
	area_t lateral_reference_area_;
	area_t parachute_area_;
};

template<
	class SimulationStateType
>
struct SplashdownEstimationTimestep
{
	using simulation_state_t = SimulationStateType;
	using time_t = typename simulation_state_t::time_t;
	using pos_t = typename simulation_state_t::pos_vec_t;
	using vel_vector_t = typename simulation_state_t::vel_vec_t;
	using accel_vector_t = typename simulation_state_t::accel_vec_t;

	using velocity_t = typename vel_vector_t::value_t;
	using acceleration_t = typename accel_vector_t::value_t;

	using ecef_pos_t = chandra::aero::ECEF<value_t>;
	using ecef_vel_t = chandra::math::Vector3D<velocity_t, true, typename ecef_pos_t::frame_t>;
	using ecef_accel_t = chandra::math::Vector3D<acceleration_t, true, typename ecef_pos_t::frame_t>;

	time_t t{ 0 };
	chandra::aero::protocol::TrackerFlightMode flight_mode = chandra::aero::protocol::TrackerFlightMode::Invalid;
	FlightEvent detected_event = FlightEvent::None;
	time_t detected_event_latency = 0_s_;
	bool tracker_gps_updated = false;
	bool tracker_imu_updated = false;
	bool splashdown_prediction_updated = false;
	vel_vector_t wind_vel;

	pos_t sim_pos;
	vel_vector_t sim_vel;
	accel_vector_t sim_accel;

	ecef_pos_t meas_pos;
	pos_t meas_pos_enu;
	ecef_accel_t meas_accel;
	accel_vector_t meas_accel_enu;

	ecef_pos_t estimated_pos;
	pos_t estimated_pos_enu;
	ecef_vel_t estimated_vel;
	vel_vector_t estimated_vel_enu;

	ecef_pos_t pred_pos;
	pos_t pred_pos_enu;
};

template<class Stream, class Vec3D>
void vec3_to_csv(Stream& _stream, const Vec3D& _vec) {
	_stream << _vec(0) << ", ";
	_stream << _vec(1) << ", ";
	_stream << _vec(2);
}

template<class Stream, class SimulationStateType>
Stream& operator << (Stream& _stream, const SplashdownEstimationTimestep<SimulationStateType>& _data) {
	_stream << _data.t << ", ";
	_stream << _data.flight_mode << ", ";
	_stream << _data.detected_event << ", ";
	_stream << _data.detected_event_latency << ", ";

	_stream << _data.tracker_gps_updated << ", ";
	_stream << _data.tracker_imu_updated << ", ";
	_stream << _data.splashdown_prediction_updated << ", ";
	vec3_to_csv(_stream, _data.wind_vel);
	_stream << ", ";
	vec3_to_csv(_stream, _data.sim_pos);
	_stream << ", ";
	vec3_to_csv(_stream, _data.sim_vel);
	_stream << ", ";
	vec3_to_csv(_stream, _data.sim_accel);
	_stream << ", ";

	vec3_to_csv(_stream, _data.meas_pos);
	_stream << ", ";
	vec3_to_csv(_stream, _data.meas_pos_enu);
	_stream << ", ";
	vec3_to_csv(_stream, _data.meas_accel);
	_stream << ", ";
	vec3_to_csv(_stream, _data.meas_accel_enu);
	_stream << ", ";

	vec3_to_csv(_stream, _data.estimated_pos);
	_stream << ", ";
	vec3_to_csv(_stream, _data.estimated_pos_enu);
	_stream << ", ";
	vec3_to_csv(_stream, _data.estimated_vel);
	_stream << ", ";
	vec3_to_csv(_stream, _data.estimated_vel_enu);
	_stream << ", ";

	vec3_to_csv(_stream, _data.pred_pos);
	_stream << ", ";
	vec3_to_csv(_stream, _data.pred_pos_enu);

	return _stream;
}

//
// Test Configuration
//
static std::string data_filename("splashdown_sim.dat");

static uint8_t vehicle_id = 0;
static uint8_t tracker_id = 1;

static auto t_max_runtime = 100_s_; // TODO: NEED TO MAKE THE USER DEFINED LITERAL FUNCTIONS CONSTEXPR SO THAT THESE CONFIGURATIONS CAN ALL BE CONSTEXPR
static auto t_sim_update = 1_ms_;
static auto t_gnss_sample = 0.2_s_;
static auto t_imu_sample = 10_ms_;
static auto t_detector_sample = 100_ms_;
static auto t_landing_delay = 5_s_;
static auto mass = 200_g_;
static auto diameter = 41.6_mm_;
static auto length = 79.8_cm_;
static auto cd = value_t(0.55);

static auto average_thrust = 11.2_N_;
static auto t_burn = 2.4_s_;

static auto v_descent = 5.33_m_per_s_;

static auto p_pad = chandra::aero::LLH<value_t>{ 38_deg_, -104_deg_, 1850_m_ };
static auto t_launch_delay = 10_s_;

int main()
{
	//
	// System Initialization
	//
	chandra::chrono::Timer<system_clock_t> sim_update_timer(t_sim_update);
	chandra::chrono::Timer<system_clock_t> gnss_sample_timer(t_gnss_sample);
	chandra::chrono::Timer<system_clock_t> imu_sample_timer(t_imu_sample);
	chandra::chrono::Timer<system_clock_t> detector_sample_timer(t_detector_sample);

	//
	// Initialze the system clock
	//
	system_clock_t::init();

	//
	// Main Loop
	//
	using gnss_status_t = chandra::aero::protocol::GNSSStatus;
	using tracker_estimator_t = chandra::aero::protocol::TrackerStateEstimator<value_t>;
	using frame_scheduler_t = chandra::aero::protocol::TrackerFrameScheduler<value_t, tracker_estimator_t, system_clock_t>;
	using basestation_estimator_t = chandra::aero::protocol::BasestationTrackingState<value_t>;

	using simulator_t = LaunchSimulator<value_t>;
	using launch_config_t = typename simulator_t::launch_config_t;
	using rocket_config_t = typename simulator_t::rocket_config_t;
	using motor_config_t = typename simulator_t::motor_config_t;
	using timestep_data_t = SplashdownEstimationTimestep<typename simulator_t::state_t>;
	using datalog_t = std::vector<timestep_data_t>;

	const auto t_start = system_clock_t::now();
	auto uptime = UptimeTracker < value_t, system_clock_t >();
	launch_config_t launch_config;
	launch_config.pad_position = p_pad;
	launch_config.launch_delay_time = t_launch_delay;
	rocket_config_t rocket_config;
	rocket_config.diameter = diameter;
	rocket_config.length = length;
	rocket_config.parachute_diameter = 0.2_m_;
	rocket_config.mass = mass;
	rocket_config.cd = cd;
	rocket_config.lateral_cd = 1.0;
	rocket_config.parachute_cd = 0.8;
	rocket_config.descent_terminal = v_descent;
	motor_config_t motor_config;
	motor_config.average_thrust = average_thrust;
	motor_config.burn_time = t_burn;
	
	gnss_status_t gnss_status;
	chandra::aero::protocol::TrackerGNSSSample<value_t> gnss_sample;
	tracker_estimator_t tracking_estimator(vehicle_id, tracker_id);
	chandra::rocketry::LiftoffDetector<value_t, system_clock_t> liftoff_detector;
	chandra::rocketry::BurnoutDetector<value_t, system_clock_t> burnout_detector;
	const auto detector_smoothing_param = 0.2;
	chandra::rocketry::ApogeeDetector<value_t, system_clock_t> apogee_detector(detector_smoothing_param); // Note: By default the apogee and landing detectors use an exponential filter
	chandra::rocketry::LandingDetector<value_t, system_clock_t> landing_detector(detector_smoothing_param);

	FlightEvent detected_event = FlightEvent::None;
	frame_scheduler_t frame_scheduler;

	basestation_estimator_t basestation_estimator;

	unsigned int seed = 0x12345678;
	std::mt19937 rnd(seed);
	GNSSErrorModel<chandra::units::mks::Q_m<value_t>> gnss_error_model(rnd, 0.4_m_, 0.8_m_);
	WindGenerator<chandra::units::mks::Q_m_per_s<value_t>, chandra::units::mks::Q_rad<value_t>> wind_gen(rnd, 8_m_per_s_, 22_deg_);
	simulator_t sim(launch_config, rocket_config, motor_config, wind_gen);

	std::cout << "Configuration:\n";
	std::cout << "\tTiming:\n\t\tt_max_runtime = " << t_max_runtime << "\n\t\tt_sim_update = " << t_sim_update << "\n\t\tt_gnss_sample = " << t_gnss_sample << "\n\t\tt_imu_sample = " << t_imu_sample << "\n";
	std::cout << "\tRocket:\n\t\trocket mass = " << mass << "\n\t\tdiameter = " << diameter << "\n\t\t CD = " << cd << "\n";
	std::cout << "\tMotor:\n\t\taveragethrust = " << average_thrust << "\n\t\tt_burn = " << t_burn << "\n";
	std::cout << "\tDescent:\n\t\tdescent rate = " << v_descent << "\n";
	std::cout << "\tLaunch:\n\t\tpad position = " << p_pad << "\n";

	typename simulator_t::state_t state;

	bool landed = false;
	auto t_landed = 0_s_;

	size_t sim_updates = 0;
	size_t gnss_updates = 0;
	size_t imu_updates = 0;
	size_t splashdown_prediction_updates = 0;
	const auto ecef_base = chandra::aero::LLHToECEF(p_pad);

	datalog_t datalog;

	std::cout << "\nRun Splashdown Estimation\n";
	sim.init(uptime());
	tracking_estimator.init();
	frame_scheduler.init();
	basestation_estimator.init();
	basestation_estimator.base_update(ecef_base);
	basestation_estimator.tracker_update(ecef_base);

	auto t_last = uptime();
	bool done = false;

	simulator_t::pos_t gnss_noise;
	simulator_t::acceleration_vector_t accel_noise;
	bool filters_settled = false;

	while((uptime() <= t_max_runtime) && !done){
		timestep_data_t timestep_data;
		timestep_data.t = uptime();

		state = sim.update(timestep_data.t);
		++sim_updates;
		timestep_data.wind_vel = state.wind_vel;
		timestep_data.flight_mode = state.flight_mode;
		timestep_data.sim_pos = state.pos;
		timestep_data.sim_vel = state.vel;
		timestep_data.sim_accel = state.accel;


		// NOTE: THIS IS WHERE A NOISE MODEL ON THE SENSORS SHOULD BE USED...
		timestep_data.meas_pos_enu = simulator_t::pos_t(state.pos + gnss_noise);
		timestep_data.meas_pos = chandra::aero::ENUToECEF(timestep_data.meas_pos_enu, ecef_base);
		state.accel.z = state.accel.z - state.g; // TODO: THE MEASURED ACCELERATION ALSO INCLUDES GRAVITY....
		timestep_data.meas_accel_enu = simulator_t::acceleration_vector_t(state.accel + accel_noise);
		timestep_data.meas_accel = chandra::aero::RotateENUToECEF(timestep_data.meas_accel_enu, ecef_base);

		if (gnss_sample_timer()) { // UPDATE TRACKER STATE
			gnss_sample.pos = timestep_data.meas_pos;
			gnss_sample.t_since_last = t_gnss_sample;

			++gnss_updates;
			timestep_data.tracker_gps_updated = true;
			tracking_estimator.update_gnss(gnss_sample);
			gnss_noise = gnss_error_model();
		}

		if (imu_sample_timer()) { // UPDATE TRACKER STATE
			++imu_updates;
			timestep_data.tracker_imu_updated = true;
			tracking_estimator.update_accel(timestep_data.meas_accel);
		}

		filters_settled = gnss_updates > 10;

		if (filters_settled && tracking_estimator.valid()) { // TODO: THIS SHOULD LOOK AT IF THE FILTER HAS (APPARENTLY) SETTLED INSIDE THE VALID() METHOD....
			timestep_data.estimated_pos = tracking_estimator.pos(); // TODO: FOR BETTER SIMULATION, THIS SHOULD BE ENCODED AND DECODED AGAIN THROUGH THE PROTOCOL....
			timestep_data.estimated_vel = tracking_estimator.vel();
		}
		else {
			timestep_data.estimated_pos = ecef_base;
			timestep_data.estimated_vel = simulator_t::velocity_vector_t();
		}

		timestep_data.estimated_pos_enu = chandra::aero::ECEFToENU(timestep_data.estimated_pos, ecef_base);
		timestep_data.estimated_vel_enu = chandra::aero::RotateECEFToENU(timestep_data.estimated_vel, ecef_base);

		const auto scheduled_frame = frame_scheduler(gnss_status, tracking_estimator);
		if (scheduled_frame == chandra::aero::protocol::TrackerFrameFormats::Localization) {
			basestation_estimator.tracker_update(timestep_data.estimated_pos, timestep_data.estimated_vel);
			if (basestation_estimator.projected_tracker_valid()) {
				timestep_data.splashdown_prediction_updated = true;
				++splashdown_prediction_updates;
			}
		}

		timestep_data.pred_pos = basestation_estimator.projected_tracker_pos();
		timestep_data.pred_pos_enu = chandra::aero::ECEFToENU(timestep_data.pred_pos, ecef_base);

		const auto dt = timestep_data.t - t_last;
		t_last = timestep_data.t;

		timestep_data.detected_event = FlightEvent::None;
		if (filters_settled && detector_sample_timer()) {
			const auto liftoff_detector_result = liftoff_detector(t_detector_sample, timestep_data.meas_accel.z + state.g);
			const auto burnout_detector_result = burnout_detector(t_detector_sample, timestep_data.meas_accel.z + state.g);
			const auto enu_pos = chandra::aero::ECEFToENU(timestep_data.estimated_pos, ecef_base); // Use The estimated position....
 			const auto apogee_detector_result = apogee_detector(t_detector_sample, enu_pos.z);
			const auto landing_detector_result = landing_detector(t_detector_sample, enu_pos.z);

			if (bool(liftoff_detector_result)) {
				std::cout << "Liftoff detected at " << timestep_data.t << "\n";
				timestep_data.detected_event = FlightEvent::Liftoff;
				timestep_data.detected_event_latency = liftoff_detector_result.latency;
			}
			else if (bool(burnout_detector_result)) {
				timestep_data.detected_event = FlightEvent::Burnout;
				timestep_data.detected_event_latency = burnout_detector_result.latency;
			}
			else if (bool(apogee_detector_result)) {
				timestep_data.detected_event = FlightEvent::Apogee;
				timestep_data.detected_event_latency = apogee_detector_result.latency;
			}
			else if (bool(landing_detector_result)) {
				timestep_data.detected_event = FlightEvent::Landing;
				timestep_data.detected_event_latency = landing_detector_result.latency;
				landed = true;
				t_landed = uptime();
			}
		}

		if (landed) {
			done = uptime() >= (t_landed + t_landing_delay);
		//	std::cout << "landed = " << landed << ", t_landed = " << t_landed << ", t_landing_delay = " << t_landing_delay << ", uptime() = " << uptime() << ", done = " << done << "\n";
		}

		datalog.push_back(timestep_data);
		system_clock_t::advance(t_sim_update);
	}

	std::cout << state << "\n";
	std::cout << sim_updates << " Simulation Updates, " << gnss_updates << " GNSS Updates, " << imu_updates << " IMU Updates and " << splashdown_prediction_updates << " Splashdown Prediction Updates\n";
	std::cout << "Writing data file...";
	
	std::ofstream data_file;
	data_file.open(data_filename);
	data_file << "; Splashdown estimation datalog (simulation)\n";
	for (auto& data : datalog) {
		data_file << data;
		data_file << "\n";

//		const auto enu_pos = chandra::aero::ECEFToENU(data.meas_pos, ecef_base);
//		const auto enu_vel = chandra::aero::RotateECEFToENU(data.estimated_vel, ecef_base);
//		data_file << "; *** ";
//		vec3_to_csv(data_file, enu_pos);
//		data_file << ", ";
//		vec3_to_csv(data_file, enu_vel);
//		data_file << " ***\n";
	}

	data_file.close();

	std::cout << "Complete!\n";

	std::cout << sim.statistics() << "\n";
	return 0;
}