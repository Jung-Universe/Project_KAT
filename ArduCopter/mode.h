#pragma once

#include "Copter.h"
#include <AP_Math/chirp.h>
#include <AP_ExternalControl/AP_ExternalControl_config.h> // TODO why is this needed if Copter.h includes this

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
#include "afs_copter.h"
#endif

class Parameters;
class ParametersG2;

class GCS_Copter;

// object shared by both Guided and Auto for takeoff.
// position controller controls vehicle but the user can control the yaw.
class _AutoTakeoff {
public:
    void run();
    void start(float complete_alt_cm, bool terrain_alt);
    bool get_completion_pos(Vector3p& pos_neu_cm);

    bool complete;          // true when takeoff is complete

private:
    // altitude above-ekf-origin below which auto takeoff does not control horizontal position
    bool no_nav_active;
    float no_nav_alt_cm;

    // auto takeoff variables
    float complete_alt_cm;  // completion altitude expressed in cm above ekf origin or above terrain (depending upon auto_takeoff_terrain_alt)
    bool terrain_alt;       // true if altitudes are above terrain
    Vector3p complete_pos;  // target takeoff position as offset from ekf origin in cm
};

#if AC_PAYLOAD_PLACE_ENABLED
class PayloadPlace {
public:
    void run();
    void start_descent();
    bool verify();

    enum class State : uint8_t {
        FlyToLocation,
        Descent_Start,
        Descent,
        Release,
        Releasing,
        Delay,
        Ascent_Start,
        Ascent,
        Done,
    };

    // these are set by the Mission code:
    State state = State::Descent_Start; // records state of payload place
    float descent_max_cm;

private:

    uint32_t descent_established_time_ms; // milliseconds
    uint32_t place_start_time_ms; // milliseconds
    float descent_thrust_level;
    float descent_start_altitude_cm;
    float descent_speed_cms;
};
#endif

class Mode {
    friend class PayloadPlace;

public:

    // Auto Pilot Modes enumeration
    enum class Number : uint8_t {
        STABILIZE =     0,  // manual airframe angle with manual throttle
        ACRO =          1,  // manual body-frame angular rate with manual throttle
        ALT_HOLD =      2,  // manual airframe angle with automatic throttle
        AUTO =          3,  // fully automatic waypoint control using mission commands
        GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
        LOITER =        5,  // automatic horizontal acceleration with automatic throttle
        RTL =           6,  // automatic return to launching point
        CIRCLE =        7,  // automatic circular flight with automatic throttle
        LAND =          9,  // automatic landing with horizontal position control
        DRIFT =        11,  // semi-autonomous position, yaw and throttle control
        SPORT =        13,  // manual earth-frame angular rate control with manual throttle
        FLIP =         14,  // automatically flip the vehicle on the roll axis
        AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
        POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
        BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
        THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
        AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
        GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
        SMART_RTL =    21,  // SMART_RTL returns to home by retracing its steps
        FLOWHOLD  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
        FOLLOW    =    23,  // follow attempts to follow another vehicle or ground station
        ZIGZAG    =    24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
        SYSTEMID  =    25,  // System ID mode produces automated system identification signals in the controllers
        AUTOROTATE =   26,  // Autonomous autorotation
        AUTO_RTL =     27,  // Auto RTL, this is not a true mode, AUTO will report as this mode if entered to perform a DO_LAND_START Landing sequence
        TURTLE =       28,  // Flip over after crash
        TDCN =         29,  /* Sejong */ // external CLAW controller drives the vehicle (shipboard landing)

        // Mode number 127 reserved for the "drone show mode" in the Skybrush
        // fork at https://github.com/skybrush-io/ardupilot
    };

    // constructor
    Mode(void);

    // do not allow copying
    CLASS_NO_COPY(Mode);

    friend class _AutoTakeoff;

    // returns a unique number specific to this mode
    virtual Number mode_number() const = 0;

    // child classes should override these methods
    virtual bool init(bool ignore_checks) {
        return true;
    }
    virtual void exit() {};
    virtual void run() = 0;
    virtual bool requires_GPS() const = 0;
    virtual bool has_manual_throttle() const = 0;
    virtual bool allows_arming(AP_Arming::Method method) const = 0;
    virtual bool is_autopilot() const { return false; }
    virtual bool has_user_takeoff(bool must_navigate) const { return false; }
    virtual bool in_guided_mode() const { return false; }
    virtual bool logs_attitude() const { return false; }
    virtual bool allows_save_trim() const { return false; }
    virtual bool allows_autotune() const { return false; }
    virtual bool allows_flip() const { return false; }
    virtual bool crash_check_enabled() const { return true; }

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    // Return the type of this mode for use by advanced failsafe
    virtual AP_AdvancedFailsafe_Copter::control_mode afs_mode() const { return AP_AdvancedFailsafe_Copter::control_mode::AFS_STABILIZED; }
#endif

    // Return true if the throttle high arming check can be skipped when arming from GCS or Scripting
    virtual bool allows_GCS_or_SCR_arming_with_throttle_high() const { return false; }

#if FRAME_CONFIG == HELI_FRAME
    virtual bool allows_inverted() const { return false; };
#endif

    // return a string for this flightmode
    virtual const char *name() const = 0;
    virtual const char *name4() const = 0;

    bool do_user_takeoff(float takeoff_alt_cm, bool must_navigate);
    virtual bool is_taking_off() const;
    static void takeoff_stop() { takeoff.stop(); }

    virtual bool is_landing() const { return false; }

    // mode requires terrain to be present to be functional
    virtual bool requires_terrain_failsafe() const { return false; }

    // functions for reporting to GCS
    virtual bool get_wp(Location &loc) const { return false; };
    virtual int32_t wp_bearing() const { return 0; }
    virtual uint32_t wp_distance() const { return 0; }
    virtual float crosstrack_error() const { return 0.0f;}

    // functions to support MAV_CMD_DO_CHANGE_SPEED
    virtual bool set_speed_xy(float speed_xy_cms) {return false;}
    virtual bool set_speed_up(float speed_xy_cms) {return false;}
    virtual bool set_speed_down(float speed_xy_cms) {return false;}

    int32_t get_alt_above_ground_cm(void);

    // pilot input processing
    void get_pilot_desired_lean_angles(float &roll_out_cd, float &pitch_out_cd, float angle_max_cd, float angle_limit_cd) const;
    Vector2f get_pilot_desired_velocity(float vel_max) const;
    float get_pilot_desired_yaw_rate() const;
    float get_pilot_desired_throttle() const;

    // returns climb target_rate reduced to avoid obstacles and
    // altitude fence
    float get_avoidance_adjusted_climbrate(float target_rate);

    // send output to the motors, can be overridden by subclasses
    virtual void output_to_motors();

    // returns true if pilot's yaw input should be used to adjust vehicle's heading
    virtual bool use_pilot_yaw() const {return true; }

    // pause and resume a mode
    virtual bool pause() { return false; };
    virtual bool resume() { return false; };

    // handle situations where the vehicle is on the ground waiting for takeoff
    void make_safe_ground_handling(bool force_throttle_unlimited = false);

    // true if weathervaning is allowed in the current mode
#if WEATHERVANE_ENABLED
    virtual bool allows_weathervaning() const { return false; }
#endif

protected:

    // helper functions
    bool is_disarmed_or_landed() const;
    void zero_throttle_and_relax_ac(bool spool_up = false);
    void zero_throttle_and_hold_attitude();

    // Return stopping point as a location with above origin alt frame
    Location get_stopping_point() const;

    // functions to control normal landing.  pause_descent is true if vehicle should not descend
    void land_run_horizontal_control();
    void land_run_vertical_control(bool pause_descent = false);
    void land_run_horiz_and_vert_control(bool pause_descent = false) {
        land_run_horizontal_control();
        land_run_vertical_control(pause_descent);
    }

#if AC_PAYLOAD_PLACE_ENABLED
    // payload place flight behaviour:
    static PayloadPlace payload_place;
#endif

    // run normal or precision landing (if enabled)
    // pause_descent is true if vehicle should not descend
    void land_run_normal_or_precland(bool pause_descent = false);

#if AC_PRECLAND_ENABLED
    // Go towards a position commanded by prec land state machine in order to retry landing
    // The passed in location is expected to be NED and in meters
    void precland_retry_position(const Vector3f &retry_pos);

    // Run precland statemachine. This function should be called from any mode that wants to do precision landing.
    // This handles everything from prec landing, to prec landing failures, to retries and failsafe measures
    void precland_run();
#endif

    // return expected input throttle setting to hover:
    virtual float throttle_hover() const;

    // Alt_Hold based flight mode states used in Alt_Hold, Loiter, and Sport
    enum class AltHoldModeState {
        MotorStopped,
        Takeoff,
        Landed_Ground_Idle,
        Landed_Pre_Takeoff,
        Flying
    };
    AltHoldModeState get_alt_hold_state(float target_climb_rate_cms);

    // convenience references to avoid code churn in conversion:
    Parameters &g;
    ParametersG2 &g2;
    AC_WPNav *&wp_nav;
    AC_Loiter *&loiter_nav;
    AC_PosControl *&pos_control;
    AP_InertialNav &inertial_nav;
    AP_AHRS &ahrs;
    AC_AttitudeControl *&attitude_control;
    MOTOR_CLASS *&motors;
    RC_Channel *&channel_roll;
    RC_Channel *&channel_pitch;
    RC_Channel *&channel_throttle;
    RC_Channel *&channel_yaw;
    float &G_Dt;

    // note that we support two entirely different automatic takeoffs:

    // "user-takeoff", which is available in modes such as ALT_HOLD
    // (see has_user_takeoff method).  "user-takeoff" is a simple
    // reach-altitude-based-on-pilot-input-or-parameter routine.

    // "auto-takeoff" is used by both Guided and Auto, and is
    // basically waypoint navigation with pilot yaw permitted.

    // user-takeoff support; takeoff state is shared across all mode instances
    class _TakeOff {
    public:
        void start(float alt_cm);
        void stop();
        void do_pilot_takeoff(float& pilot_climb_rate);
        bool triggered(float target_climb_rate) const;

        bool running() const { return _running; }
    private:
        bool _running;
        float take_off_start_alt;
        float take_off_complete_alt;
    };

    static _TakeOff takeoff;

    virtual bool do_user_takeoff_start(float takeoff_alt_cm);

    static _AutoTakeoff auto_takeoff;

public:
    // Navigation Yaw control
    class AutoYaw {

    public:

        // Autopilot Yaw Mode enumeration
        enum class Mode {
            HOLD =             0,  // hold zero yaw rate
            LOOK_AT_NEXT_WP =  1,  // point towards next waypoint (no pilot input accepted)
            ROI =              2,  // point towards a location held in roi (no pilot input accepted)
            FIXED =            3,  // point towards a particular angle (no pilot input accepted)
            LOOK_AHEAD =       4,  // point in the direction the copter is moving
            RESETTOARMEDYAW =  5,  // point towards heading at time motors were armed
            ANGLE_RATE =       6,  // turn at a specified rate from a starting angle
            RATE =             7,  // turn at a specified rate (held in auto_yaw_rate)
            CIRCLE =           8,  // use AC_Circle's provided yaw (used during Loiter-Turns commands)
            PILOT_RATE =       9,  // target rate from pilot stick
            WEATHERVANE =     10,  // yaw into wind
        };

        // mode(): current method of determining desired yaw:
        Mode mode() const { return _mode; }
        void set_mode_to_default(bool rtl);
        void set_mode(Mode new_mode);
        Mode default_mode(bool rtl) const;

        void set_rate(float new_rate_cds);

        // set_roi(...): set a "look at" location:
        void set_roi(const Location &roi_location);

        void set_fixed_yaw(float angle_deg,
                           float turn_rate_dps,
                           int8_t direction,
                           bool relative_angle);

        void set_yaw_angle_rate(float yaw_angle_d, float yaw_rate_ds);

        void set_yaw_angle_offset(const float yaw_angle_offset_d);

        bool reached_fixed_yaw_target();

#if WEATHERVANE_ENABLED
        void update_weathervane(const int16_t pilot_yaw_cds);
#endif

        AC_AttitudeControl::HeadingCommand get_heading();

    private:

        // yaw_cd(): main product of AutoYaw; the heading:
        float yaw_cd();

        // rate_cds(): desired yaw rate in centidegrees/second:
        float rate_cds();

        // returns a yaw in degrees, direction of vehicle travel:
        float look_ahead_yaw();

        float roi_yaw() const;

        // auto flight mode's yaw mode
        Mode _mode = Mode::LOOK_AT_NEXT_WP;
        Mode _last_mode;

        // Yaw will point at this location if mode is set to Mode::ROI
        Vector3f roi;

        // yaw used for YAW_FIXED yaw_mode
        float _fixed_yaw_offset_cd;

        // Deg/s we should turn
        float _fixed_yaw_slewrate_cds;

        // time of the last yaw update
        uint32_t _last_update_ms;

        // heading when in yaw_look_ahead_yaw
        float _look_ahead_yaw;

        // turn rate (in cds) when auto_yaw_mode is set to AUTO_YAW_RATE
        float _yaw_angle_cd;
        float _yaw_rate_cds;
        float _pilot_yaw_rate_cds;
    };
    static AutoYaw auto_yaw;

    // pass-through functions to reduce code churn on conversion;
    // these are candidates for moving into the Mode base
    // class.
    float get_pilot_desired_climb_rate(float throttle_control);
    float get_non_takeoff_throttle(void);
    void update_simple_mode(void);
    bool set_mode(Mode::Number mode, ModeReason reason);
    void set_land_complete(bool b);
    GCS_Copter &gcs();
    uint16_t get_pilot_speed_dn(void);
    // end pass-through functions
};


#if MODE_ACRO_ENABLED
class ModeAcro : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::ACRO; }

    enum class Trainer {
        OFF = 0,
        LEVELING = 1,
        LIMITED = 2,
    };

    enum class AcroOptions {
        AIR_MODE = 1 << 0,
        RATE_LOOP_ONLY = 1 << 1,
    };

    virtual void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return true; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool init(bool ignore_checks) override;
    void exit() override;
    // whether an air-mode aux switch has been toggled
    void air_mode_aux_changed();
    bool allows_save_trim() const override { return true; }
    bool allows_flip() const override { return true; }
    bool crash_check_enabled() const override { return false; }

protected:

    const char *name() const override { return "ACRO"; }
    const char *name4() const override { return "ACRO"; }

    // get_pilot_desired_angle_rates - transform pilot's normalised roll pitch and yaw input into a desired lean angle rates
    // inputs are -1 to 1 and the function returns desired angle rates in centi-degrees-per-second
    void get_pilot_desired_angle_rates(float roll_in, float pitch_in, float yaw_in, float &roll_out, float &pitch_out, float &yaw_out);

    float throttle_hover() const override;

private:
    bool disable_air_mode_reset;
};
#endif

#if FRAME_CONFIG == HELI_FRAME
class ModeAcro_Heli : public ModeAcro {

public:
    // inherit constructor
    using ModeAcro::Mode;

    bool init(bool ignore_checks) override;
    void run() override;
    void virtual_flybar( float &roll_out, float &pitch_out, float &yaw_out, float pitch_leak, float roll_leak);

protected:
private:
};
#endif


class ModeAltHold : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::ALT_HOLD; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override {
        return !must_navigate;
    }
    bool allows_autotune() const override { return true; }
    bool allows_flip() const override { return true; }
#if FRAME_CONFIG == HELI_FRAME
    bool allows_inverted() const override { return true; };
#endif
protected:

    const char *name() const override { return "ALT_HOLD"; }
    const char *name4() const override { return "ALTH"; }

private:

};

class ModeAuto : public Mode {

public:
    friend class PayloadPlace;  // in case wp_run is accidentally required

    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return auto_RTL? Number::AUTO_RTL : Number::AUTO; }

    bool init(bool ignore_checks) override;
    void exit() override;
    void run() override;

    bool requires_GPS() const override;
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override;
    bool is_autopilot() const override { return true; }
    bool in_guided_mode() const override { return _mode == SubMode::NAVGUIDED || _mode == SubMode::NAV_SCRIPT_TIME; }
#if FRAME_CONFIG == HELI_FRAME
    bool allows_inverted() const override { return true; };
#endif

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    // Return the type of this mode for use by advanced failsafe
    AP_AdvancedFailsafe_Copter::control_mode afs_mode() const override { return AP_AdvancedFailsafe_Copter::control_mode::AFS_AUTO; }
#endif

    // Return true if the throttle high arming check can be skipped when arming from GCS or Scripting
    bool allows_GCS_or_SCR_arming_with_throttle_high() const override { return true; }

    // Auto modes
    enum class SubMode : uint8_t {
        TAKEOFF,
        WP,
        LAND,
        RTL,
        CIRCLE_MOVE_TO_EDGE,
        CIRCLE,
        NAVGUIDED,
        LOITER,
        LOITER_TO_ALT,
#if AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED && AC_PAYLOAD_PLACE_ENABLED
        NAV_PAYLOAD_PLACE,
#endif
        NAV_SCRIPT_TIME,
        NAV_ATTITUDE_TIME,
    };

    // set submode.  returns true on success, false on failure
    void set_submode(SubMode new_submode);

    // pause continue in auto mode
    bool pause() override;
    bool resume() override;
    bool paused() const;

    bool loiter_start();
    void rtl_start();
    void takeoff_start(const Location& dest_loc);
    bool wp_start(const Location& dest_loc);
    void land_start();
    void circle_movetoedge_start(const Location &circle_center, float radius_m, bool ccw_turn);
    void circle_start();
    void nav_guided_start();

    bool is_landing() const override;

    bool is_taking_off() const override;
    bool use_pilot_yaw() const override;

    bool set_speed_xy(float speed_xy_cms) override;
    bool set_speed_up(float speed_up_cms) override;
    bool set_speed_down(float speed_down_cms) override;

    bool requires_terrain_failsafe() const override { return true; }

    void payload_place_start();

    // for GCS_MAVLink to call:
    bool do_guided(const AP_Mission::Mission_Command& cmd);

    // Go straight to landing sequence via DO_LAND_START, if succeeds pretend to be Auto RTL mode
    bool jump_to_landing_sequence_auto_RTL(ModeReason reason);

    // Join mission after DO_RETURN_PATH_START waypoint, if succeeds pretend to be Auto RTL mode
    bool return_path_start_auto_RTL(ModeReason reason);

    // Try join return path else do land start
    bool return_path_or_jump_to_landing_sequence_auto_RTL(ModeReason reason);

    // lua accessors for nav script time support
    bool nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4);
    void nav_script_time_done(uint16_t id);

    AP_Mission mission{
        FUNCTOR_BIND_MEMBER(&ModeAuto::start_command, bool, const AP_Mission::Mission_Command &),
        FUNCTOR_BIND_MEMBER(&ModeAuto::verify_command, bool, const AP_Mission::Mission_Command &),
        FUNCTOR_BIND_MEMBER(&ModeAuto::exit_mission, void)};

    // Mission change detector
    AP_Mission_ChangeDetector mis_change_detector;

    // true if weathervaning is allowed in auto
#if WEATHERVANE_ENABLED
    bool allows_weathervaning(void) const override;
#endif

protected:

    const char *name() const override { return auto_RTL? "AUTO RTL" : "AUTO"; }
    const char *name4() const override { return auto_RTL? "ARTL" : "AUTO"; }

    uint32_t wp_distance() const override;
    int32_t wp_bearing() const override;
    float crosstrack_error() const override { return wp_nav->crosstrack_error();}
    bool get_wp(Location &loc) const override;

private:

    enum class Option : int32_t {
        AllowArming                        = (1 << 0U),
        AllowTakeOffWithoutRaisingThrottle = (1 << 1U),
        IgnorePilotYaw                     = (1 << 2U),
        AllowWeatherVaning                 = (1 << 7U),
    };
    bool option_is_enabled(Option option) const;

    // Enter auto rtl pseudo mode
    bool enter_auto_rtl(ModeReason reason);

    bool start_command(const AP_Mission::Mission_Command& cmd);
    bool verify_command(const AP_Mission::Mission_Command& cmd);
    void exit_mission();

    bool check_for_mission_change();    // detect external changes to mission

    void takeoff_run();
    void wp_run();
    void land_run();
    void rtl_run();
    void circle_run();
    void nav_guided_run();
    void loiter_run();
    void loiter_to_alt_run();
    void nav_attitude_time_run();

    // return the Location portion of a command.  If the command's lat and lon and/or alt are zero the default_loc's lat,lon and/or alt are returned instead
    Location loc_from_cmd(const AP_Mission::Mission_Command& cmd, const Location& default_loc) const;

    SubMode _mode = SubMode::TAKEOFF;   // controls which auto controller is run

    bool shift_alt_to_current_alt(Location& target_loc) const;

    // subtract position controller offsets from target location
    // should be used when the location will be used as a target for the position controller
    void subtract_pos_offsets(Location& target_loc) const;

    void do_takeoff(const AP_Mission::Mission_Command& cmd);
    void do_nav_wp(const AP_Mission::Mission_Command& cmd);
    bool set_next_wp(const AP_Mission::Mission_Command& current_cmd, const Location &default_loc);
    void do_land(const AP_Mission::Mission_Command& cmd);
    void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
    void do_circle(const AP_Mission::Mission_Command& cmd);
    void do_loiter_time(const AP_Mission::Mission_Command& cmd);
    void do_loiter_to_alt(const AP_Mission::Mission_Command& cmd);
    void do_spline_wp(const AP_Mission::Mission_Command& cmd);
    void get_spline_from_cmd(const AP_Mission::Mission_Command& cmd, const Location& default_loc, Location& dest_loc, Location& next_dest_loc, bool& next_dest_loc_is_spline);
#if AC_NAV_GUIDED
    void do_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
    void do_guided_limits(const AP_Mission::Mission_Command& cmd);
#endif
    void do_nav_delay(const AP_Mission::Mission_Command& cmd);
    void do_wait_delay(const AP_Mission::Mission_Command& cmd);
    void do_within_distance(const AP_Mission::Mission_Command& cmd);
    void do_yaw(const AP_Mission::Mission_Command& cmd);
    void do_change_speed(const AP_Mission::Mission_Command& cmd);
    void do_set_home(const AP_Mission::Mission_Command& cmd);
    void do_roi(const AP_Mission::Mission_Command& cmd);
    void do_mount_control(const AP_Mission::Mission_Command& cmd);
#if HAL_PARACHUTE_ENABLED
    void do_parachute(const AP_Mission::Mission_Command& cmd);
#endif
#if AP_WINCH_ENABLED
    void do_winch(const AP_Mission::Mission_Command& cmd);
#endif
    void do_payload_place(const AP_Mission::Mission_Command& cmd);
    void do_RTL(void);
#if AP_SCRIPTING_ENABLED
    void do_nav_script_time(const AP_Mission::Mission_Command& cmd);
#endif
    void do_nav_attitude_time(const AP_Mission::Mission_Command& cmd);

    bool verify_takeoff();
    bool verify_land();
    bool verify_payload_place();
    bool verify_loiter_unlimited();
    bool verify_loiter_time(const AP_Mission::Mission_Command& cmd);
    bool verify_loiter_to_alt() const;
    bool verify_RTL();
    bool verify_wait_delay();
    bool verify_within_distance();
    bool verify_yaw();
    bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
    bool verify_circle(const AP_Mission::Mission_Command& cmd);
    bool verify_spline_wp(const AP_Mission::Mission_Command& cmd);
#if AC_NAV_GUIDED
    bool verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
#endif
    bool verify_nav_delay(const AP_Mission::Mission_Command& cmd);
#if AP_SCRIPTING_ENABLED
    bool verify_nav_script_time();
#endif
    bool verify_nav_attitude_time(const AP_Mission::Mission_Command& cmd);

    // Loiter control
    uint16_t loiter_time_max;                // How long we should stay in Loiter Mode for mission scripting (time in seconds)
    uint32_t loiter_time;                    // How long have we been loitering - The start time in millis

    struct {
        bool reached_destination_xy : 1;
        bool loiter_start_done : 1;
        bool reached_alt : 1;
        float alt_error_cm;
        int32_t alt;
    } loiter_to_alt;

    // Delay the next navigation command
    uint32_t nav_delay_time_max_ms;  // used for delaying the navigation commands (eg land,takeoff etc.)
    uint32_t nav_delay_time_start_ms;

    // Delay Mission Scripting Command
    int32_t condition_value;  // used in condition commands (eg delay, change alt, etc.)
    uint32_t condition_start;

    // Land within Auto state
    enum class State {
        FlyToLocation = 0,
        Descending = 1
    };
    State state = State::FlyToLocation;

    bool waiting_to_start;  // true if waiting for vehicle to be armed or EKF origin before starting mission

    // True if we have entered AUTO to perform a DO_LAND_START landing sequence and we should report as AUTO RTL mode
    bool auto_RTL;

#if AP_SCRIPTING_ENABLED
    // nav_script_time command variables
    struct {
        bool done;          // true once lua script indicates it has completed
        uint16_t id;        // unique id to avoid race conditions between commands and lua scripts
        uint32_t start_ms;  // system time nav_script_time command was received (used for timeout)
        uint8_t command;    // command number provided by mission command
        uint8_t timeout_s;  // timeout (in seconds) provided by mission command
        float arg1;         // 1st argument provided by mission command
        float arg2;         // 2nd argument provided by mission command
        int16_t arg3;       // 3rd argument provided by mission command
        int16_t arg4;       // 4th argument provided by mission command
    } nav_scripting;
#endif

    // nav attitude time command variables
    struct {
        int16_t roll_deg;   // target roll angle in degrees.  provided by mission command
        int8_t pitch_deg;   // target pitch angle in degrees.  provided by mission command
        int16_t yaw_deg;    // target yaw angle in degrees.  provided by mission command
        float climb_rate;   // climb rate in m/s. provided by mission command
        uint32_t start_ms;  // system time that nav attitude time command was received (used for timeout)
    } nav_attitude_time;

    // desired speeds
    struct {
        float xy;     // desired speed horizontally in m/s. 0 if unset
        float up;     // desired speed upwards in m/s. 0 if unset
        float down;   // desired speed downwards in m/s. 0 if unset
    } desired_speed_override;
};

#if AUTOTUNE_ENABLED
/*
  wrapper class for AC_AutoTune
 */

#if FRAME_CONFIG == HELI_FRAME
class AutoTune : public AC_AutoTune_Heli
#else
class AutoTune : public AC_AutoTune_Multi
#endif
{
public:
    bool init() override;
    void run() override;

protected:
    bool position_ok() override;
    float get_pilot_desired_climb_rate_cms(void) const override;
    void get_pilot_desired_rp_yrate_cd(float &roll_cd, float &pitch_cd, float &yaw_rate_cds) override;
    void init_z_limits() override;
#if HAL_LOGGING_ENABLED
    void log_pids() override;
#endif
};

class ModeAutoTune : public Mode {

    // ParametersG2 sets a pointer within our autotune object:
    friend class ParametersG2;

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::AUTOTUNE; }

    bool init(bool ignore_checks) override;
    void exit() override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; }
    bool is_autopilot() const override { return false; }

    AutoTune autotune;

protected:

    const char *name() const override { return "AUTOTUNE"; }
    const char *name4() const override { return "ATUN"; }
};
#endif


class ModeBrake : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::BRAKE; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    bool is_autopilot() const override { return false; }

    void timeout_to_loiter_ms(uint32_t timeout_ms);

protected:

    const char *name() const override { return "BRAKE"; }
    const char *name4() const override { return "BRAK"; }

private:

    uint32_t _timeout_start;
    uint32_t _timeout_ms;

};


class ModeCircle : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::CIRCLE; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "CIRCLE"; }
    const char *name4() const override { return "CIRC"; }

    uint32_t wp_distance() const override;
    int32_t wp_bearing() const override;

private:

    // Circle
    bool speed_changing = false;     // true when the roll stick is being held to facilitate stopping at 0 rate
};


class ModeDrift : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::DRIFT; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "DRIFT"; }
    const char *name4() const override { return "DRIF"; }

private:

    float get_throttle_assist(float velz, float pilot_throttle_scaled);

};


class ModeFlip : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::FLIP; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    bool is_autopilot() const override { return false; }
    bool crash_check_enabled() const override { return false; }

protected:

    const char *name() const override { return "FLIP"; }
    const char *name4() const override { return "FLIP"; }

private:

    // Flip
    Vector3f orig_attitude;         // original vehicle attitude before flip

    enum class FlipState : uint8_t {
        Start,
        Roll,
        Pitch_A,
        Pitch_B,
        Recover,
        Abandon
    };
    FlipState _state;               // current state of flip
    Mode::Number   orig_control_mode;   // flight mode when flip was initated
    uint32_t  start_time_ms;          // time since flip began
    int8_t    roll_dir;            // roll direction (-1 = roll left, 1 = roll right)
    int8_t    pitch_dir;           // pitch direction (-1 = pitch forward, 1 = pitch back)
};


#if MODE_FLOWHOLD_ENABLED
/*
  class to support FLOWHOLD mode, which is a position hold mode using
  optical flow directly, avoiding the need for a rangefinder
 */

class ModeFlowHold : public Mode {
public:
    // need a constructor for parameters
    ModeFlowHold(void);
    Number mode_number() const override { return Number::FLOWHOLD; }

    bool init(bool ignore_checks) override;
    void run(void) override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override {
        return !must_navigate;
    }
    bool allows_flip() const override { return true; }

    static const struct AP_Param::GroupInfo var_info[];

protected:
    const char *name() const override { return "FLOWHOLD"; }
    const char *name4() const override { return "FHLD"; }

private:

    // FlowHold states
    enum FlowHoldModeState {
        FlowHold_MotorStopped,
        FlowHold_Takeoff,
        FlowHold_Flying,
        FlowHold_Landed
    };

    // calculate attitude from flow data
    void flow_to_angle(Vector2f &bf_angle);

    LowPassFilterConstDtVector2f flow_filter;

    bool flowhold_init(bool ignore_checks);
    void flowhold_run();
    void flowhold_flow_to_angle(Vector2f &angle, bool stick_input);
    void update_height_estimate(void);

    // minimum assumed height
    const float height_min = 0.1f;

    // maximum scaling height
    const float height_max = 3.0f;

    AP_Float flow_max;
    AC_PI_2D flow_pi_xy{0.2f, 0.3f, 3000, 5, 0.0025f};
    AP_Float flow_filter_hz;
    AP_Int8  flow_min_quality;
    AP_Int8  brake_rate_dps;

    float quality_filtered;

    uint8_t log_counter;
    bool limited;
    Vector2f xy_I;

    // accumulated INS delta velocity in north-east form since last flow update
    Vector2f delta_velocity_ne;

    // last flow rate in radians/sec in north-east axis
    Vector2f last_flow_rate_rps;

    // timestamp of last flow data
    uint32_t last_flow_ms;

    float last_ins_height;
    float height_offset;

    // are we braking after pilot input?
    bool braking;

    // last time there was significant stick input
    uint32_t last_stick_input_ms;
};
#endif // MODE_FLOWHOLD_ENABLED


class ModeGuided : public Mode {

public:
#if AP_EXTERNAL_CONTROL_ENABLED
    friend class AP_ExternalControl_Copter;
#endif

    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::GUIDED; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override;
    bool is_autopilot() const override { return true; }
    bool has_user_takeoff(bool must_navigate) const override { return true; }
    bool in_guided_mode() const override { return true; }

    bool requires_terrain_failsafe() const override { return true; }

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    // Return the type of this mode for use by advanced failsafe
    AP_AdvancedFailsafe_Copter::control_mode afs_mode() const override { return AP_AdvancedFailsafe_Copter::control_mode::AFS_AUTO; }
#endif

    // Return true if the throttle high arming check can be skipped when arming from GCS or Scripting
    bool allows_GCS_or_SCR_arming_with_throttle_high() const override { return true; }

    // Sets guided's angular target submode: Using a rotation quaternion, angular velocity, and climbrate or thrust (depends on user option)
    // attitude_quat: IF zero: ang_vel (angular velocity) must be provided even if all zeroes
    //                IF non-zero: attitude_control is performed using both the attitude quaternion and angular velocity
    // ang_vel: angular velocity (rad/s)
    // climb_rate_cms_or_thrust: represents either the climb_rate (cm/s) or thrust scaled from [0, 1], unitless
    // use_thrust: IF true: climb_rate_cms_or_thrust represents thrust
    //             IF false: climb_rate_cms_or_thrust represents climb_rate (cm/s)
    void set_angle(const Quaternion &attitude_quat, const Vector3f &ang_vel, float climb_rate_cms_or_thrust, bool use_thrust);

    bool set_destination(const Vector3f& destination, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false, bool terrain_alt = false);
    bool set_destination(const Location& dest_loc, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false);
    bool get_wp(Location &loc) const override;
    void set_accel(const Vector3f& acceleration, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false, bool log_request = true);
    void set_velocity(const Vector3f& velocity, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false, bool log_request = true);
    void set_velaccel(const Vector3f& velocity, const Vector3f& acceleration, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false, bool log_request = true);
    bool set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false);
    bool set_destination_posvelaccel(const Vector3f& destination, const Vector3f& velocity, const Vector3f& acceleration, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false);

    // get position, velocity and acceleration targets
    const Vector3p& get_target_pos() const;
    const Vector3f& get_target_vel() const;
    const Vector3f& get_target_accel() const;

    // returns true if GUIDED_OPTIONS param suggests SET_ATTITUDE_TARGET's "thrust" field should be interpreted as thrust instead of climb rate
    bool set_attitude_target_provides_thrust() const;
    bool stabilizing_pos_xy() const;
    bool stabilizing_vel_xy() const;
    bool use_wpnav_for_position_control() const;

    void limit_clear();
    void limit_init_time_and_pos();
    void limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm);
    bool limit_check();

    bool is_taking_off() const override;
    
    bool set_speed_xy(float speed_xy_cms) override;
    bool set_speed_up(float speed_up_cms) override;
    bool set_speed_down(float speed_down_cms) override;

    // initialises position controller to implement take-off
    // takeoff_alt_cm is interpreted as alt-above-home (in cm) or alt-above-terrain if a rangefinder is available
    bool do_user_takeoff_start(float takeoff_alt_cm) override;

    enum class SubMode {
        TakeOff,
        WP,
        Pos,
        PosVelAccel,
        VelAccel,
        Accel,
        Angle,
    };

    SubMode submode() const { return guided_mode; }

    void angle_control_start();
    void angle_control_run();

    // return guided mode timeout in milliseconds. Only used for velocity, acceleration, angle control, and angular rate control
    uint32_t get_timeout_ms() const;

    bool use_pilot_yaw() const override;

    // pause continue in guided mode
    bool pause() override;
    bool resume() override;

    // true if weathervaning is allowed in guided
#if WEATHERVANE_ENABLED
    bool allows_weathervaning(void) const override;
#endif

protected:

    const char *name() const override { return "GUIDED"; }
    const char *name4() const override { return "GUID"; }

    uint32_t wp_distance() const override;
    int32_t wp_bearing() const override;
    float crosstrack_error() const override;

private:

    // enum for GUID_OPTIONS parameter
    enum class Option : uint32_t {
        AllowArmingFromTX   = (1U << 0),
        // this bit is still available, pilot yaw was mapped to bit 2 for symmetry with auto
        IgnorePilotYaw      = (1U << 2),
        SetAttitudeTarget_ThrustAsThrust = (1U << 3),
        DoNotStabilizePositionXY = (1U << 4),
        DoNotStabilizeVelocityXY = (1U << 5),
        WPNavUsedForPosControl = (1U << 6),
        AllowWeatherVaning = (1U << 7)
    };

    // returns true if the Guided-mode-option is set (see GUID_OPTIONS)
    bool option_is_enabled(Option option) const;

    // wp controller
    void wp_control_start();
    void wp_control_run();

    void pva_control_start();
    void pos_control_start();
    void accel_control_start();
    void velaccel_control_start();
    void posvelaccel_control_start();
    void takeoff_run();
    void pos_control_run();
    void accel_control_run();
    void velaccel_control_run();
    void pause_control_run();
    void posvelaccel_control_run();
    void set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle);

    // controls which controller is run (pos or vel):
    SubMode guided_mode = SubMode::TakeOff;
    bool send_notification;     // used to send one time notification to ground station
    bool takeoff_complete;      // true once takeoff has completed (used to trigger retracting of landing gear)

    // guided mode is paused or not
    bool _paused;
};


class ModeGuidedNoGPS : public ModeGuided {

public:
    // inherit constructor
    using ModeGuided::Mode;
    Number mode_number() const override { return Number::GUIDED_NOGPS; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "GUIDED_NOGPS"; }
    const char *name4() const override { return "GNGP"; }

private:

};


class ModeLand : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::LAND; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    bool is_autopilot() const override { return true; }

    bool is_landing() const override { return true; };

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    // Return the type of this mode for use by advanced failsafe
    AP_AdvancedFailsafe_Copter::control_mode afs_mode() const override { return AP_AdvancedFailsafe_Copter::control_mode::AFS_AUTO; }
#endif

    void do_not_use_GPS();

    // returns true if LAND mode is trying to control X/Y position
    bool controlling_position() const { return control_position; }

    void set_land_pause(bool new_value) { land_pause = new_value; }

protected:

    const char *name() const override { return "LAND"; }
    const char *name4() const override { return "LAND"; }

private:

    void gps_run();
    void nogps_run();

    bool control_position; // true if we are using an external reference to control position

    uint32_t land_start_time;
    bool land_pause;
};


class ModeLoiter : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::LOITER; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override { return true; }
    bool allows_autotune() const override { return true; }

#if FRAME_CONFIG == HELI_FRAME
    bool allows_inverted() const override { return true; };
#endif

#if AC_PRECLAND_ENABLED
    void set_precision_loiter_enabled(bool value) { _precision_loiter_enabled = value; }
#endif

protected:

    const char *name() const override { return "LOITER"; }
    const char *name4() const override { return "LOIT"; }

    uint32_t wp_distance() const override;
    int32_t wp_bearing() const override;
    float crosstrack_error() const override { return pos_control->crosstrack_error();}

#if AC_PRECLAND_ENABLED
    bool do_precision_loiter();
    void precision_loiter_xy();
#endif

private:

#if AC_PRECLAND_ENABLED
    bool _precision_loiter_enabled;
    bool _precision_loiter_active; // true if user has switched on prec loiter
#endif

};


class ModePosHold : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::POSHOLD; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override { return true; }
    bool allows_autotune() const override { return true; }

protected:

    const char *name() const override { return "POSHOLD"; }
    const char *name4() const override { return "PHLD"; }

private:

    void update_pilot_lean_angle(float &lean_angle_filtered, float &lean_angle_raw);
    float mix_controls(float mix_ratio, float first_control, float second_control);
    void update_brake_angle_from_velocity(float &brake_angle, float velocity);
    void init_wind_comp_estimate();
    void update_wind_comp_estimate();
    void get_wind_comp_lean_angles(float &roll_angle, float &pitch_angle);
    void roll_controller_to_pilot_override();
    void pitch_controller_to_pilot_override();

    enum class RPMode {
        PILOT_OVERRIDE=0,            // pilot is controlling this axis (i.e. roll or pitch)
        BRAKE,                       // this axis is braking towards zero
        BRAKE_READY_TO_LOITER,       // this axis has completed braking and is ready to enter loiter mode (both modes must be this value before moving to next stage)
        BRAKE_TO_LOITER,             // both vehicle's axis (roll and pitch) are transitioning from braking to loiter mode (braking and loiter controls are mixed)
        LOITER,                      // both vehicle axis are holding position
        CONTROLLER_TO_PILOT_OVERRIDE // pilot has input controls on this axis and this axis is transitioning to pilot override (other axis will transition to brake if no pilot input)
    };

    RPMode roll_mode;
    RPMode pitch_mode;

    // pilot input related variables
    float pilot_roll;                         // pilot requested roll angle (filtered to slow returns to zero)
    float pilot_pitch;                        // pilot requested roll angle (filtered to slow returns to zero)

    // braking related variables
    struct {
        uint8_t time_updated_roll   : 1;    // true once we have re-estimated the braking time.  This is done once as the vehicle begins to flatten out after braking
        uint8_t time_updated_pitch  : 1;    // true once we have re-estimated the braking time.  This is done once as the vehicle begins to flatten out after braking

        float gain;                         // gain used during conversion of vehicle's velocity to lean angle during braking (calculated from rate)
        float roll;                         // target roll angle during braking periods
        float pitch;                        // target pitch angle during braking periods
        int16_t timeout_roll;               // number of cycles allowed for the braking to complete, this timeout will be updated at half-braking
        int16_t timeout_pitch;              // number of cycles allowed for the braking to complete, this timeout will be updated at half-braking
        float angle_max_roll;               // maximum lean angle achieved during braking.  Used to determine when the vehicle has begun to flatten out so that we can re-estimate the braking time
        float angle_max_pitch;              // maximum lean angle achieved during braking  Used to determine when the vehicle has begun to flatten out so that we can re-estimate the braking time
        int16_t to_loiter_timer;            // cycles to mix brake and loiter controls in POSHOLD_TO_LOITER
    } brake;

    // loiter related variables
    int16_t controller_to_pilot_timer_roll;     // cycles to mix controller and pilot controls in POSHOLD_CONTROLLER_TO_PILOT
    int16_t controller_to_pilot_timer_pitch;    // cycles to mix controller and pilot controls in POSHOLD_CONTROLLER_TO_PILOT
    float controller_final_roll;                // final roll angle from controller as we exit brake or loiter mode (used for mixing with pilot input)
    float controller_final_pitch;               // final pitch angle from controller as we exit brake or loiter mode (used for mixing with pilot input)

    // wind compensation related variables
    Vector2f wind_comp_ef;                      // wind compensation in earth frame, filtered lean angles from position controller
    float wind_comp_roll;                       // roll angle to compensate for wind
    float wind_comp_pitch;                      // pitch angle to compensate for wind
    uint16_t wind_comp_start_timer;             // counter to delay start of wind compensation for a short time after loiter is engaged
    int8_t  wind_comp_timer;                    // counter to reduce wind comp roll/pitch lean angle calcs to 10hz

    // final output
    float roll;   // final roll angle sent to attitude controller
    float pitch;  // final pitch angle sent to attitude controller

};


class ModeRTL : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::RTL; }

    bool init(bool ignore_checks) override;
    void run() override {
        return run(true);
    }
    void run(bool disarm_on_land);

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    bool is_autopilot() const override { return true; }

    bool requires_terrain_failsafe() const override { return true; }

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    // Return the type of this mode for use by advanced failsafe
    AP_AdvancedFailsafe_Copter::control_mode afs_mode() const override { return AP_AdvancedFailsafe_Copter::control_mode::AFS_AUTO; }
#endif

    // for reporting to GCS
    bool get_wp(Location &loc) const override;

    bool use_pilot_yaw() const override;

    bool set_speed_xy(float speed_xy_cms) override;
    bool set_speed_up(float speed_up_cms) override;
    bool set_speed_down(float speed_down_cms) override;

    // RTL states
    enum class SubMode : uint8_t {
        STARTING,
        INITIAL_CLIMB,
        RETURN_HOME,
        LOITER_AT_HOME,
        FINAL_DESCENT,
        LAND
    };
    SubMode state() { return _state; }

    // this should probably not be exposed
    bool state_complete() const { return _state_complete; }

    virtual bool is_landing() const override;

    void restart_without_terrain();

    // enum for RTL_ALT_TYPE parameter
    enum class RTLAltType : int8_t {
        RELATIVE = 0,
        TERRAIN = 1
    };
    ModeRTL::RTLAltType get_alt_type() const;

protected:

    const char *name() const override { return "RTL"; }
    const char *name4() const override { return "RTL "; }

    // for reporting to GCS
    uint32_t wp_distance() const override;
    int32_t wp_bearing() const override;
    float crosstrack_error() const override { return wp_nav->crosstrack_error();}

    void descent_start();
    void descent_run();
    void land_start();
    void land_run(bool disarm_on_land);

    void set_descent_target_alt(uint32_t alt) { rtl_path.descent_target.alt = alt; }

private:

    void climb_start();
    void return_start();
    void climb_return_run();
    void loiterathome_start();
    void loiterathome_run();
    void build_path();
    void compute_return_target();

    SubMode _state = SubMode::INITIAL_CLIMB;  // records state of rtl (initial climb, returning home, etc)
    bool _state_complete = false; // set to true if the current state is completed

    struct {
        // NEU w/ Z element alt-above-ekf-origin unless use_terrain is true in which case Z element is alt-above-terrain
        Location origin_point;
        Location climb_target;
        Location return_target;
        Location descent_target;
        bool land;
    } rtl_path;

    // return target alt type
    enum class ReturnTargetAltType {
        RELATIVE = 0,
        RANGEFINDER = 1,
        TERRAINDATABASE = 2
    };

    // Loiter timer - Records how long we have been in loiter
    uint32_t _loiter_start_time;

    bool terrain_following_allowed;

    // enum for RTL_OPTIONS parameter
    enum class Options : int32_t {
        // First pair of bits are still available, pilot yaw was mapped to bit 2 for symmetry with auto
        IgnorePilotYaw    = (1U << 2),
    };

};


class ModeSmartRTL : public ModeRTL {

public:
    // inherit constructor
    using ModeRTL::Mode;
    Number mode_number() const override { return Number::SMART_RTL; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; }
    bool is_autopilot() const override { return true; }

    void save_position();
    void exit() override;

    bool is_landing() const override;
    bool use_pilot_yaw() const override;

    // Safe RTL states
    enum class SubMode : uint8_t {
        WAIT_FOR_PATH_CLEANUP,
        PATH_FOLLOW,
        PRELAND_POSITION,
        DESCEND,
        LAND
    };

protected:

    const char *name() const override { return "SMARTRTL"; }
    const char *name4() const override { return "SRTL"; }

    // for reporting to GCS
    bool get_wp(Location &loc) const override;
    uint32_t wp_distance() const override;
    int32_t wp_bearing() const override;
    float crosstrack_error() const override { return wp_nav->crosstrack_error();}

private:

    void wait_cleanup_run();
    void path_follow_run();
    void pre_land_position_run();
    void land();
    SubMode smart_rtl_state = SubMode::PATH_FOLLOW;

    // keep track of how long we have failed to get another return
    // point while following our path home.  If we take too long we
    // may choose to land the vehicle.
    uint32_t path_follow_last_pop_fail_ms;

    // backup last popped point so that it can be restored to the path
    // if vehicle exits SmartRTL mode before reaching home. invalid if zero
    Vector3f dest_NED_backup;
};


class ModeSport : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::SPORT; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override {
        return !must_navigate;
    }

protected:

    const char *name() const override { return "SPORT"; }
    const char *name4() const override { return "SPRT"; }

private:

};


class ModeStabilize : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::STABILIZE; }

    virtual void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return true; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool allows_save_trim() const override { return true; }
    bool allows_autotune() const override { return true; }
    bool allows_flip() const override { return true; }

protected:

    const char *name() const override { return "STABILIZE"; }
    const char *name4() const override { return "STAB"; }

private:

};

#if FRAME_CONFIG == HELI_FRAME
class ModeStabilize_Heli : public ModeStabilize {

public:
    // inherit constructor
    using ModeStabilize::Mode;

    bool init(bool ignore_checks) override;
    void run() override;

    bool allows_inverted() const override { return true; };

protected:

private:

};
#endif

class ModeSystemId : public Mode {

public:
    ModeSystemId(void);
    Number mode_number() const override { return Number::SYSTEMID; }

    bool init(bool ignore_checks) override;
    void run() override;
    void exit() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return true; }
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    bool is_autopilot() const override { return false; }
    bool logs_attitude() const override { return true; }

    void set_magnitude(float input) { waveform_magnitude.set(input); }

    static const struct AP_Param::GroupInfo var_info[];

    Chirp chirp_input;

protected:

    const char *name() const override { return "SYSTEMID"; }
    const char *name4() const override { return "SYSI"; }

private:

    void log_data() const;
    bool is_poscontrol_axis_type() const;

    enum class AxisType {
        NONE = 0,               // none
        INPUT_ROLL = 1,         // angle input roll axis is being excited
        INPUT_PITCH = 2,        // angle pitch axis is being excited
        INPUT_YAW = 3,          // angle yaw axis is being excited
        RECOVER_ROLL = 4,       // angle roll axis is being excited
        RECOVER_PITCH = 5,      // angle pitch axis is being excited
        RECOVER_YAW = 6,        // angle yaw axis is being excited
        RATE_ROLL = 7,          // rate roll axis is being excited
        RATE_PITCH = 8,         // rate pitch axis is being excited
        RATE_YAW = 9,           // rate yaw axis is being excited
        MIX_ROLL = 10,          // mixer roll axis is being excited
        MIX_PITCH = 11,         // mixer pitch axis is being excited
        MIX_YAW = 12,           // mixer pitch axis is being excited
        MIX_THROTTLE = 13,      // mixer throttle axis is being excited
        DISTURB_POS_LAT = 14,   // lateral body axis measured position is being excited
        DISTURB_POS_LONG = 15,  // longitudinal body axis measured position is being excited
        DISTURB_VEL_LAT = 16,   // lateral body axis measured velocity is being excited
        DISTURB_VEL_LONG = 17,  // longitudinal body axis measured velocity is being excited
        INPUT_VEL_LAT = 18,     // lateral body axis commanded velocity is being excited
        INPUT_VEL_LONG = 19,    // longitudinal body axis commanded velocity is being excited
    };

    AP_Int8 axis;               // Controls which axis are being excited. Set to non-zero to display other parameters
    AP_Float waveform_magnitude;// Magnitude of chirp waveform
    AP_Float frequency_start;   // Frequency at the start of the chirp
    AP_Float frequency_stop;    // Frequency at the end of the chirp
    AP_Float time_fade_in;      // Time to reach maximum amplitude of chirp
    AP_Float time_record;       // Time taken to complete the chirp waveform
    AP_Float time_fade_out;     // Time to reach zero amplitude after chirp finishes

    bool att_bf_feedforward;    // Setting of attitude_control->get_bf_feedforward
    float waveform_time;        // Time reference for waveform
    float waveform_sample;      // Current waveform sample
    float waveform_freq_rads;   // Instantaneous waveform frequency
    float time_const_freq;      // Time at constant frequency before chirp starts
    int8_t log_subsample;       // Subsample multiple for logging.
    Vector2f target_vel;        // target velocity for position controller modes
    Vector2f target_pos;       // target positon
    Vector2f input_vel_last;    // last cycle input velocity
    // System ID states
    enum class SystemIDModeState {
        SYSTEMID_STATE_STOPPED,
        SYSTEMID_STATE_TESTING
    } systemid_state;
};

class ModeThrow : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::THROW; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }

    // Throw types
    enum class ThrowType {
        Upward = 0,
        Drop = 1
    };

    enum class PreThrowMotorState {
        STOPPED = 0,
        RUNNING = 1,
    };

protected:

    const char *name() const override { return "THROW"; }
    const char *name4() const override { return "THRW"; }

private:

    bool throw_detected();
    bool throw_position_good() const;
    bool throw_height_good() const;
    bool throw_attitude_good() const;

    // Throw stages
    enum ThrowModeStage {
        Throw_Disarmed,
        Throw_Detecting,
        Throw_Wait_Throttle_Unlimited,
        Throw_Uprighting,
        Throw_HgtStabilise,
        Throw_PosHold
    };

    ThrowModeStage stage = Throw_Disarmed;
    ThrowModeStage prev_stage = Throw_Disarmed;
    uint32_t last_log_ms;
    bool nextmode_attempted;
    uint32_t free_fall_start_ms;    // system time free fall was detected
    float free_fall_start_velz;     // vertical velocity when free fall was detected
};

#if MODE_TURTLE_ENABLED
class ModeTurtle : public Mode {

public:
    // inherit constructors
    using Mode::Mode;
    Number mode_number() const override { return Number::TURTLE; }

    bool init(bool ignore_checks) override;
    void run() override;
    void exit() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return true; }
    bool allows_arming(AP_Arming::Method method) const override;
    bool is_autopilot() const override { return false; }
    void change_motor_direction(bool reverse);
    void output_to_motors() override;

protected:
    const char *name() const override { return "TURTLE"; }
    const char *name4() const override { return "TRTL"; }

private:
    void arm_motors();
    void disarm_motors();

    float motors_output;
    Vector2f motors_input;
    uint32_t last_throttle_warning_output_ms;
};
#endif

#if MODE_TDCN_ENABLED
/* Sejong */
// TDCN integrates the externally supplied CLAW controller
// (mode_tdcn_CLAW_IBSC_ship_Fianl_NED.c).  Version 1 only monitors CLAW - its
// control output is not applied to the vehicle.  Implementation: mode_tdcn.cpp
// CLAW 제어기 게인.
//
// 원래 mode_tdcn_CLAW_data_0729.c 에 상수로 박혀 있어 값을 바꾸려면 재빌드가
// 필요했다.  파라미터로 빼서 미션플래너에서 CLAW_ 로 검색 / 조정할 수 있게 한다.
// apply() 가 CLAW_step() 직전에 CLAW_P 로 복사하므로 바꾸면 즉시 반영된다.
//
// BSC_B_mat (제어효과 행렬) 은 제외했다.  기체 제원에서 나오는 값이고 48개짜리
// 배열이라 파라미터로 낼 대상이 아니다.
//
// ModeTDCN 과 별도 그룹인 이유는 접두사 때문이다.  ModeTDCN 안에 두면 이름이
// TDCN_CLAW_... 가 되어 16자(AP_MAX_NAME_SIZE) 를 넘긴다.
class CLAW_Gains {
public:
    CLAW_Gains(void);

    static const struct AP_Param::GroupInfo var_info[];

    // 파라미터 값을 CLAW_P 로 옮긴다.  CLAW_step() 전에 부른다.
    void apply(void) const;

private:
    AP_Float _scale_th, _scale_r, _scale_p, _scale_y;
    AP_Float _k_pos_p, _k_pos_i, _k_vel_p, _k_vel_i;
    AP_Float _awu_limit;
    AP_Float _ome_xx, _ome_yy, _ome_zz, _ome_ph, _ome_th, _ome_ps;
    AP_Float _zeta_xx, _zeta_yy, _zeta_zz, _zeta_ph, _zeta_th, _zeta_ps;
    AP_Float _tau_hdot, _tau_r;
};

class ModeTDCN : public Mode {

public:
    ModeTDCN(void);

    // CLAW 게인 파라미터.  ParametersG2 가 주소를 잡아야 해서 public 이다.
    CLAW_Gains claw_gains;

    // 믹서 직전 훅.  TDCN_CLAW_ON_OFF 가 1 이면 여기서 아두파일럿이 계산한
    // 제어값을 CLAW 값으로 갈아끼운다.  Copter::motors_output() 이 부른다.
    void output_to_motors() override;

    // inherit constructors
    using Mode::Mode;
    Number mode_number() const override { return Number::TDCN; }

    // 파라미터 테이블.  ParametersG2 에 TDCN_ 접두사로 등록되어 미션플래너
    // 파라미터 목록에서 검색 / 변경할 수 있다 (Parameters.cpp 의 var_info2).
    static const struct AP_Param::GroupInfo var_info[];

    // 시나리오 상태 - GCS 가 MAV_CMD_USER_1 param1 으로 보낸다
    enum class State : uint8_t {
        NONE          = 0,  // 아직 GCS 명령을 받지 못함
        HANGAR_OPEN   = 1,  // 격납함 열기
        TAKEOFF_WAIT  = 2,  // 이륙 대기
        ARMED         = 3,  // ARMED
        LAUNCH        = 4,  // 이륙 사출
        FLIGHT_WAIT   = 5,  // 비행 대기
        TRACKING      = 6,  // 추종 비행 (타겟 정보를 받는 유일한 상태)
        LANDING_WAIT  = 7,  // 착륙 대기
        LANDING_SYNC  = 8,  // 착륙 동기
        LANDING_STOW  = 9,  // 착륙 수납
        DISARMED      = 10, // DISARMED
        HANGAR_CLOSE  = 11, // 격납함 닫기
    };

    bool init(bool ignore_checks) override;
    void run() override;
    void exit() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; }
    bool is_autopilot() const override { return true; }

    // 이륙/착륙 중인지를 ArduPilot 본체에 알려준다.  안 알려주면
    //  - 이륙(state 4): land_detector 가 "착륙 상태인데 스로틀이 높다" 로 보고
    //    INTERNAL_ERROR(flow_of_control) 를 기록한다.  이 내부 오류는 이후
    //    prearm 을 영구히 막는다.
    //  - 착륙(state 9): failsafe 가 착륙을 중단하고 RTL 로 바꿔버리고,
    //    fence/throttle mix 도 착륙 상태를 반영하지 못한다.
    bool is_taking_off() const override;
    bool is_landing() const override;

    // MAV_CMD_USER_1 수신 + 파싱.  GCS_Mavlink.cpp 가 호출한다.
    //
    // GCS::update_receive 는 메인 스레드 스케줄러 태스크이므로 run() 과 같은
    // 스레드에서 돈다.  따라서 수신값을 따로 보관해 두고 run() 에서 꺼내 쓸
    // 필요가 없고, 여기서 바로 파싱해도 안전하다 (락 불필요).
    MAV_RESULT GCS_command(const mavlink_command_int_t &packet);

    // state 번호에 대응하는 이름 (로그/GCS 메시지용)
    static const char *state_name(State state);

protected:
    const char *name() const override { return "TDCN"; }
    const char *name4() const override { return "TDCN"; }

private:

    // run() 2단계 - CLAW_U 에 기체 정보를 전달한다
    void Update_Info_for_CLAW();

    // CLAW 한 스텝 (home 갱신 -> 입력 전달 -> CLAW_step -> 출력 로깅).
    // CLAW 를 돌려야 하는 state_*() 에서 호출한다.
    void Run_CLAW();

    // state 순서 가드.  GCS 가 순서를 건너뛰거나 현재 단계가 끝나기 전에
    // 다음 번호를 보내는 것을 막는다.
    static bool state_order_ok(State from, State to);   // 번호 순서가 맞는가

    // CLAW 출력 모니터링.  CLAW_step() 직후에 호출한다.
    void Log_Write_TDCN();

    void state_hangar_open();       // 1
    void state_takeoff_wait();      // 2
    void state_armed();             // 3
    void state_launch();            // 4
    void state_flight_wait();       // 5
    void state_tracking();          // 6
    void state_landing_wait();      // 7
    void state_landing_sync();      // 8
    void state_landing_stow();      // 9
    void state_disarmed();          // 10
    void state_hangar_close();      // 11

    State _state;                   // 현재 시나리오 상태

    // state 진입 훅.  run() 의 case 문이 매 루프 state_*() 를 호출하므로,
    // "방금 진입" 과 "계속 유지" 를 구분할 방법이 필요하다.
    //   _state_entered   진입한 그 루프에만 true.  run() 끝에서 소비한다.
    //                    한 번만 해야 하는 동작(격납함 열기 등)에 쓴다.
    //   _state_start_ms  현재 state 에 들어온 시각.  경과시간 / 타임아웃에 쓴다.
    bool _state_entered;
    uint32_t _state_start_ms;

    // 현재 state 가 할 일을 마쳤는가.  state 진입 시 false 로 내려가고,
    // 각 state_*() 가 자기 완료 조건을 스스로 판단해 true 로 올린다.
    // 이 값이 false 인 동안에는 다음 번호로 넘어갈 수 없다.
    //
    // 완료된 뒤에도 각 state_*() 는 자기 상태를 계속 유지한다.  조건이
    // 흐트러지면 _state_done 이 다시 false 로 내려가고 그 함수가 복구를
    // 시도한다 (예: DISARM_DELAY 로 무장이 풀리면 state 3 이 재무장).
    bool _state_done;

    // arm / disarm 재시도 시각.  두 함수 모두 체크를 전부 돌리고 결과를 GCS 에
    // 출력하므로 400Hz 로 부를 수 없다.  한 번에 한 state 만 활성이라 공유한다.
    uint32_t _action_retry_ms;

    // state 3 - 직전 루프의 무장 상태.  무장 -> 해제로 떨어지는 순간을 잡아서
    // 1Hz 재시도를 기다리지 않고 곧바로 다시 무장하기 위한 것이다.
    bool _was_armed;

    // 무장이 걸린 시각.  run() 이 매 루프 해제->무장 엣지를 잡아 갱신한다.
    // state 4 가 "무장한 지 얼마나 됐는가" 로 이륙 시작 시점을 미루는 데 쓴다.
    // (_was_armed 는 state 3 전용이라 따로 둔다)
    uint32_t _armed_ms;
    bool     _armed_prev;

    // state 4 - 이륙 시퀀스를 이미 시작했는가.
    // _state_entered 는 run() 끝에서 소비되므로, 정착 대기로 한 루프라도
    // 일찍 리턴하면 "진입" 정보가 사라진다.  그래서 별도 플래그로 들고 있는다.
    bool _takeoff_started;

    // 이륙 전 state (0~3) 의 기체 처리.  지상이면 안전 처리, 공중이면 제자리
    // 유지.  비행 중에 TDCN 으로 모드를 바꿔도 기체가 가라앉지 않게 한다.
    void preflight_vehicle_handling();

    // 위 처리에서 잡을 위치를 이미 정했는가.  지상으로 내려오면 다시 내린다.
    bool _air_hold_valid;

    uint16_t _log_counter;          // 로그 데시메이션 카운터

    // state 2 - 마지막으로 GCS 에 알린 pre-arm 결과.  400Hz 로 같은 내용을
    // 반복해 보내지 않기 위해 결과가 바뀔 때만 알린다.
    bool _prearm_ready;

    // state 5 - 유지할 위치 (EKF origin 기준 NEU cm).  state 4 의 이륙 완료
    // 위치를 잡아두고 계속 그 점을 명령한다.
    Vector3p _hold_pos_neu_cm;

    // state 6 - 추종할 타겟 위치 (EKF origin 기준 NEU cm).  GCS 가 준 위경도를
    // 매 루프 변환해 넣는다.  변환 실패 시 마지막 값을 유지한다.
    Vector3p _track_pos_neu_cm;

    // 추종 비행(state 6) 타겟.  MAV_CMD_USER_1 수신값을 그대로 보관하고
    // Update_Info_for_CLAW() 에서 CLAW_U.Dest_poti_i 로 변환 없이 전달한다.
    // (heading 만 deg -> rad)
    // 위경도는 Location 의 int32 (1e7 deg) 로 보관한다.  float 로 담으면
    // 0.4~1.4 m 로 양자화되므로 COMMAND_INT 의 x/y 를 그대로 넣는다.
    // 고도 기준(AltFrame)도 Location 이 함께 보유한다.
    Location _target_loc;
    float    _target_heading_deg;   // 진북 기준 (deg)

    // CLAW_step() 직전에 떠 두는 입력 스냅샷.
    //
    // "TDCN 이 넘긴 값" 과 "CLAW 가 내부에서 인식한 값" 을 비교하려면 넘긴 쪽을
    // 그대로 들고 있어야 한다.  CLAW 는 home 래치 스텝에서 Dest_poti_i 를 현재
    // 위치로 스스로 덮어쓰므로, CLAW_step() 뒤에 읽으면 넘긴 값이 아니게 된다.
    //
    // 고도는 넘길 때의 기준(up 양수) 그대로 담는다.  CLAW 는 안에서 down 으로
    // 뒤집으므로, 로그로 남길 때 이쪽을 뒤집어 맞춘다.
    double _in_cur_lat, _in_cur_lng, _in_cur_alt;   // deg, deg, m(up)
    double _in_dst_lat, _in_dst_lng, _in_dst_alt;   // deg, deg, m(up)
    double _in_vel_n, _in_vel_e, _in_vel_d;         // m/s, NED
    float  _in_p, _in_q, _in_r;                     // rad/s, body FRD
    float  _in_roll, _in_pitch, _in_yaw;            // rad
    float  _in_ship_hdg;                            // rad, 진북

    // 아두파일럿 rate PID 출력 스냅샷.
    //
    // output_to_motors() 가 CLAW 값으로 motors 를 덮어쓰기 전에 잡아둔다.
    // 로깅은 update_flight_mode() 안에서 일어나는데 그것은 motors_output()
    // 보다 뒤이므로, 로깅 시점에 motors->get_*() 를 읽으면 CLAW 값이 나온다.
    // (그러면 인계 중 TDCC 의 MR/MP/MY/MT 가 CLAW 를 자기 자신과 비교하게 된다)
    float  _ap_roll_out;
    float  _ap_pitch_out;
    float  _ap_yaw_out;
    float  _ap_throttle_out;

    // --- 파라미터 ---
    //
    // 소스에 박아 두면 값을 바꿀 때마다 재빌드해야 하고, 실기체에서는 현장에서
    // 조정할 수가 없다.  그래서 시나리오가 쓰는 고도 / 속도를 파라미터로 뺐다.
    //
    // state 6 의 추종 속도와 state 9 의 착륙 속도는 여기 없다.  각각
    // WPNAV_SPEED / WPNAV_ACCEL 과 LAND_SPEED / LAND_ALT_LOW 를 그대로 쓰므로
    // 이미 미션플래너에서 조절 가능하다.
    AP_Float _takeoff_alt;      // state 4 이륙 목표 고도 (cm, home 기준 up)
    AP_Float _takeoff_spd;      // state 4 상승 속도 (cm/s)
    AP_Float _land_alt;         // state 8 착륙 동기 고도 (cm, home 기준 up)
    AP_Float _land_spd;         // state 8 하강 속도 (cm/s)

    // 0 = 아두파일럿이 몰고 CLAW 는 병렬 계산만 (v1)
    // 1 = CLAW 의 제어값 4개를 믹서에 직접 넣는다 (v2)
    AP_Int8  _claw_on_off;

    // CLAW 출력을 믹서에 넣어도 되는 상태인가.  output_to_motors() 가 쓴다.
    bool claw_output_active() const;
};
#endif

// modes below rely on Guided mode so must be declared at the end (instead of in alphabetical order)

class ModeAvoidADSB : public ModeGuided {

public:
    // inherit constructor
    using ModeGuided::Mode;
    Number mode_number() const override { return Number::AVOID_ADSB; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; }
    bool is_autopilot() const override { return true; }

    bool set_velocity(const Vector3f& velocity_neu);

protected:

    const char *name() const override { return "AVOID_ADSB"; }
    const char *name4() const override { return "AVOI"; }

private:

};

#if MODE_FOLLOW_ENABLED
class ModeFollow : public ModeGuided {

public:

    // inherit constructor
    using ModeGuided::Mode;
    Number mode_number() const override { return Number::FOLLOW; }

    bool init(bool ignore_checks) override;
    void exit() override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; }
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "FOLLOW"; }
    const char *name4() const override { return "FOLL"; }

    // for reporting to GCS
    bool get_wp(Location &loc) const override;
    uint32_t wp_distance() const override;
    int32_t wp_bearing() const override;

    uint32_t last_log_ms;   // system time of last time desired velocity was logging
};
#endif

class ModeZigZag : public Mode {        

public:
    ModeZigZag(void);

    // Inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::ZIGZAG; }

    enum class Destination : uint8_t {
        A,  // Destination A
        B,  // Destination B
    };

    enum class Direction : uint8_t {
        FORWARD,        // moving forward from the yaw direction
        RIGHT,          // moving right from the yaw direction
        BACKWARD,       // moving backward from the yaw direction
        LEFT,           // moving left from the yaw direction
    } zigzag_direction;

    bool init(bool ignore_checks) override;
    void exit() override;
    void run() override;

    // auto control methods.  copter flies grid pattern
    void run_auto();
    void suspend_auto();
    void init_auto();

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; }
    bool is_autopilot() const override { return true; }
    bool has_user_takeoff(bool must_navigate) const override { return true; }

    // save current position as A or B.  If both A and B have been saved move to the one specified
    void save_or_move_to_destination(Destination ab_dest);

    // return manual control to the pilot
    void return_to_manual_control(bool maintain_target);

    static const struct AP_Param::GroupInfo var_info[];

protected:

    const char *name() const override { return "ZIGZAG"; }
    const char *name4() const override { return "ZIGZ"; }
    uint32_t wp_distance() const override;
    int32_t wp_bearing() const override;
    float crosstrack_error() const override;

private:

    void auto_control();
    void manual_control();
    bool reached_destination();
    bool calculate_next_dest(Destination ab_dest, bool use_wpnav_alt, Vector3f& next_dest, bool& terrain_alt) const;
    void spray(bool b);
    bool calculate_side_dest(Vector3f& next_dest, bool& terrain_alt) const;
    void move_to_side();

    Vector2f dest_A;    // in NEU frame in cm relative to ekf origin
    Vector2f dest_B;    // in NEU frame in cm relative to ekf origin
    Vector3f current_dest; // current target destination (use for resume after suspending)
    bool current_terr_alt;

    // parameters
    AP_Int8  _auto_enabled;    // top level enable/disable control
#if HAL_SPRAYER_ENABLED
    AP_Int8  _spray_enabled;   // auto spray enable/disable
#endif
    AP_Int8  _wp_delay;        // delay for zigzag waypoint
    AP_Float _side_dist;       // sideways distance
    AP_Int8  _direction;       // sideways direction
    AP_Int16 _line_num;        // total number of lines

    enum ZigZagState {
        STORING_POINTS, // storing points A and B, pilot has manual control
        AUTO,           // after A and B defined, pilot toggle the switch from one side to the other, vehicle flies autonomously
        MANUAL_REGAIN   // pilot toggle the switch to middle position, has manual control
    } stage;

    enum AutoState {
        MANUAL,         // not in ZigZag Auto
        AB_MOVING,      // moving from A to B or from B to A
        SIDEWAYS,       // moving to sideways
    } auto_stage;

    uint32_t reach_wp_time_ms = 0;  // time since vehicle reached destination (or zero if not yet reached)
    Destination ab_dest_stored;     // store the current destination
    bool is_auto;                   // enable zigzag auto feature which is automate both AB and sideways
    uint16_t line_count = 0;        // current line number
    int16_t line_num = 0;           // target line number
    bool is_suspended;              // true if zigzag auto is suspended
};

#if MODE_AUTOROTATE_ENABLED
class ModeAutorotate : public Mode {

public:

    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::AUTOROTATE; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool is_autopilot() const override { return true; }
    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; };

    static const struct AP_Param::GroupInfo  var_info[];

protected:

    const char *name() const override { return "AUTOROTATE"; }
    const char *name4() const override { return "AROT"; }

private:

    // --- Internal variables ---
    float _initial_rpm;             // Head speed recorded at initiation of flight mode (RPM)
    float _target_head_speed;       // The terget head main rotor head speed.  Normalised by main rotor set point
    float _desired_v_z;             // Desired vertical
    int32_t _pitch_target;          // Target pitch attitude to pass to attitude controller
    uint32_t _entry_time_start_ms;  // Time remaining until entry phase moves on to glide phase
    float _hs_decay;                // The head accerleration during the entry phase

    enum class Autorotation_Phase {
        ENTRY,
        SS_GLIDE,
        FLARE,
        TOUCH_DOWN,
        LANDED } phase_switch;

    enum class Navigation_Decision {
        USER_CONTROL_STABILISED,
        STRAIGHT_AHEAD,
        INTO_WIND,
        NEAREST_RALLY} nav_pos_switch;

    // --- Internal flags ---
    struct controller_flags {
            bool entry_initial             : 1;
            bool ss_glide_initial          : 1;
            bool flare_initial             : 1;
            bool touch_down_initial        : 1;
            bool landed_initial            : 1;
            bool straight_ahead_initial    : 1;
            bool level_initial             : 1;
            bool break_initial             : 1;
            bool bad_rpm                   : 1;
    } _flags;

    struct message_flags {
            bool bad_rpm                   : 1;
    } _msg_flags;

    //--- Internal functions ---
    void warning_message(uint8_t message_n);    //Handles output messages to the terminal

};
#endif
