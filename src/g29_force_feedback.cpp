#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <mutex>

#include <linux/input.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

#include "ros_g29_force_feedback/msg/force_feedback.hpp"
#include "ros_g29_force_feedback/msg/g29_state.hpp"

class G29ForceFeedback : public rclcpp::Node {

private:
    rclcpp::Subscription<ros_g29_force_feedback::msg::ForceFeedback>::SharedPtr sub_target;
    rclcpp::Publisher<ros_g29_force_feedback::msg::G29State>::SharedPtr pub_state_;
    rclcpp::TimerBase::SharedPtr timer;
    // device info
    int m_device_handle;
    int m_axis_code = ABS_X;
    int m_axis_min;
    int m_axis_max;

    // rosparam
    std::string m_device_name;
    std::string m_input_topic;
    std::string m_state_topic;
    double m_loop_rate;
    double m_max_torque;
    double m_min_torque;
    double m_brake_position;
    double m_brake_torque;
    double m_auto_centering_max_torque;
    double m_auto_centering_max_position;
    double m_eps;
    bool m_auto_centering;
    bool m_publish_state;

    // variables
    std::string m_current_mode = "idle";
    ros_g29_force_feedback::msg::ForceFeedback m_target;
    bool m_is_target_updated = false;
    bool m_is_brake_range = false;
    struct ff_effect m_effect;
    double m_position;
    double m_torque;
    double m_attack_length;

    // Thread safety for parameter updates
    std::mutex m_param_mutex;

    // Parameter callback handle
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_param_callback_handle;

public:
    G29ForceFeedback();
    ~G29ForceFeedback();

private:
    void targetCallback(const ros_g29_force_feedback::msg::ForceFeedback::SharedPtr in_target);
    void loop();
    int testBit(int bit, unsigned char *array);
    void initDevice();
    void calcRotateForce(double &torque, double &attack_length, const ros_g29_force_feedback::msg::ForceFeedback &target, const double &current_position);
    void calcCenteringForce(double &torque, const ros_g29_force_feedback::msg::ForceFeedback &target, const double &current_position);
    void uploadForce(const double &position, const double &force, const double &attack_length);
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);
};


G29ForceFeedback::G29ForceFeedback() 
    : Node("g29_force_feedback"){
        
    declare_parameter("device_name", "/dev/input/event25");
    declare_parameter("input_topic", "ff_target");
    declare_parameter("state_topic", "ff_state");
    declare_parameter("publish_state", true);
    declare_parameter("loop_rate", 0.1);
    declare_parameter("max_torque", 1.0);
    declare_parameter("min_torque", 0.2);
    declare_parameter("brake_position", 0.1);
    declare_parameter("brake_torque", 0.2);
    declare_parameter("auto_centering_max_torque", 0.3);
    declare_parameter("auto_centering_max_position", 0.2);
    declare_parameter("eps", 0.02);
    declare_parameter("auto_centering", false);

    get_parameter("device_name", m_device_name);
    get_parameter("input_topic", m_input_topic);
    get_parameter("state_topic", m_state_topic);
    get_parameter("publish_state", m_publish_state);
    get_parameter("loop_rate", m_loop_rate);
    get_parameter("max_torque", m_max_torque);
    get_parameter("min_torque", m_min_torque);
    get_parameter("brake_position", m_brake_position);
    get_parameter("brake_torque", m_brake_torque);
    get_parameter("auto_centering_max_torque", m_auto_centering_max_torque);
    get_parameter("auto_centering_max_position", m_auto_centering_max_position);
    get_parameter("eps", m_eps);
    get_parameter("auto_centering", m_auto_centering);

    // Create subscription with parameterized topic name
    sub_target = this->create_subscription<ros_g29_force_feedback::msg::ForceFeedback>(
        m_input_topic,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&G29ForceFeedback::targetCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", m_input_topic.c_str());

    // Create state publisher if enabled
    if (m_publish_state) {
        pub_state_ = this->create_publisher<ros_g29_force_feedback::msg::G29State>(
            m_state_topic,
            rclcpp::SystemDefaultsQoS());
        RCLCPP_INFO(this->get_logger(), "State publishing enabled on topic: %s", m_state_topic.c_str());
    }

    initDevice();

    rclcpp::sleep_for(std::chrono::seconds(1));
    timer = this->create_wall_timer(std::chrono::milliseconds((int)(m_loop_rate*1000)), 
            std::bind(&G29ForceFeedback::loop,this));

    // Register parameter callback for runtime reconfiguration
    m_param_callback_handle = this->add_on_set_parameters_callback(
        std::bind(&G29ForceFeedback::parametersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Dynamic parameter reconfiguration enabled for: max_torque, min_torque, brake_position, brake_torque, auto_centering_max_torque, auto_centering_max_position, eps, auto_centering");
}

G29ForceFeedback::~G29ForceFeedback() {

    m_effect.type = FF_CONSTANT;
    m_effect.id = -1;
    m_effect.u.constant.level = 0;
    m_effect.direction = 0;
    // upload m_effect
    if (ioctl(m_device_handle, EVIOCSFF, &m_effect) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to upload force effect during shutdown");
    }
}


rcl_interfaces::msg::SetParametersResult G29ForceFeedback::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "";

    // Lock mutex for thread-safe parameter updates
    std::lock_guard<std::mutex> lock(m_param_mutex);

    for (const auto & param : parameters)
    {
        const std::string param_name = param.get_name();

        // Handle max_torque
        if (param_name == "max_torque")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                double new_value = param.as_double();
                if (new_value < 0.0 || new_value > 1.0)
                {
                    result.successful = false;
                    result.reason = "max_torque must be in range [0.0, 1.0]";
                    RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
                }
                else
                {
                    m_max_torque = new_value;
                    RCLCPP_INFO(this->get_logger(), "max_torque updated to %.3f", new_value);
                }
            }
        }
        // Handle min_torque
        else if (param_name == "min_torque")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                double new_value = param.as_double();
                if (new_value < 0.0 || new_value > 1.0)
                {
                    result.successful = false;
                    result.reason = "min_torque must be in range [0.0, 1.0]";
                    RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
                }
                else
                {
                    m_min_torque = new_value;
                    RCLCPP_INFO(this->get_logger(), "min_torque updated to %.3f", new_value);
                }
            }
        }
        // Handle brake_position
        else if (param_name == "brake_position")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                double new_value = param.as_double();
                if (new_value < 0.0 || new_value > 1.0)
                {
                    result.successful = false;
                    result.reason = "brake_position must be in range [0.0, 1.0]";
                    RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
                }
                else
                {
                    m_brake_position = new_value;
                    RCLCPP_INFO(this->get_logger(), "brake_position updated to %.3f (%.1f degrees)",
                               new_value, new_value * 450.0);
                }
            }
        }
        // Handle brake_torque
        else if (param_name == "brake_torque")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                double new_value = param.as_double();
                if (new_value < 0.0 || new_value > 1.0)
                {
                    result.successful = false;
                    result.reason = "brake_torque must be in range [0.0, 1.0]";
                    RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
                }
                else
                {
                    m_brake_torque = new_value;
                    RCLCPP_INFO(this->get_logger(), "brake_torque updated to %.3f", new_value);
                }
            }
        }
        // Handle auto_centering_max_torque
        else if (param_name == "auto_centering_max_torque")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                double new_value = param.as_double();
                if (new_value < 0.0 || new_value > 1.0)
                {
                    result.successful = false;
                    result.reason = "auto_centering_max_torque must be in range [0.0, 1.0]";
                    RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
                }
                else
                {
                    m_auto_centering_max_torque = new_value;
                    RCLCPP_INFO(this->get_logger(), "auto_centering_max_torque updated to %.3f", new_value);
                }
            }
        }
        // Handle auto_centering_max_position
        else if (param_name == "auto_centering_max_position")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                double new_value = param.as_double();
                if (new_value < 0.0 || new_value > 1.0)
                {
                    result.successful = false;
                    result.reason = "auto_centering_max_position must be in range [0.0, 1.0]";
                    RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
                }
                else
                {
                    m_auto_centering_max_position = new_value;
                    RCLCPP_INFO(this->get_logger(), "auto_centering_max_position updated to %.3f (%.1f degrees)",
                               new_value, new_value * 450.0);
                }
            }
        }
        // Handle eps (dead zone threshold)
        else if (param_name == "eps")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                double new_value = param.as_double();
                if (new_value < 0.0 || new_value > 0.5)
                {
                    result.successful = false;
                    result.reason = "eps must be in range [0.0, 0.5]";
                    RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
                }
                else
                {
                    m_eps = new_value;
                    RCLCPP_INFO(this->get_logger(), "eps updated to %.4f (±%.1f degrees)",
                               new_value, new_value * 450.0);
                }
            }
        }
        // Handle auto_centering mode
        else if (param_name == "auto_centering")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
            {
                bool new_value = param.as_bool();
                m_auto_centering = new_value;
                RCLCPP_INFO(this->get_logger(), "auto_centering %s",
                           new_value ? "enabled" : "disabled");
            }
        }
        // Static parameters - reject changes
        else if (param_name == "device_name")
        {
            result.successful = false;
            result.reason = "device_name is read-only (requires node restart to change)";
            RCLCPP_WARN(this->get_logger(), "%s", result.reason.c_str());
        }
        else if (param_name == "input_topic")
        {
            result.successful = false;
            result.reason = "input_topic is read-only (requires node restart to change)";
            RCLCPP_WARN(this->get_logger(), "%s", result.reason.c_str());
        }
        else if (param_name == "loop_rate")
        {
            result.successful = false;
            result.reason = "loop_rate is read-only (requires node restart to change)";
            RCLCPP_WARN(this->get_logger(), "%s", result.reason.c_str());
        }
        else
        {
            // Unknown parameter
            result.successful = false;
            result.reason = "Unknown parameter: " + param_name;
            RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
        }

        // If any parameter failed, stop processing
        if (!result.successful)
        {
            break;
        }
    }

    return result;
}


// update input event with timer callback
void G29ForceFeedback::loop() {

    struct input_event event;
    double last_position = m_position;
    // get current state
    while (read(m_device_handle, &event, sizeof(event)) == sizeof(event)) {
        if (event.type == EV_ABS && event.code == m_axis_code) {
            m_position = (event.value - (m_axis_max + m_axis_min) * 0.5) * 2 / (m_axis_max - m_axis_min);
        }
    }

    if (m_is_brake_range || m_auto_centering) {
        calcCenteringForce(m_torque, m_target, m_position);
        m_attack_length = 0.0;

    } else {
        calcRotateForce(m_torque, m_attack_length, m_target, m_position);
        m_is_target_updated = false;
    }

    uploadForce(m_target.position, m_torque, m_attack_length);
    
    // Determine current mode
    if (m_auto_centering) {
        m_current_mode = "centering";
    } else if (m_is_brake_range) {
        m_current_mode = "braking";
    } else {
        double diff = m_target.position - m_position;
        if (fabs(diff) < m_eps) {
            m_current_mode = "idle";
        } else {
            m_current_mode = "rotating";
        }
    }
    
    // Publish state if enabled
    if (m_publish_state && pub_state_) {
        auto state_msg = ros_g29_force_feedback::msg::G29State();
        state_msg.header.stamp = this->now();
        state_msg.header.frame_id = "g29_wheel";
        state_msg.current_position = m_position;
        state_msg.target_position = m_target.position;
        state_msg.applied_torque = fabs(m_torque);
        state_msg.mode = m_current_mode;
        state_msg.device_connected = (m_device_handle >= 0);
        
        pub_state_->publish(state_msg);
    }
}


void G29ForceFeedback::calcRotateForce(double &torque,
                                       double &attack_length,
                                       const ros_g29_force_feedback::msg::ForceFeedback &target,
                                       const double &current_position) {

    double diff = target.position - current_position;
    double direction = (diff > 0.0) ? 1.0 : -1.0;

    if (fabs(diff) < m_eps) {
        torque = 0.0;
        attack_length = 0.0;

    } else if (fabs(diff) < m_brake_position) {
        m_is_brake_range = true;
        torque = target.torque * m_brake_torque * -direction;
        attack_length = m_loop_rate;

    } else {
        torque = target.torque * direction;
        attack_length = m_loop_rate;
    }
}


void G29ForceFeedback::calcCenteringForce(double &torque,
                                          const ros_g29_force_feedback::msg::ForceFeedback &target,
                                          const double &current_position) {

    double diff = target.position - current_position;
    double direction = (diff > 0.0) ? 1.0 : -1.0;

    if (fabs(diff) < m_eps)
        torque = 0.0;

    else {
        double torque_range = m_auto_centering_max_torque - m_min_torque;
        double power = (fabs(diff) - m_eps) / (m_auto_centering_max_position - m_eps);
        double buf_torque = power * torque_range + m_min_torque;
        torque = std::min(buf_torque, m_auto_centering_max_torque) * direction;
    }
}


// update input event with writing information to the event file
void G29ForceFeedback::uploadForce(const double &position,
                                   const double &torque,
                                   const double &attack_length) {

    // std::cout << torque << std::endl;
    // set effect
    m_effect.u.constant.level = 0x7fff * std::min(torque, m_max_torque);
    m_effect.direction = 0xC000;
    m_effect.u.constant.envelope.attack_level = 0; /* 0x7fff * force / 2 */
    m_effect.u.constant.envelope.attack_length = attack_length;
    m_effect.u.constant.envelope.fade_level = 0;
    m_effect.u.constant.envelope.fade_length = attack_length;

    // upload effect
    if (ioctl(m_device_handle, EVIOCSFF, &m_effect) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to upload force effect");
    }
}


// get target information of wheel control from ros message
void G29ForceFeedback::targetCallback(const ros_g29_force_feedback::msg::ForceFeedback::SharedPtr in_msg) {

    if (m_target.position == in_msg->position && m_target.torque == fabs(in_msg->torque)) {
        m_is_target_updated = false;

    } else {
        m_target = *in_msg;
        m_target.torque = fabs(m_target.torque);
        m_is_target_updated = true;
        m_is_brake_range = false;
    }
}


// initialize force feedback device
void G29ForceFeedback::initDevice() {
    // setup device
    unsigned char key_bits[1+KEY_MAX/8/sizeof(unsigned char)];
    unsigned char abs_bits[1+ABS_MAX/8/sizeof(unsigned char)];
    unsigned char ff_bits[1+FF_MAX/8/sizeof(unsigned char)];
    struct input_event event;
    struct input_absinfo abs_info;

    m_device_handle = open(m_device_name.c_str(), O_RDWR|O_NONBLOCK);
    if (m_device_handle < 0) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open device: %s", m_device_name.c_str());
        exit(1);

    } else {RCLCPP_INFO(this->get_logger(), "Device opened: %s", m_device_name.c_str());}

    // which axes has the device?
    memset(abs_bits, 0, sizeof(abs_bits));
    if (ioctl(m_device_handle, EVIOCGBIT(EV_ABS, sizeof(abs_bits)), abs_bits) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Cannot get absolute axis bits");
        exit(1);
    }

    // get some information about force feedback
    memset(ff_bits, 0, sizeof(ff_bits));
    if (ioctl(m_device_handle, EVIOCGBIT(EV_FF, sizeof(ff_bits)), ff_bits) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Cannot get force feedback capability bits");
        exit(1);
    }

    // get axis value range
    if (ioctl(m_device_handle, EVIOCGABS(m_axis_code), &abs_info) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Cannot get axis range");
        exit(1);
    }
    m_axis_max = abs_info.maximum;
    m_axis_min = abs_info.minimum;
    if (m_axis_min >= m_axis_max) {
        RCLCPP_ERROR(this->get_logger(), "Axis range has invalid values (min: %d, max: %d)", m_axis_min, m_axis_max);
        exit(1);
    }

    // check force feedback is supported?
    if(!testBit(FF_CONSTANT, ff_bits)) {
        RCLCPP_ERROR(this->get_logger(), "Force feedback is not supported by device");
        exit(1);

    } else { RCLCPP_INFO(this->get_logger(), "Force feedback supported"); }

    // auto centering off
    memset(&event, 0, sizeof(event));
    event.type = EV_FF;
    event.code = FF_AUTOCENTER;
    event.value = 0;
    if (write(m_device_handle, &event, sizeof(event)) != sizeof(event)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to disable auto centering");
        exit(1);
    }

    // init effect and get effect id
    memset(&m_effect, 0, sizeof(m_effect));
    m_effect.type = FF_CONSTANT;
    m_effect.id = -1; // initial value
    m_effect.trigger.button = 0;
    m_effect.trigger.interval = 0;
    m_effect.replay.length = 0xffff;  // longest value
    m_effect.replay.delay = 0; // delay from write(...)
    m_effect.u.constant.level = 0;
    m_effect.direction = 0xC000;
    m_effect.u.constant.envelope.attack_length = 0;
    m_effect.u.constant.envelope.attack_level = 0;
    m_effect.u.constant.envelope.fade_length = 0;
    m_effect.u.constant.envelope.fade_level = 0;

    if (ioctl(m_device_handle, EVIOCSFF, &m_effect) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to upload initial force effect");
        exit(1);
    }

    // start m_effect
    memset(&event, 0, sizeof(event));
    event.type = EV_FF;
    event.code = m_effect.id;
    event.value = 1;
    if (write(m_device_handle, &event, sizeof(event)) != sizeof(event)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start force effect");
        exit(1);
    }
}


// util for initDevice()
int G29ForceFeedback::testBit(int bit, unsigned char *array) {

    return ((array[bit / (sizeof(unsigned char) * 8)] >> (bit % (sizeof(unsigned char) * 8))) & 1);
}


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);

    auto g29_ff = std::make_shared<G29ForceFeedback>();
    rclcpp::spin(g29_ff);

    rclcpp::shutdown();
    return 0;
}
