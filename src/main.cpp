#include <memory>

#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <tmx_cpp/tmx.hpp>

#include <tmx_cpp/sensors.hpp>
#include <tmx_cpp/sensors/ADXL345.hpp>
#include <tmx_cpp/sensors/INA226.hpp>
#include <tmx_cpp/sensors/MPU9250.hpp>
#include <tmx_cpp/sensors/VEML6040.hpp>

#include <tmx_cpp/modules.hpp>
#include <tmx_cpp/modules/HiwonderServo.hpp>
#include <tmx_cpp/modules/PCA9685.hpp>
#include <tmx_cpp/modules/SSD1306_oled.hpp>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

using namespace py::literals;

namespace tmx = tmx_cpp;

using tmx::HiwonderServo_module;
using tmx::MESSAGE_TYPE;
using tmx::PCA9685_module;
using tmx::TMX;

PYBIND11_MODULE(tmx_cpp_py, m) {
  m.doc() = R"pbdoc(
        Pybind11 example plugin
        -----------------------

        .. currentmodule:: tmx_cpp_py

        .. autosummary::
           :toctree: _generate
    )pbdoc";

  py::enum_<MESSAGE_TYPE>(m, "MessageType",
                          "Outward Message types (Out of PC - Into MCU)")
      .value("SERIAL_LOOP_BACK", MESSAGE_TYPE::SERIAL_LOOP_BACK)
      .value("SET_PIN_MODE", MESSAGE_TYPE::SET_PIN_MODE)
      .value("DIGITAL_WRITE", MESSAGE_TYPE::DIGITAL_WRITE)
      .value("PWM_WRITE", MESSAGE_TYPE::PWM_WRITE)
      .value("MODIFY_REPORTING", MESSAGE_TYPE::MODIFY_REPORTING)
      .value("FIRMWARE_VERSION", MESSAGE_TYPE::FIRMWARE_VERSION)
      .value("GET_PICO_UNIQUE_ID", MESSAGE_TYPE::GET_PICO_UNIQUE_ID)
      .value("SERVO_ATTACH", MESSAGE_TYPE::SERVO_ATTACH)
      .value("SERVO_WRITE", MESSAGE_TYPE::SERVO_WRITE)
      .value("SERVO_DETACH", MESSAGE_TYPE::SERVO_DETACH)
      .value("I2C_BEGIN", MESSAGE_TYPE::I2C_BEGIN)
      .value("I2C_READ", MESSAGE_TYPE::I2C_READ)
      .value("I2C_WRITE", MESSAGE_TYPE::I2C_WRITE)
      .value("SONAR_NEW", MESSAGE_TYPE::SONAR_NEW)
      .value("DHT_NEW", MESSAGE_TYPE::DHT_NEW)
      .value("STOP_ALL_REPORTS", MESSAGE_TYPE::STOP_ALL_REPORTS)
      .value("ENABLE_ALL_REPORTS", MESSAGE_TYPE::ENABLE_ALL_REPORTS)
      .value("RESET_DATA", MESSAGE_TYPE::RESET_DATA)
      .value("RESET_BOARD", MESSAGE_TYPE::RESET_BOARD)
      .value("INITIALIZE_NEO_PIXELS", MESSAGE_TYPE::INITIALIZE_NEO_PIXELS)
      .value("SHOW_NEO_PIXELS", MESSAGE_TYPE::SHOW_NEO_PIXELS)
      .value("SET_NEO_PIXEL", MESSAGE_TYPE::SET_NEO_PIXEL)
      .value("CLEAR_ALL_NEO_PIXELS", MESSAGE_TYPE::CLEAR_ALL_NEO_PIXELS)
      .value("FILL_NEO_PIXELS", MESSAGE_TYPE::FILL_NEO_PIXELS)
      .value("SPI_INIT", MESSAGE_TYPE::SPI_INIT)
      .value("SPI_WRITE", MESSAGE_TYPE::SPI_WRITE)
      .value("SPI_READ", MESSAGE_TYPE::SPI_READ)
      .value("SPI_SET_FORMAT", MESSAGE_TYPE::SPI_SET_FORMAT)
      .value("SPI_CS_CONTROL", MESSAGE_TYPE::SPI_CS_CONTROL)
      .value("SET_SCAN_DELAY", MESSAGE_TYPE::SET_SCAN_DELAY)
      .value("ENCODER_NEW", MESSAGE_TYPE::ENCODER_NEW)
      .value("SENSOR_NEW", MESSAGE_TYPE::SENSOR_NEW)
      .value("PING", MESSAGE_TYPE::PING)
      .value("MODULE_NEW", MESSAGE_TYPE::MODULE_NEW)
      .value("MODULE_DATA", MESSAGE_TYPE::MODULE_DATA)
      .value("GET_ID", MESSAGE_TYPE::GET_ID)
      .value("SET_ID", MESSAGE_TYPE::SET_ID);

  py::enum_<TMX::PIN_MODES>(m, "PinMode")
      .value("DIGITAL_INPUT", TMX::PIN_MODES::DIGITAL_INPUT)
      .value("DIGITAL_OUTPUT", TMX::PIN_MODES::DIGITAL_OUTPUT)
      .value("PWM_OUTPUT", TMX::PIN_MODES::PWM_OUTPUT)
      .value("DIGITAL_INPUT_PULL_UP", TMX::PIN_MODES::DIGITAL_INPUT_PULL_UP)
      .value("DIGITAL_INPUT_PULL_DOWN", TMX::PIN_MODES::DIGITAL_INPUT_PULL_DOWN)
      .value("ANALOG_INPUT", TMX::PIN_MODES::ANALOG_INPUT);

  py::class_<TMX, std::shared_ptr<TMX>>(m, "TMX")
      .def(py::init<std::function<void()>, std::string>(), "stop_func"_a,
           "port"_a = "/dev/ttyACM0")
      .def(py::init([](std::string port) {
             return std::make_shared<tmx_cpp::TMX>([]() {}, port);
           }),
           "port"_a = "/dev/ttyACM0")
      // Add Pin Callbacks
      .def("add_digital_callback", &TMX::add_digital_callback, "pin"_a,
           "callback"_a)
      .def("add_analog_callback", &TMX::add_analog_callback, "pin"_a,
           "callback"_a)
      .def("stop", &TMX::stop, "Stop all communication")
      .def("send_message",
           static_cast<void (TMX::*)(
               MESSAGE_TYPE, const std::vector<uint8_t> &)>(&TMX::sendMessage),
           "type"_a, "message"_a)
      .def("set_pin_mode", &TMX::setPinMode, "pin"_a, "mode"_a,
           "reporting"_a = true, "analog_differential"_a = 0)
      .def("digital_write", &TMX::digitalWrite, "pin"_a, "value"_a)
      .def("pwm_write", &TMX::pwmWrite, "pin"_a, "value"_a)
      // Encoder
      .def("attach_encoder", &TMX::attach_encoder, "pin_A"_a, "pin_B"_a,
           "callback"_a)
      // Sonar
      .def("attach_sonar", &TMX::attach_sonar, "trigger_pin"_a, "echo_pin"_a,
           "callback"_a,
           "The sonar callback and return value is in centimeters, as "
           "specified by the original telemetrix protocol.")
      // Servos
      .def("attach_servo", &TMX::attach_servo, "pin"_a, "min_pulses"_a = 1000,
           "max_pulses"_a = 2000)
      .def("write_servo", &TMX::write_servo, "pin"_a, "duty_cycle"_a)
      .def("detach_servo", &TMX::detach_servo, "pin"_a)
      .def("set_scan_delay", &TMX::setScanDelay, "delay"_a)
      .def("set_i2c_pins", &TMX::setI2CPins, "sda"_a, "scl"_a, "port"_a)
      .def_property_readonly("connected",
                             [](TMX &self) { return self.serial->isOpen(); });

  /* ===== SENSORS =====*/
  py::class_<tmx::Sensor_type, std::shared_ptr<tmx::Sensor_type>>(m,
                                                                  "SensorBase");
  // TODO: Should add check if TMX is connected.
  py::class_<tmx::Sensors, std::shared_ptr<tmx::Sensors>>(m, "Sensors")
      .def(py::init<std::shared_ptr<TMX>>(), "tmx"_a)
      .def("add_sens", &tmx::Sensors::add_sens, "sensor"_a);

  // ADDR 0x53
  py::class_<tmx::ADXL345_module, std::shared_ptr<tmx::ADXL345_module>,
             tmx::Sensor_type>(m, "ADXL345_Sensor")
      .def(py::init<uint8_t, uint8_t, tmx::ADXL345_cb_t>(), "i2c_port"_a,
           "i2c_addr"_a, "callback"_a);
  // .def("send_module", &tmx::ADXL345_module::send_module);

  // ADDR 0x40
  py::class_<tmx::INA226_module, std::shared_ptr<tmx::INA226_module>,
             tmx::Sensor_type>(m, "INA226_Sensor")
      .def(py::init<uint8_t, uint8_t, tmx::INA226_cb_t>(), "i2c_port"_a,
           "i2c_addr"_a, "callback"_a);

  // ADDR 0x63
  py::class_<tmx::MPU9250_module, std::shared_ptr<tmx::MPU9250_module>,
             tmx::Sensor_type>(m, "MPU9250_Sensor")
      .def(py::init<uint8_t, uint8_t, tmx::MPU9250_cb_t>(), "i2c_port"_a,
           "i2c_addr"_a, "callback"_a);

  // ADDR 0x10
  py::class_<tmx::VEML6040_module, std::shared_ptr<tmx::VEML6040_module>,
             tmx::Sensor_type>(m, "VEML6040_sensor")
      .def(py::init<uint8_t, uint8_t, tmx::VEML6040_cb_t>(), "i2c_port"_a,
           "i2c_addr"_a, "callback"_a);

  /* ===== MODULES ===== */
  py::class_<tmx::Module_type, std::shared_ptr<tmx::Module_type>>(m,
                                                                  "ModuleBase");
  // TODO: Should add check if TMX is connected.
  py::class_<tmx::Modules, std::shared_ptr<tmx::Modules>>(m, "Modules")
      .def(py::init<std::shared_ptr<TMX>>(), "tmx"_a)
      .def("add_mod", &tmx::Modules::add_mod, "module"_a);

  // ADDR 0x3C
  py::class_<tmx::SSD1306_module, std::shared_ptr<tmx::SSD1306_module>,
             tmx::Module_type>(m, "SSD1306_Module")
      .def(py::init<uint8_t, uint8_t>(), "i2c_port"_a, "address"_a = 0x3C)
      .def("send_text", &tmx::SSD1306_module::send_text, "text"_a,
           "timeout"_a = 200ms)
      .def(
          "send_image",
          [](std::shared_ptr<tmx::SSD1306_module> self, uint8_t width,
             uint8_t height, std::vector<uint8_t> img_buffer,
             std::chrono::milliseconds timeout) {
            return self->send_image(width, height, img_buffer.data(), timeout);
          },
          "width"_a, "height"_a, "img_buffer"_a, "timeout"_a = 200ms)
      .def(
          "send_image",
          [](std::shared_ptr<tmx::SSD1306_module> self, uint8_t width,
             uint8_t height, py::bytes img_buffer,
             std::chrono::milliseconds timeout) {
            return self->send_image(
                width, height,
                reinterpret_cast<uint8_t *>(std::string(img_buffer).data()),
                timeout);
          },
          "width"_a, "height"_a, "img_buffer"_a, "timeout"_a = 200ms);

  py::class_<HiwonderServo_module::Servo_pos>(m, "ServoPos")
      .def(py::init<uint8_t, uint16_t, uint16_t>(), "id"_a = 0, "angle"_a = 0,
           "time"_a = 0)
      .def_readwrite("id", &HiwonderServo_module::Servo_pos::id)
      .def_readwrite("angle", &HiwonderServo_module::Servo_pos::angle)
      .def_readwrite("time", &HiwonderServo_module::Servo_pos::time)
      .def("__repr__", [](const HiwonderServo_module::Servo_pos &self) {
        return "ServoPos(id=" + std::to_string(self.id) +
               ", angle=" + std::to_string(self.angle) +
               ", time=" + std::to_string(self.time) + ")";
      });

  py::class_<HiwonderServo_module, std::shared_ptr<HiwonderServo_module>,
             tmx::Module_type>(m, "HiwonderServo_Module")
      .def(py::init<uint8_t, uint8_t, uint8_t, std::vector<uint8_t>,
                    std::function<void(
                        std::vector<std::tuple<
                            uint8_t, HiwonderServo_module::Servo_pos>>)>>())
      .def("get_offset", &HiwonderServo_module::get_offset, "id"_a)
      .def("get_range", &HiwonderServo_module::get_range, "id"_a)
      .def_readonly("servo_ids", &HiwonderServo_module::servo_ids)
      .def("enable_servo", &HiwonderServo_module::set_enable_servo, "id"_a,
           "enabled"_a)
      .def("enable_all", &HiwonderServo_module::set_enabled_all, "enabled"_a)
      .def("get_servo_num", &HiwonderServo_module::get_servo_num, "id"_a)
      .def("set_servo", &HiwonderServo_module::set_single_servo, "id"_a,
           "angle"_a, "time"_a)
      .def("set_multiple_servos", &HiwonderServo_module::set_multiple_servos,
           "servos"_a, "time"_a)
      .def("verify_id", &HiwonderServo_module::verify_id, "id"_a);

  py::class_<PCA9685_module::PWM_val>(m, "PWMVal")
      .def(py::init<uint8_t, uint16_t, uint16_t>(), "channel"_a, "high"_a,
           "low"_a = 0)
      .def_readwrite("channel", &PCA9685_module::PWM_val::channel)
      .def_readwrite("high", &PCA9685_module::PWM_val::high)
      .def_readwrite("low", &PCA9685_module::PWM_val::low);

  // ADDR 0x40
  py::class_<PCA9685_module, std::shared_ptr<PCA9685_module>, tmx::Module_type>(
      m, "PCA9685_Module")
      .def(py::init<uint8_t, uint8_t, int>(), "i2c_port"_a, "address"_a = 0x40,
           "frequency"_a = 200)
      .def("set_pwm", &PCA9685_module::set_pwm, "channel"_a, "high"_a,
           "low"_a = 0)
      .def(
          "set_pwm",
          [](PCA9685_module &self, PCA9685_module::PWM_val pwm_value) {
            return self.set_pwm(pwm_value.channel, pwm_value.high,
                                pwm_value.low);
          },
          "pwm_value"_a)
      .def("set_multiple_pwm", &PCA9685_module::set_multiple_pwm, "pwm_vals"_a)
      .def("set_mircoseconds", &PCA9685_module::set_mircoseconds, "channel"_a,
           "microseconds"_a);

#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif
}
