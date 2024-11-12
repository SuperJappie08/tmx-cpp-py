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
#include <tmx_cpp/modules/SSD1306_oled.hpp>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

// std::shared_ptr<tmx_cpp::TMX> create_tmx(const std::function<void()>
// &stop_func,
//                                          std::string port = "/dev/ttyACM0") {
//   return std::make_shared<tmx_cpp::TMX>(stop_func, port);
// }

tmx_cpp::SSD1306_module create_ssd1306(uint8_t i2c_port,
                                       uint8_t address = 0x3C) {
  return tmx_cpp::SSD1306_module(i2c_port, address);
}

namespace py = pybind11;

using namespace py::literals;

PYBIND11_MODULE(tmx_cpp_py, m) {
  m.doc() = R"pbdoc(
        Pybind11 example plugin
        -----------------------

        .. currentmodule:: tmx_cpp_py

        .. autosummary::
           :toctree: _generate

           add
           subtract
    )pbdoc";
  py::class_<tmx_cpp::TMX, std::shared_ptr<tmx_cpp::TMX>>(m, "TMX")
      .def(py::init<std::function<void()>, std::string>(), "stop_func"_a,
           "port"_a = "/dev/ttyACM0")
      .def("stop", &tmx_cpp::TMX::stop, "Stop all communication")
      .def("add_digital_callback", &tmx_cpp::TMX::add_digital_callback)
      .def("add_analog_callback", &tmx_cpp::TMX::add_analog_callback);
  // TODO: Message Sending
  // .def("sendMessage", &tmx_cpp::TMX::sendMessage, );

  py::class_<tmx_cpp::Sensor_type, std::shared_ptr<tmx_cpp::Sensor_type>>(
      m, "SensorBase");
  py::class_<tmx_cpp::Sensors, std::shared_ptr<tmx_cpp::Sensors>>(m, "Sensors")
      .def(py::init<std::shared_ptr<tmx_cpp::TMX>>())
      .def("add_sens", &tmx_cpp::Sensors::add_sens);

  // ADDR 0x53
  py::class_<tmx_cpp::ADXL345_module, std::shared_ptr<tmx_cpp::ADXL345_module>,
             tmx_cpp::Sensor_type>(m, "ADXL345_Sensor")
      .def(py::init<uint8_t, uint8_t, tmx_cpp::ADXL345_cb_t>());
  // .def("send_module", &tmx_cpp::ADXL345_module::send_module);

  // ADDR 0x40
  py::class_<tmx_cpp::INA226_module, std::shared_ptr<tmx_cpp::INA226_module>,
             tmx_cpp::Sensor_type>(m, "INA226_Sensor")
      .def(py::init<uint8_t, uint8_t, tmx_cpp::INA226_cb_t>());

  // ADDR 0x63
  py::class_<tmx_cpp::MPU9250_module, std::shared_ptr<tmx_cpp::MPU9250_module>,
             tmx_cpp::Sensor_type>(m, "MPU9250_Sensor")
      .def(py::init<uint8_t, uint8_t, tmx_cpp::MPU9250_cb_t>());

  // ADDR 0x10
  py::class_<tmx_cpp::VEML6040_module,
             std::shared_ptr<tmx_cpp::VEML6040_module>, tmx_cpp::Sensor_type>(
      m, "VEML6040_sensor")
      .def(py::init<uint8_t, uint8_t, tmx_cpp::VEML6040_cb_t>());

  py::class_<tmx_cpp::Module_type, std::shared_ptr<tmx_cpp::Module_type>>(
      m, "ModuleBase");
  py::class_<tmx_cpp::Modules, std::shared_ptr<tmx_cpp::Modules>>(m, "Modules")
      .def(py::init<std::shared_ptr<tmx_cpp::TMX>>())
      .def("add_mod", &tmx_cpp::Modules::add_mod);

  py::class_<tmx_cpp::SSD1306_module, std::shared_ptr<tmx_cpp::SSD1306_module>,
             tmx_cpp::Module_type>(m, "SSD1306_Module")
      .def(py::init(&create_ssd1306), "i2c_port"_a, "address"_a = 0x3C)
      .def("send_text", &tmx_cpp::SSD1306_module::send_text, "text"_a,
           "timeout"_a = 200ms)
      .def("send_image", &tmx_cpp::SSD1306_module::send_image, "width"_a,
           "height"_a, "img_buffer"_a, "timeout"_a = 200ms);

  // py::class_<tmx_cpp::INA>

#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif
}
