# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)

project(rosduino_host VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ROSDUINO_BUILD_TESTS "Build RosDuino unit and integration tests" ON)
option(ROSDUINO_BUILD_FUZZ "Build RosDuino libFuzzer targets" OFF)
option(ROSDUINO_BUILD_VMCU "Build vMCU sketch runners" ON)
option(ROSDUINO_BUILD_SERIAL_TCP "Build rosduino-serial-tcp" ON)
option(ROSDUINO_SANITIZE "Enable sanitizers" ON)

set(ROSDUINO_TOOL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")

add_library(rosduino_warnings INTERFACE)
target_compile_options(rosduino_warnings INTERFACE
  -Wall
  -Wextra
  -Werror
)

add_library(rosduino_sanitizers INTERFACE)
if(ROSDUINO_SANITIZE)
  if(APPLE AND CMAKE_SYSTEM_VERSION VERSION_GREATER_EQUAL "25.0")
    # AppleClang 17 ASan runtime deadlocks in __malloc_init on macOS 26 (Darwin 25.x);
    # use UBSan on host and full ASan+UBSan on Linux/Docker.
    set(ROSDUINO_SAN_FLAGS "-fsanitize=undefined")
  else()
    set(ROSDUINO_SAN_FLAGS "-fsanitize=address,undefined")
  endif()
  target_compile_options(rosduino_sanitizers INTERFACE
    ${ROSDUINO_SAN_FLAGS}
    -fno-omit-frame-pointer
  )
  target_link_options(rosduino_sanitizers INTERFACE
    ${ROSDUINO_SAN_FLAGS}
  )
endif()

# Live rosduino_arduino/src in rosduino_tool, else vendored copy next to host (make libs).
if(EXISTS "${ROSDUINO_TOOL_ROOT}/rosduino_arduino/src/rosduino/core")
  set(ROSDUINO_CORE_INC_DIR "${ROSDUINO_TOOL_ROOT}/rosduino_arduino/src")
elseif(EXISTS "${ROSDUINO_TOOL_ROOT}/core/rosduino/core")
  set(ROSDUINO_CORE_INC_DIR "${ROSDUINO_TOOL_ROOT}/core")
else()
  message(FATAL_ERROR
    "RosDuino core headers not found (rosduino_arduino/src or vendor/core). Run make libs.")
endif()

add_library(rosduino_core INTERFACE)
target_include_directories(rosduino_core INTERFACE "${ROSDUINO_CORE_INC_DIR}")
target_compile_features(rosduino_core INTERFACE cxx_std_11)

add_library(rosduino_transport
  transport/serial_transport.cpp
  transport/tcp_transport.cpp
  transport/transport.cpp
)
target_include_directories(rosduino_transport PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(rosduino_transport PUBLIC cxx_std_17)
target_link_libraries(rosduino_transport PUBLIC
  rosduino_warnings
  rosduino_sanitizers
)

add_library(rosduino_bridge
  bridge/agent_session.cpp
)
target_include_directories(rosduino_bridge PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(rosduino_bridge PUBLIC cxx_std_17)
target_link_libraries(rosduino_bridge PUBLIC
  rosduino_core
  rosduino_warnings
  rosduino_sanitizers
)

add_library(rosduino_vmcu_shim
  vmcu/arduino_shim/arduino.cpp
)
target_include_directories(rosduino_vmcu_shim PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/vmcu/arduino_shim"
)
target_compile_features(rosduino_vmcu_shim PUBLIC cxx_std_17)
target_link_libraries(rosduino_vmcu_shim PUBLIC
  rosduino_warnings
  rosduino_sanitizers
)

function(rosduino_add_vmcu target_name sketch_name)
  set(_vmcu_candidates
    "${ROSDUINO_CORE_INC_DIR}/../examples/Full/${sketch_name}/${sketch_name}.ino"
    "${ROSDUINO_CORE_INC_DIR}/../examples/Simple/${sketch_name}/${sketch_name}.ino"
    "${ROSDUINO_TOOL_ROOT}/tests/sketches/${sketch_name}/${sketch_name}.ino"
  )
  set(_ino "")
  foreach(_cand IN LISTS _vmcu_candidates)
    if(EXISTS "${_cand}")
      set(_ino "${_cand}")
      break()
    endif()
  endforeach()
  if(_ino STREQUAL "")
    message(FATAL_ERROR "vMCU sketch '${sketch_name}' not found")
  endif()
  message(STATUS "vMCU ${target_name}: ${sketch_name} (${_ino})")

  add_executable(${target_name}
    vmcu/main.cpp
    "${_ino}"
  )
  set_source_files_properties("${_ino}" PROPERTIES
    LANGUAGE CXX
    COMPILE_FLAGS "-x c++"
  )
  target_include_directories(${target_name} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/vmcu/arduino_shim"
    "${ROSDUINO_CORE_INC_DIR}"
  )
  target_compile_features(${target_name} PRIVATE cxx_std_17)
  target_compile_definitions(${target_name} PRIVATE ROSDUINO_PROFILE_LITE)
  target_link_libraries(${target_name} PRIVATE
    rosduino_vmcu_shim
    rosduino_core
    rosduino_warnings
    rosduino_sanitizers
  )
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(${target_name} PRIVATE util)
  endif()
endfunction()

if(ROSDUINO_BUILD_VMCU)
  set(VMCU_SKETCH "NanoBasic" CACHE STRING "Arduino sketch compiled into rosduino_vmcu")
  rosduino_add_vmcu(rosduino_vmcu "${VMCU_SKETCH}")
endif()

if(ROSDUINO_BUILD_SERIAL_TCP OR ROSDUINO_BUILD_TESTS)
  add_library(rosduino_serial_tcp
    serial_tcp/serial_tcp.cpp
  )
  target_include_directories(rosduino_serial_tcp PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
  target_compile_features(rosduino_serial_tcp PUBLIC cxx_std_17)
  target_link_libraries(rosduino_serial_tcp PUBLIC
    rosduino_transport
    rosduino_warnings
    rosduino_sanitizers
  )
endif()

if(ROSDUINO_BUILD_SERIAL_TCP)
  add_executable(rosduino-serial-tcp
    serial_tcp/main.cpp
  )
  target_compile_features(rosduino-serial-tcp PRIVATE cxx_std_17)
  target_link_libraries(rosduino-serial-tcp PRIVATE
    rosduino_serial_tcp
    rosduino_warnings
    rosduino_sanitizers
  )
endif()

if(ROSDUINO_BUILD_TESTS)
  enable_testing()

  add_library(doctest INTERFACE)
  target_include_directories(doctest SYSTEM INTERFACE
    "${ROSDUINO_TOOL_ROOT}/third_party/doctest"
  )

  add_executable(rosduino_unit_tests
    "${ROSDUINO_TOOL_ROOT}/tests/unit/smoke_test.cpp"
    "${ROSDUINO_TOOL_ROOT}/tests/unit/cobs_crc_test.cpp"
    "${ROSDUINO_TOOL_ROOT}/tests/unit/frame_test.cpp"
    "${ROSDUINO_TOOL_ROOT}/tests/unit/cdr_test.cpp"
    "${ROSDUINO_TOOL_ROOT}/tests/unit/commands_test.cpp"
    "${ROSDUINO_TOOL_ROOT}/tests/unit/session_test.cpp"
    "${ROSDUINO_TOOL_ROOT}/tests/unit/roundtrip_test.cpp"
    "${ROSDUINO_TOOL_ROOT}/tests/unit/int32_test.cpp"
    "${ROSDUINO_TOOL_ROOT}/tests/unit/bool_empty_test.cpp"
    "${ROSDUINO_TOOL_ROOT}/tests/unit/node_limits_test.cpp"
    "${ROSDUINO_TOOL_ROOT}/tests/unit/qos_test.cpp"
    "${ROSDUINO_TOOL_ROOT}/tests/unit/range_test.cpp"
    "${ROSDUINO_TOOL_ROOT}/tests/unit/kit_cdr_test.cpp"
    "${ROSDUINO_TOOL_ROOT}/tests/unit/timer_test.cpp"
    "${ROSDUINO_TOOL_ROOT}/tests/unit/simple_api_test.cpp"
    "${ROSDUINO_TOOL_ROOT}/tests/unit/transport_test.cpp"
    "${ROSDUINO_TOOL_ROOT}/tests/unit/bridge_test.cpp"
    "${ROSDUINO_TOOL_ROOT}/tests/unit/arduino_shim_test.cpp"
  )
  target_compile_features(rosduino_unit_tests PRIVATE cxx_std_17)
  target_compile_definitions(rosduino_unit_tests PRIVATE
    ROSDUINO_VECTORS_DIR="${ROSDUINO_TOOL_ROOT}/tests/vectors"
  )
  target_link_libraries(rosduino_unit_tests PRIVATE
    rosduino_core
    rosduino_transport
    rosduino_bridge
    rosduino_vmcu_shim
    rosduino_warnings
    rosduino_sanitizers
    doctest
  )
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(rosduino_unit_tests PRIVATE util)
  endif()

  add_test(NAME rosduino_unit_tests COMMAND rosduino_unit_tests)
  add_test(NAME rosduino_vmcu_help COMMAND rosduino_vmcu --help)

  # Pairs: <id> <sketch>. Target rosduino_vmcu_<id>, define ROSDUINO_VMCU_<ID>_BIN.
  set(ROSDUINO_TEST_VMCUS
    service_server ServiceServer
    pubsub PubSub
    overflow NanoOverflow
    basic NanoBasic
    load_nano LoadNano
    load_flood LoadFlood
    spin_measure SpinMeasure
    led Led
    light LightSensor
    client ServiceClient
    time TimeAndLog
    led_service LedService
    add_server AddServer
    add_client AddClient
  )
  set(_vmcu_targets "")
  set(_vmcu_bin_defs "")
  list(LENGTH ROSDUINO_TEST_VMCUS _vmcu_count)
  math(EXPR _vmcu_last "${_vmcu_count} - 1")
  foreach(_i RANGE 0 ${_vmcu_last} 2)
    math(EXPR _j "${_i} + 1")
    list(GET ROSDUINO_TEST_VMCUS ${_i} _id)
    list(GET ROSDUINO_TEST_VMCUS ${_j} _sketch)
    rosduino_add_vmcu(rosduino_vmcu_${_id} ${_sketch})
    string(TOUPPER "${_id}" _id_upper)
    list(APPEND _vmcu_targets rosduino_vmcu_${_id})
    list(APPEND _vmcu_bin_defs "ROSDUINO_VMCU_${_id_upper}_BIN=\"$<TARGET_FILE:rosduino_vmcu_${_id}>\"")
  endforeach()

  add_executable(rosduino_integration_tests
    "${ROSDUINO_TOOL_ROOT}/tests/integration/vmcu_bridge_test.cpp"
    "${ROSDUINO_TOOL_ROOT}/tests/integration/serial_tcp_test.cpp"
  )
  target_compile_features(rosduino_integration_tests PRIVATE cxx_std_17)
  target_compile_definitions(rosduino_integration_tests PRIVATE ${_vmcu_bin_defs})
  target_link_libraries(rosduino_integration_tests PRIVATE
    rosduino_core
    rosduino_transport
    rosduino_bridge
    rosduino_serial_tcp
    rosduino_warnings
    rosduino_sanitizers
    doctest
  )
  add_dependencies(rosduino_integration_tests ${_vmcu_targets})
  add_test(NAME rosduino_integration_tests COMMAND rosduino_integration_tests)
  set_tests_properties(rosduino_integration_tests PROPERTIES TIMEOUT 120)
endif()

if(ROSDUINO_BUILD_FUZZ)
  foreach(fuzz_name IN ITEMS fuzz_frame_reader fuzz_commands fuzz_cdr_reader)
    add_executable(${fuzz_name}
      "${ROSDUINO_TOOL_ROOT}/tests/fuzz/${fuzz_name}.cpp"
    )
    target_compile_features(${fuzz_name} PRIVATE cxx_std_17)
    target_compile_options(${fuzz_name} PRIVATE
      -fsanitize=fuzzer,address,undefined
      -fno-omit-frame-pointer
    )
    target_link_options(${fuzz_name} PRIVATE
      -fsanitize=fuzzer,address,undefined
    )
    target_link_libraries(${fuzz_name} PRIVATE
      rosduino_core
      rosduino_warnings
    )
  endforeach()
endif()
