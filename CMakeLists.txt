cmake_minimum_required(VERSION 3.8)
project(joy_msg_router_rs_tests)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

if(BUILD_TESTING)
  # Add pytest-based tests
  ament_add_pytest_test(test_integration
    tests/test_integration.py
    TIMEOUT 60
  )
  
  # Add standalone pytest tests
  ament_add_pytest_test(test_joy_router
    tests/test_joy_router.py
    TIMEOUT 60
  )
endif()

ament_package()