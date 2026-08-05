set(MOCK_DRIVER_SRC "${CMAKE_CURRENT_LIST_DIR}/../../../../test/mock_drivers.c")

idf_component_register(SRCS "test_encoder.c" "${MOCK_DRIVER_SRC}"
                    INCLUDE_DIRS "." "${CMAKE_CURRENT_LIST_DIR}/../../../../test")
