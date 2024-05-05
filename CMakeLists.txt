idf_component_register(SRCS I2c.cpp StUsb4500.cpp MaxSonarMB7040.cpp UBloxUBX.cpp BQ25792.cpp UBloxDDC.cpp 
                       LSM303AGR.cpp BNO055.cpp
                       REQUIRES esp_system esp_timer driver log 
                       INCLUDE_DIRS "."
                       )