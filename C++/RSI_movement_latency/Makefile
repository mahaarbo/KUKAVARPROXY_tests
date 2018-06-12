# Makefile for RSI_movement_latency
# Variables to control makefile operation
CXX = g++
CXXFLAGS = -std=c++11
LFLAGS1 = -lboost_timer -lboost_system -lboost_program_options -lboost_thread
LFLAGS2 = -ltinyxml
# Targets needed to bring the executable up to date

rsi_move_latency: rsi_move_latency.o kuka_hardware_interface.o
	$(CXX) $(CXXFLAGS) -o move_latency rsi_move_latency.o kuka_hardware_interface.o $(LFLAGS1) $(LFLAGS2)


kuka_hardware_interface.o: kuka_hardware_interface.cpp kuka_hardware_interface.h udp_server.h #rsi_state.h rsi_command.h 
	$(CXX) $(CXXFLAGS) -c kuka_hardware_interface.cpp $(LFLAGS2)

rsi_move_latency.o: rsi_move_latency.cpp posix_serial.c kuka_hardware_interface.h 
	$(CXX) $(CXXFLAGS) -c rsi_move_latency.cpp $(LFLAGS1) -I.

rsi_command.o: rsi_command.h

test_imu: test_imu.cpp posix_serial.c
	$(CXX) $(CXXFLAGS) -o test_imu test_imu.cpp $(LFLAGS1)
