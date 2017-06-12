#pragma once

#include <chrono>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <mstd/error_or.hpp>
#include <mstd/optional.hpp>

#ifdef WIN32
#include <windows.h>
#endif

namespace Serial {

using mstd::optional;
using mstd::error_or;

namespace _detail {

enum BaudRate {
	_0      = 0,
	_50     = 50,
	_75     = 75,
	_110    = 110,
	_134    = 134,
	_150    = 150,
	_200    = 200,
	_300    = 300,
	_600    = 600,
	_1200   = 1200,
	_1800   = 1800,
	_2400   = 2400,
	_4800   = 4800,
	_9600   = 9600,
	_19200  = 19200,
	_38400  = 38400,
	_57600  = 57600,
	_115200 = 115200,
	_230400 = 230400,
};

enum Parity {
	none = 0,
	even = 1,
	odd  = 2
};

enum StopBits {
	one = 1,
	two = 2
};

enum DataBits {
	five  = 5,
	six   = 6,
	seven = 7,
	eight = 8
};

}

using _detail::Parity;
using _detail::StopBits;
using _detail::DataBits;

class Port {

#ifdef WIN32
	HANDLE handle_ = INVALID_HANDLE_VALUE;
#else
	int handle_ = -1;
#endif

public:
	using native_handle_t = decltype(handle_);

	Port() {}
	Port(Port &&);
	~Port() { close(); }

	Port & operator = (Port &&);

	// Opens (a file handle to) the serial port.
	// On POSIX systems, provide the device file name (e.g. "/dev/ttyUSB0").
	// On Windows, provide the COM port name (e.g "COM3").
	error_or<void> open(char const * name);

	// Take ownership of an already opened serial port.
	error_or<void> open(native_handle_t native_handle);

	// Check if the Port contains an handle or file descriptor.
	bool opened() const;

	// Closes the file handle. Doesn't do anything when !opened().
	error_or<void> close();

	// Set baud rate and other settings.
	error_or<void> set(
		long baud_rate,
		Parity = Parity::none,
		StopBits = StopBits::one,
		DataBits = DataBits::eight
	);

	// Write a byte.
	error_or<void> write(unsigned char);

	// Read a byte, without timeout.
	error_or<unsigned char> read();

	// Read a byte.
	// Returns nullopt on timeout.
	// Note that many USB serial ports have a latency of about 16 milliseconds.
	// So, for example, read(2ms) might return nothing, even though a byte was
	// already received by the USB serial port, if the receive buffer wasn't
	// yet transferred to the computer.
	error_or<optional<unsigned char>> read(std::chrono::milliseconds timeout);

	// Drop all bytes that were already received, but not yet read.
	error_or<void> flush();

	// The native file handle.
	// (The HANDLE on Windows, the file descriptor (int) on other systems.)
	native_handle_t native_handle() const { return handle_; }

	// Releases ownership of the file handle.
	// (opened() will be false afterwards and the file handle won't be closed.)
	native_handle_t release_native_handle();

	// Gives a list with suggestions for port names to give to open().
	// This list may be incomplete.
	static std::vector<std::string> ports();
};

inline bool Port::opened() const {
#ifdef WIN32
	return handle_ != INVALID_HANDLE_VALUE;
#else
	return handle_ >= 0;
#endif
}

inline Port::native_handle_t Port::release_native_handle() {
	native_handle_t h = handle_;
	handle_ = Port().handle_;
	return h;
}

// Set baud rate and other settings on a native handle.
error_or<void> set(
	Port::native_handle_t,
	long baud_rate,
	Parity = Parity::none,
	StopBits = StopBits::one,
	DataBits = DataBits::eight
);

}
