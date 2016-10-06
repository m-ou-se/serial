#include <system_error>
#include <utility>

#ifdef WIN32
#include <windows.h>
#else
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#endif

#include "serial.hpp"

namespace Serial {

#ifndef WIN32
namespace {
void set_raw_tty_settings(termios & tty) {
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_oflag &= ~OPOST;
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_cflag |= CLOCAL | CREAD;
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 0;
}
}
#endif

error_or<void> Port::open(char const * name) {
#ifdef WIN32
	std::string file_name = "\\\\.\\";
	file_name += name;
	auto handle = CreateFile(file_name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
	if (handle == INVALID_HANDLE_VALUE) return std::error_code(GetLastError(), std::system_category());
	return open(handle);
#else
	int fd = ::open(name, O_RDWR);
	if (fd == 0) return std::error_code(errno, std::system_category());
	if (auto r = open(fd)) {
		return {};
	} else {
		::close(fd);
		return r.error();
	}
#endif
}

error_or<void> Port::open(native_handle_t handle) {
#ifndef WIN32
	// On Windows, serial ports are always open in 'raw' mode.
	// On other systems, we always switch to raw mode,
	// while leaving the other settings (baud rate, etc.) as they are.
	termios tty;
	if (tcgetattr(handle, &tty)) return std::error_code(errno, std::system_category());
	set_raw_tty_settings(tty);
	if (tcsetattr(handle, TCSANOW, &tty)) return std::error_code(errno, std::system_category());
#endif
	close();
	handle_ = handle;
	return {};
}

error_or<void> Port::set(
	long baud_rate,
	Parity parity,
	StopBits stop_bits,
	DataBits data_bits
) {
#ifdef WIN32
	DCB dcb = {};
	dcb.DCBlength = sizeof(DCB);
	dcb.BaudRate = baud_rate;
	dcb.fBinary = TRUE;
	dcb.fParity = parity == Parity::none ? FALSE : TRUE;
	dcb.fOutxCtsFlow = FALSE;
	dcb.fOutxDsrFlow = FALSE;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fDsrSensitivity = FALSE;
	dcb.fTXContinueOnXoff = TRUE;
	dcb.fOutX = FALSE;
	dcb.fInX = FALSE;
	dcb.fErrorChar = FALSE;
	dcb.fNull = FALSE;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fAbortOnError = FALSE;
	dcb.wReserved = 0;
	dcb.XonLim = 0;
	dcb.XoffLim = 0;
	dcb.ByteSize = data_bits;
	dcb.Parity = parity == Parity::none ? NOPARITY : parity == Parity::odd ? ODDPARITY : EVENPARITY;
	dcb.StopBits = stop_bits == 2 ? TWOSTOPBITS : ONESTOPBIT;
	dcb.XonChar = 0;
	dcb.XoffChar = 0;
	dcb.ErrorChar = 0;
	dcb.EofChar = 0;
	dcb.EvtChar = 0;
	if (!SetCommState(handle_, &dcb)) {
		return std::error_code(GetLastError(), std::system_category());
	}
#else
	termios tty;
	if (tcgetattr(handle_, &tty)) {
		return std::error_code(errno, std::system_category());
	}
	set_raw_tty_settings(tty);
	if (parity) {
		tty.c_cflag |= PARENB;
		if (parity == Parity::odd) {
			tty.c_cflag |= PARODD;
		} else {
			tty.c_cflag &= ~PARODD;
		}
	} else {
		tty.c_cflag &= ~PARENB;
	}
	if (stop_bits == 2) {
		tty.c_cflag |= CSTOPB;
	} else {
		tty.c_cflag &= ~CSTOPB;
	}
	tty.c_cflag &= ~CSIZE;
	switch (data_bits) {
		case 5: tty.c_cflag |= CS5; break;
		case 6: tty.c_cflag |= CS6; break;
		case 7: tty.c_cflag |= CS7; break;
		case 8: tty.c_cflag |= CS8; break;
	}
	speed_t b;
	switch (baud_rate) {
		case      0: b =      B0; break;
		case     50: b =     B50; break;
		case     75: b =     B75; break;
		case    110: b =    B110; break;
		case    134: b =    B134; break;
		case    150: b =    B150; break;
		case    200: b =    B200; break;
		case    300: b =    B300; break;
		case    600: b =    B600; break;
		case   1200: b =   B1200; break;
		case   1800: b =   B1800; break;
		case   2400: b =   B2400; break;
		case   4800: b =   B4800; break;
		case   9600: b =   B9600; break;
		case  19200: b =  B19200; break;
		case  38400: b =  B38400; break;
		case  57600: b =  B57600; break;
		case 115200: b = B115200; break;
		case 230400: b = B230400; break;
		default: return std::make_error_code(std::errc::invalid_argument);
	}
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_oflag &= ~OPOST;
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_cflag |= CLOCAL | CREAD;
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 0;
	if (cfsetospeed(&tty, b) || cfsetispeed(&tty, b) || tcsetattr(handle_, TCSANOW, &tty)) {
		return std::error_code(errno, std::system_category());
	}
#endif
	return {};
}

Port::Port(Port && other) {
	std::swap(handle_, other.handle_);
}

Port & Port::operator = (Port && other) {
	if (other.handle_ != handle_) {
		close();
		std::swap(handle_, other.handle_);
	}
	return *this;
}

error_or<void> Port::close() {
	if (!opened()) return {};
	std::error_code e = {};
#ifdef WIN32
	if (!CloseHandle(handle_)) {
		e = std::error_code(GetLastError(), std::system_category());
	}
#else
	if (::close(handle_)) {
		e = std::error_code(errno, std::system_category());
	}
#endif
	handle_ = Port().handle_;
	return e;
}

error_or<void> Port::write(unsigned char b) {
#ifdef WIN32
	DWORD written = 0;
	if (!WriteFile(handle_, &b, 1, &written, 0) || written != 1) {
		return std::error_code(GetLastError(), std::system_category());
	}
#else
	if (::write(handle_, &b, 1) < 0) {
		return std::error_code(errno, std::system_category());
	}
#endif
	return {};
}

error_or<unsigned char> Port::read(std::chrono::milliseconds timeout) {
#ifdef WIN32
	COMMTIMEOUTS t;
	t.ReadIntervalTimeout = 0;
	t.ReadTotalTimeoutConstant = timeout.count();;
	t.ReadTotalTimeoutMultiplier = 0;
	t.WriteTotalTimeoutConstant = 0;
	t.WriteTotalTimeoutMultiplier = 0;
	unsigned char b;
	DWORD read = 0;
	if (!SetCommTimeouts(handle_, &t) || !ReadFile(handle_, &b , 1, &read, 0)) {
		return std::error_code(GetLastError(), std::system_category());
	}
	if (read == 0) return std::make_error_code(std::errc::stream_timeout);
	return b;
#else
	if (timeout.count() != 0) {
		timeval tv;
		tv.tv_sec = timeout.count() / 1000;
		tv.tv_usec = timeout.count() % 1000 * 1000;
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(handle_, &fds);
		int r = ::select(handle_ + 1, &fds, 0, 0, &tv);
		if (r < 0) throw std::error_code(errno, std::system_category());
		if (r == 0) return std::make_error_code(std::errc::stream_timeout);
	}
	unsigned char b;
	ssize_t r = ::read(handle_, &b, 1);
	if (r < 0) throw std::error_code(errno, std::system_category());
	if (r == 0) return std::make_error_code(std::errc::stream_timeout);
	usleep(1);
	return b;
#endif
}

error_or<void> Port::flush() {
#ifdef WIN32
	if (!PurgeComm(handle_, PURGE_RXCLEAR)) {
		return std::error_code(GetLastError(), std::system_category());
	}
#else
	if (tcflush(handle_, TCIFLUSH)) {
		return std::make_error_code(std::errc::stream_timeout);
	}
#endif
	return {};
}

std::vector<std::string> Port::ports() {
	std::vector<std::string> result;
#ifdef _WIN32
	// GetDefaultCommConfig is slow, so we only test for the first 16 ports.
	for (int i = 1; i <= 16; i++) {
		char port[6] = "COM";
		char * p = &port[3];
		if (i >= 10)  *p++ = '0' + i / 10;
		*p++ = '0' + i % 10;
		*p++ = '\0';
		COMMCONFIG conf;
		DWORD conf_size = sizeof(conf);
		if (GetDefaultCommConfig(port, &conf, &conf_size) || conf_size > sizeof(conf)) {
			result.push_back(std::move(port));
		}
	}
#else
	for (char const * pattern : {"/dev/ttyS*", "/dev/ttyUSB*", "/dev/ttyACM*", "/dev/tty.*"}) {
		glob_t g;
		glob(pattern, 0, nullptr, &g);
		result.reserve(g.gl_pathc);
		for (size_t i = 0; i < g.gl_pathc; ++i) {
			result.push_back(g.gl_pathv[i]);
		}
		globfree(&g);
	}
#endif
	return result;
}

}
