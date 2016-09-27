# serial
Cross-platform C++17 serial port library.

# Example

```C++
Port p;
p.open(port_name);
p.set(9600, Parity::none, StopBits::two, DataBits::eight);
p.write(0x10);
if (auto c = p.read(100ms)) {
    std::cout << "Got: " << *c << std::endl;
} else {
    std::cout << "Didn't receive anything in 100 milliseconds." << std::endl;
}
```

## Documentation

See [serial.hpp](serial.hpp).

## License

The source code is released under the terms of the 2-clause BSD license, see [COPYING](COPYING).
