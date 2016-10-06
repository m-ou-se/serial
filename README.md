# serial
Cross-platform C++14 serial port library.

# Example

```C++
Port p;
p.open(port_name);
p.set(9600, Parity::none, StopBits::two, DataBits::eight);
p.write(0x10);
if (auto c = p.read(100ms)) {
    std::cout << "Got: " << *c << std::endl;
} else {
    std::cout << "Didn't receive anything for 100 milliseconds." << std::endl;
}
```

## Documentation

See [serial.hpp](serial.hpp).

## Dependencies

- [mstd](https://github.com/m-ou-se/mstd)

## License

Two-clause BSD license, see [COPYING](COPYING).
