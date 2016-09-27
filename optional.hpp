#if __has_include(<optional>)
#include <optional>
#elif __has_include(<experimental/optional>)
#include <experimental/optional>
namespace std {
using std::experimental::optional;
using std::experimental::nullopt;
using std::experimental::make_optional;
}
#else
#error Missing <optional>
#endif
