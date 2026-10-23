// Standalone float/double logging format check; no GTest or libdart needed.
//
// Build it next to dart/common/Logging.cpp, e.g.
//   c++ -std=c++17 -I<repo> -I<build dir containing dart/config.hpp>
//       scripts/check_logging_float.cpp dart/common/Logging.cpp
//       dart/common/Console.cpp
// Add -DDART_LOGGING_FORCE_LEGACY_FLOAT=1 (source-private) to Logging.cpp to
// run its no-floating-charconv path on any platform. The fixed-string cases
// always run. -DDART_CHECK_LOGGING_FLOAT_ORACLE=1 (needs C++17 floating
// to_chars) adds a differential check against the standard library.
// -DDART_CHECK_LOGGING_FLOAT_ALLOCATION_FAILURES=1 enables test-only allocation
// faults; compile Logging.cpp with the forced legacy path for this check.
//
// Usage: check_logging_float [random_count_per_type]

#include <dart/common/Logging.hpp>

#include <algorithm>
#include <iostream>
#include <limits>
#include <locale>
#include <new>
#include <string>
#include <type_traits>
#include <vector>

#include <cerrno>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#ifdef DART_CHECK_LOGGING_FLOAT_ORACLE
  #include <charconv>
#endif

#ifdef DART_CHECK_LOGGING_FLOAT_ALLOCATION_FAILURES
namespace {
bool allocationProbe = false;
std::size_t allocationIndex = 0;
std::size_t allocationFailAt = 0;
bool allocationInjected = false;
bool allocationsChangeErrno = false;
const char* allocationFormat = nullptr;
std::size_t allocationFault = 0;
int allocationMode = 0;
} // namespace

// Only this standalone test executable replaces allocation, never libdart.
void* operator new(std::size_t size)
{
  if (allocationProbe && allocationIndex++ == allocationFailAt) {
    allocationInjected = true;
    errno = ENOMEM;
    throw std::bad_alloc();
  }
  if (void* result = std::malloc(size ? size : 1)) {
    // Recovery allocations may also change errno after the conversion guard.
    if (allocationsChangeErrno || (allocationProbe && allocationInjected))
      errno = ENOMEM;
    return result;
  }
  throw std::bad_alloc();
}

void operator delete(void* memory) noexcept
{
  std::free(memory);
}

  #if __cplusplus >= 201402L && defined(__cpp_sized_deallocation)
void operator delete(void* memory, std::size_t) noexcept
{
  std::free(memory);
}
  #endif
#endif // DART_CHECK_LOGGING_FLOAT_ALLOCATION_FAILURES

namespace {

using dart::common::LogLevel;

std::uint64_t checks = 0;
std::uint64_t failures = 0;
std::uint64_t oracleChecks = 0;

std::string captured;
int capturedRound = 0;
int capturedFlags = 0;
int capturedErrno = 0;

void expectEq(
    const std::string& expected, const std::string& actual, const char* what)
{
  ++checks;
  if (expected == actual)
    return;
  if (failures++ < 32) {
    std::cout << "MISMATCH " << what << ": expected [" << expected
              << "] actual [" << actual << "]";
#ifdef DART_CHECK_LOGGING_FLOAT_ALLOCATION_FAILURES
    if (allocationFormat)
      std::cout << " format=[" << allocationFormat
                << "] failAt=" << allocationFault << " mode=" << allocationMode;
#endif
    std::cout << "\n";
  }
}

void expectTrue(bool condition, const char* what)
{
  expectEq("true", condition ? "true" : "false", what);
}

template <typename T>
std::string logged(const char* format, T value)
{
  captured.clear();
  dart::common::info(format, value);
  return captured;
}

template <typename T>
using Bits = typename std::
    conditional<sizeof(T) == 4, std::uint32_t, std::uint64_t>::type;

template <typename T>
T fromBits(Bits<T> bits)
{
  T value;
  std::memcpy(&value, &bits, sizeof value);
  return value;
}

template <typename T>
Bits<T> toBits(T value)
{
  Bits<T> bits;
  std::memcpy(&bits, &value, sizeof value);
  return bits;
}

//==============================================================================
void fixedCases()
{
  // Plain precision search prints 1.26217745e-29 / 7.1202363472230444e-307.
  expectEq(
      "1.2621775e-29", logged("{}", fromBits<float>(0x0f800000u)), "2^-96");
  expectEq(
      "-1.2621775e-29", logged("{}", fromBits<float>(0x8f800000u)), "-2^-96");
  expectEq(
      "7.120236347223045e-307",
      logged("{}", fromBits<double>(0x0060000000000000ull)),
      "2^-1017");
  expectEq(
      "-7.120236347223045e-307",
      logged("{}", fromBits<double>(0x8060000000000000ull)),
      "-2^-1017");
  expectEq("1.5474251e+26", logged("{}", std::ldexp(1.0f, 87)), "2^87");
  expectEq("1.2379401e+27", logged("{}", std::ldexp(1.0f, 90)), "2^90");
  expectEq("-1.5474251e+26", logged("{}", std::ldexp(-1.0f, 87)), "-2^87");
  expectEq("-1.2379401e+27", logged("{}", std::ldexp(-1.0f, 90)), "-2^90");
  expectEq("0", logged("{}", 0.0), "zero");
  expectEq("-0", logged("{}", -0.0), "-zero");
  expectEq("-0", logged("{}", -0.0f), "-zero float");
  expectEq(
      "1.1754944e-38",
      logged("{}", std::numeric_limits<float>::min()),
      "float min");
  expectEq(
      "1e-45",
      logged("{}", std::numeric_limits<float>::denorm_min()),
      "float denorm");
  expectEq(
      "3.4028235e+38",
      logged("{}", std::numeric_limits<float>::max()),
      "float max");
  expectEq(
      "2.2250738585072014e-308",
      logged("{}", std::numeric_limits<double>::min()),
      "double min");
  expectEq(
      "5e-324",
      logged("{}", std::numeric_limits<double>::denorm_min()),
      "double denorm");
  expectEq(
      "1.7976931348623157e+308",
      logged("{}", std::numeric_limits<double>::max()),
      "double max");
  expectEq("0.1", logged("{}", 0.1f), "0.1f");
  expectEq("0.1", logged("{}", 0.1), "0.1");
  expectEq("0.0001", logged("{}", 1e-4), "layout low edge");
  expectEq("1e-05", logged("{}", 1e-5), "layout below low edge");
  expectEq("9999999", logged("{}", 9999999.0f), "float layout high");
  expectEq("1e+07", logged("{}", 1e7f), "float layout high edge");
  expectEq(
      "999999999999999.9",
      logged("{}", 999999999999999.9),
      "double layout high");
  expectEq("1000000000000000", logged("{}", 1e15), "double 1e15");
  expectEq("1e+16", logged("{}", 1e16), "double 1e16");
  expectEq("12.5", logged("{}", 12.5), "12.5");
  expectEq("1.2345678", logged("{}", 1.2345678f), "float 8 digits");
  expectEq(
      "1.2345678901234567",
      logged("{}", 1.2345678901234567),
      "double 17 digits");
  // Explicit forms use round-to-nearest-even on exact ties.
  expectEq("1.2500e-01", logged("{:.4e}", 0.125), "e4");
  expectEq("0.12", logged("{:.2f}", 0.125), "f2 tie");
  expectEq("0.125", logged("{:.3g}", 0.125f), "g3");
  expectEq("2", logged("{:.0f}", 2.5), "f0 tie even");
  expectEq("4", logged("{:.0f}", 3.5f), "f0 tie odd");
  expectEq("2e+00", logged("{:.0e}", 1.5), "e0 tie");
  expectEq("1e+02", logged("{:.0g}", 100.0), "g0 normalizes to 1");
  expectEq("0.5", logged("{:.1f}", 0.5), "f1");
  expectEq(
      std::string(1001, '0').insert(1, "."),
      logged("{:.1000f}", 0.0),
      "precision 1000 f");
  expectEq(
      std::string("0.") + std::string(1000, '0'),
      logged("{:.1000f}", 0.0f),
      "precision 1000 float f");
  expectTrue(logged("{:.1000e}", 1.5).size() == 1006, "precision 1000 e size");
  expectEq(
      "0.3333333333333333148296163",
      logged("{:.25g}", 1.0 / 3.0).substr(0, 27),
      "g25");
  // Signed non-finite values, in every presentation.
  const char* formats[] = {"{}", "{:.2e}", "{:.2f}", "{:.2g}"};
  for (const char* format : formats) {
    expectEq(
        "nan", logged(format, std::numeric_limits<double>::quiet_NaN()), "nan");
    expectEq(
        "-nan",
        logged(format, -std::numeric_limits<double>::quiet_NaN()),
        "-nan");
    expectEq(
        "inf", logged(format, std::numeric_limits<float>::infinity()), "inf");
    expectEq(
        "-inf",
        logged(format, -std::numeric_limits<float>::infinity()),
        "-inf");
  }
}

//==============================================================================
struct CommaDecimal : std::numpunct<char>
{
  char do_decimal_point() const override
  {
    return ',';
  }
};

void localeCases()
{
  const std::locale previous
      = std::locale::global(std::locale(std::locale(), new CommaDecimal));
  const std::string a = logged("{}", 1.5);
  const std::string b = logged("{:.2f}", 1.5f);
  const std::string c = logged("{}", 0.1f);
  std::locale::global(previous);
  expectEq("1.5", a, "classic locale default");
  expectEq("1.50", b, "classic locale explicit");
  expectEq("0.1", c, "classic locale float");
}

//==============================================================================
// The caller's rounding mode, exception flags and errno are the caller's again
// before the sink runs and afterwards.
void environmentCases()
{
  const int original = std::fegetround();
  const int modes[] = {FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO};
  for (const int mode : modes) {
    if (std::fesetround(mode) != 0)
      continue;
    std::feclearexcept(FE_ALL_EXCEPT);
    std::feraiseexcept(FE_INVALID | FE_DIVBYZERO);
    const int flags = std::fetestexcept(FE_ALL_EXCEPT);
    errno = EDOM;
    const std::string a = logged("{}", 0.1);
    expectEq("0.1", a, "mode default");
    expectTrue(capturedRound == mode, "sink sees caller rounding mode");
    expectTrue(capturedFlags == flags, "sink sees caller exception flags");
    expectTrue(capturedErrno == EDOM, "sink sees caller errno");
    expectEq(
        "1.2621775e-29",
        logged("{}", fromBits<float>(0x0f800000u)),
        "mode 2^-96");
    expectEq(
        "7.120236347223045e-307",
        logged("{}", fromBits<double>(0x0060000000000000ull)),
        "mode 2^-1017");
    expectEq("0.12", logged("{:.2f}", 0.125), "mode explicit f");
    expectEq(
        "5e-324",
        logged("{}", std::numeric_limits<double>::denorm_min()),
        "mode subnormal");
    expectTrue(std::fegetround() == mode, "mode restored");
    expectTrue(std::fetestexcept(FE_ALL_EXCEPT) == flags, "flags restored");
    expectTrue(errno == EDOM, "errno restored");
  }
  std::fesetround(original);
  std::feclearexcept(FE_ALL_EXCEPT);
  errno = 0;
}

//==============================================================================
#ifdef DART_CHECK_LOGGING_FLOAT_ALLOCATION_FAILURES
void callbackDiagnosticCase()
{
  int deliveries = 0;
  int diagnostics = 0;
  int records = 0;
  bool preserved = true;
  bool callbackSawCaller = false;
  auto previous
      = dart::common::setLogSink([&](LogLevel, const std::string& message) {
          preserved &= errno == EDOM;
          ++deliveries;
          diagnostics
              += message.find("A log callback threw") != std::string::npos;
          records += message == "callback record";
          errno = E2BIG;
        });
  dart::common::setLogCallback(
      [](LogLevel, const char*, std::size_t, void* data) {
        *static_cast<bool*>(data) = errno == EDOM;
        allocationsChangeErrno = true;
        errno = ERANGE;
        throw std::bad_alloc();
      },
      &callbackSawCaller);
  errno = EDOM;
  dart::common::info("callback record");
  const int returnedErrno = errno;
  allocationsChangeErrno = false;
  dart::common::setLogCallback(nullptr);
  dart::common::setLogSink(std::move(previous));
  expectTrue(callbackSawCaller, "callback sees caller errno");
  expectTrue(
      deliveries == 2, "callback failure delivers diagnostic and record");
  expectTrue(diagnostics == 1, "callback failure reports once");
  expectTrue(records == 1, "callback failure retains original record");
  expectTrue(
      preserved, "all callback-failure sink deliveries see caller errno");
  expectTrue(returnedErrno == EDOM, "callback failure restores errno");
}

void entryAllocationCases()
{
  for (const bool withArgument : {false, true}) {
    captured.clear();
    allocationIndex = 0;
    allocationFailAt = 0;
    allocationInjected = false;
    errno = EDOM;
    allocationProbe = true;
    if (withArgument)
      dart::common::info(
          "format-copy-longer-than-small-string-buffer {}", 1.25);
    else
      dart::common::info("format-copy-longer-than-small-string-buffer");
    allocationProbe = false;
    const int returnedErrno = errno;
    expectTrue(allocationInjected, "entry allocation fault occurred");
    expectTrue(returnedErrno == EDOM, "entry allocation fault restores errno");
    expectTrue(
        captured.empty(), "failed format copy never dispatches partial output");
  }
}

void allocationFailureCases()
{
  std::fenv_t original;
  expectTrue(std::fegetenv(&original) == 0, "save allocation-test environment");
  const int originalErrno = errno;
  std::uint64_t attempts = 0;
  std::uint64_t diagnostics = 0;
  // Keep the sink itself allocation-free while the formatter is under test.
  captured.reserve(4096);
  for (const char* format : {"{}", "{:.1000e}"}) {
    const std::string expected = logged(format, 1.2345678901234567);
    allocationFailAt = std::numeric_limits<std::size_t>::max();
    allocationIndex = 0;
    allocationProbe = true;
    dart::common::info(format, 1.2345678901234567);
    allocationProbe = false;
    const std::size_t allocations = allocationIndex;
    allocationFormat = format;
    allocationFault = std::numeric_limits<std::size_t>::max();
    allocationMode = FE_TONEAREST;
    // Short output may fit entirely in the implementation's small buffers.
    // The high-precision case must still exercise real allocation failures.
    if (std::strcmp(format, "{}") != 0)
      expectTrue(
          allocations != 0, "high-precision allocation sweep is nonempty");
    for (const int mode :
         {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO}) {
      allocationMode = mode;
      allocationFault = std::numeric_limits<std::size_t>::max();
      expectTrue(
          std::fesetround(mode) == 0, "set allocation-test rounding mode");
      for (std::size_t failAt = 0; failAt < allocations; ++failAt) {
        std::feclearexcept(FE_ALL_EXCEPT);
        std::feraiseexcept(FE_INVALID | FE_DIVBYZERO);
        const int flags = std::fetestexcept(FE_ALL_EXCEPT);
        captured.clear();
        allocationIndex = 0;
        allocationFailAt = failAt;
        allocationInjected = false;
        errno = EDOM;
        allocationProbe = true;
        dart::common::info(format, 1.2345678901234567);
        allocationProbe = false;
        const int returnedErrno = errno;
        allocationFault = failAt;
        allocationMode = mode;
        ++attempts;
        expectTrue(allocationInjected, "requested allocation fault occurred");
        expectTrue(std::fegetround() == mode, "fault restores rounding mode");
        expectTrue(
            std::fetestexcept(FE_ALL_EXCEPT) == flags, "fault restores flags");
        expectTrue(returnedErrno == EDOM, "fault restores errno");
        const bool diagnostic
            = captured.find("[log format error: number conversion failed]")
              == 0;
        diagnostics += diagnostic;
        expectTrue(
            captured.empty() || captured == expected || diagnostic,
            "fault emits complete output or conversion diagnostic");
        if (!captured.empty()) {
          expectTrue(
              capturedRound == mode, "fault sink sees caller rounding mode");
          expectTrue(capturedFlags == flags, "fault sink sees caller flags");
          expectTrue(capturedErrno == EDOM, "fault sink sees caller errno");
        }
      }
    }
  }
  allocationFormat = nullptr;
  expectTrue(attempts != 0, "allocation fault sweep is nonempty");
  expectTrue(
      diagnostics != 0, "failed output streams reach conversion diagnostic");
  std::fesetenv(&original);
  errno = originalErrno;
  std::cout << "allocation_fault_attempts=" << attempts
            << " conversion_diagnostics=" << diagnostics << "\n";
}
#endif // DART_CHECK_LOGGING_FLOAT_ALLOCATION_FAILURES

//==============================================================================
#ifdef DART_CHECK_LOGGING_FLOAT_ORACLE

std::uint64_t rngState = 0x4d595df4d0f33173ULL; // Shared with the prototype.

std::uint64_t random64()
{
  std::uint64_t z = (rngState += 0x9e3779b97f4a7c15ULL);
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
  return z ^ (z >> 31);
}

template <typename T>
std::string reference(T value, char presentation, int precision)
{
  char buffer[2048];
  std::to_chars_result result;
  if (presentation == 0)
    result = std::to_chars(
        buffer, buffer + sizeof(buffer), value, std::chars_format::scientific);
  else
    result = std::to_chars(
        buffer,
        buffer + sizeof(buffer),
        value,
        presentation == 'e'   ? std::chars_format::scientific
        : presentation == 'f' ? std::chars_format::fixed
                              : std::chars_format::general,
        precision);
  if (result.ec != std::errc{})
    return "ERROR";
  std::string text(buffer, result.ptr);
  if (presentation != 0)
    return text;
  // Documented DART default layout on top of shortest scientific digits.
  const auto epos = text.find('e');
  const int exponent = std::atoi(text.c_str() + epos + 1);
  if (exponent < -4
      || exponent >= std::min(16, std::numeric_limits<T>::digits10 + 1))
    return text;
  std::string out;
  std::size_t start = 0;
  if (text[0] == '-') {
    out = "-";
    start = 1;
  }
  std::string digits;
  for (std::size_t i = start; i < epos; ++i)
    if (text[i] != '.')
      digits += text[i];
  if (exponent < 0)
    return out + "0." + std::string(-exponent - 1, '0') + digits;
  const std::size_t integerDigits = static_cast<std::size_t>(exponent) + 1;
  if (digits.size() <= integerDigits)
    return out + digits + std::string(integerDigits - digits.size(), '0');
  return out + digits.substr(0, integerDigits) + "."
         + digits.substr(integerDigits);
}

template <typename T>
void compare(T value, char presentation = 0, int precision = 0)
{
  if (!std::isfinite(value))
    return;
  std::string format = "{}";
  if (presentation)
    format = "{:." + std::to_string(precision) + presentation + "}";
  ++oracleChecks;
  const std::string expected = reference(value, presentation, precision);
  const std::string actual = logged(format.c_str(), value);
  if (expected == actual)
    return;
  if (failures++ < 32)
    std::cout << "ORACLE MISMATCH bits=" << std::hex << toBits(value)
              << std::dec << " format=" << format << " expected=" << expected
              << " actual=" << actual << "\n";
}

template <typename T>
void differential(std::uint64_t randomCount)
{
  std::vector<T> edge{
      T(0),
      -T(0),
      T(0.1),
      T(-0.1),
      T(0.125),
      T(1.5),
      T(2.5),
      std::numeric_limits<T>::max(),
      std::numeric_limits<T>::lowest(),
      std::numeric_limits<T>::min(),
      std::numeric_limits<T>::denorm_min()};
  // Every binary exponent boundary and its immediate neighbors.
  for (int e
       = std::numeric_limits<T>::min_exponent - std::numeric_limits<T>::digits;
       e <= std::numeric_limits<T>::max_exponent;
       ++e) {
    const T p = std::ldexp(T(1), e);
    if (!std::isfinite(p) || p == 0)
      continue;
    for (const T v :
         {p,
          -p,
          std::nextafter(p, T(0)),
          std::nextafter(p, std::numeric_limits<T>::infinity())})
      edge.push_back(v);
  }
  // Decimal powers and their immediate binary neighbors.
  for (int e = std::numeric_limits<T>::min_exponent10 - 3;
       e <= std::numeric_limits<T>::max_exponent10;
       ++e) {
    const T p = static_cast<T>(std::pow(10.0L, e));
    if (!std::isfinite(p) || p == 0)
      continue;
    edge.push_back(p);
    edge.push_back(std::nextafter(p, T(0)));
    edge.push_back(std::nextafter(p, std::numeric_limits<T>::infinity()));
  }
  // Subnormal, min-normal and max neighborhoods.
  for (Bits<T> b = 1; b < 2048; ++b) {
    edge.push_back(fromBits<T>(b));
    edge.push_back(fromBits<T>(toBits(std::numeric_limits<T>::max()) - b));
    edge.push_back(fromBits<T>(toBits(std::numeric_limits<T>::min()) + b));
  }
  const int digits = std::numeric_limits<T>::max_digits10;
  const int precisions[] = {0, 1, 2, 6, digits};
  const char kinds[] = {'e', 'f', 'g'};
  for (const T v : edge) {
    compare(v);
    for (const char kind : kinds)
      for (const int precision : precisions)
        compare(v, kind, precision);
  }
  for (std::uint64_t i = 0; i < randomCount; ++i) {
    const T v = fromBits<T>(static_cast<Bits<T>>(random64()));
    compare(v);
    if (i < 2000)
      for (const char kind : kinds)
        for (const int precision : {0, 1, 2, 6, digits, 1000})
          compare(v, kind, precision);
  }
  // The rounding modes other than nearest, with flags/errno seen by the sink.
  const int original = std::fegetround();
  for (const int mode : {FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO}) {
    if (std::fesetround(mode) != 0)
      continue;
    for (const T v :
         {T(0.1),
          T(0.125),
          T(1.5),
          T(-1.5),
          std::numeric_limits<T>::denorm_min(),
          std::numeric_limits<T>::max(),
          std::ldexp(T(1), std::numeric_limits<T>::min_exponent / 4)}) {
      compare(v);
      for (const char kind : kinds)
        for (const int precision : {0, 1, 2, 6, 1000})
          compare(v, kind, precision);
      expectTrue(capturedRound == mode, "oracle sink sees caller mode");
    }
    expectTrue(std::fegetround() == mode, "oracle mode restored");
  }
  std::fesetround(original);
}

#endif // DART_CHECK_LOGGING_FLOAT_ORACLE

} // namespace

int main(int argc, char** argv)
{
  const std::uint64_t randomCount
      = argc > 1 ? std::strtoull(argv[1], nullptr, 10) : 200000;

  dart::common::setLogLevel(LogLevel::Trace);
  dart::common::setLogSink([](LogLevel, const std::string& message) {
    captured = message;
    capturedRound = std::fegetround();
    capturedFlags = std::fetestexcept(FE_ALL_EXCEPT);
    capturedErrno = errno;
  });

  fixedCases();
  localeCases();
  environmentCases();
#ifdef DART_CHECK_LOGGING_FLOAT_ALLOCATION_FAILURES
  allocationFailureCases();
  entryAllocationCases();
  callbackDiagnosticCase();
#endif
#ifdef DART_CHECK_LOGGING_FLOAT_ORACLE
  differential<float>(randomCount);
  differential<double>(randomCount);
#else
  (void)randomCount;
#endif
  dart::common::resetLogSink();

  std::cout << "check_logging_float: fixed_checks=" << checks
            << " oracle_checks=" << oracleChecks << " failures=" << failures
#ifdef DART_CHECK_LOGGING_FLOAT_ORACLE
            << " oracle=on"
#else
            << " oracle=off"
#endif
            << "\n";
  if (failures != 0 || checks == 0) {
    std::cout << "FAILED\n";
    return 1;
  }
#ifdef DART_CHECK_LOGGING_FLOAT_ORACLE
  if (oracleChecks == 0) {
    std::cout << "FAILED (no oracle cases ran)\n";
    return 1;
  }
#endif
  return 0;
}
