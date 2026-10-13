#include "CalcEngine.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const double kPi = 3.14159265358979323846;

namespace {

struct Parser {
  const char *s;
  int pos;
  int len;
  double xVal;
  double ansVal;
  bool deg;
  bool failed;
  int depth;
  char msg[28];

  // Bounded recursion: every '(' , function call, unary sign and '^'
  // consumes one depth unit. 20 nested constructs is far beyond any
  // expression the 110-character UI editor can meaningfully hold, and
  // keeps worst-case stack use safe on the ESP32 loop task.
  static const int MAX_DEPTH = 20;
  bool enterDepth() {
    if (++depth > MAX_DEPTH) { fail("Too deep"); return false; }
    return true;
  }
  void leaveDepth() { depth--; }

  void fail(const char *m) {
    if (!failed) {
      failed = true;
      strncpy(msg, m, sizeof(msg) - 1);
      msg[sizeof(msg) - 1] = '\0';
    }
  }
  void skip() {
    while (pos < len && (s[pos] == ' ' || s[pos] == '\t')) pos++;
  }
  char peek() {
    skip();
    return pos < len ? s[pos] : '\0';
  }
  bool eat(char c) {
    if (peek() == c) { pos++; return true; }
    return false;
  }
  double check(double v) {
    if (isnan(v)) { fail("Math ERROR"); return 0; }
    if (isinf(v)) { fail("Overflow ERROR"); return 0; }
    return v;
  }

  double parseExpression() {
    double v = parseTerm();
    while (!failed) {
      char c = peek();
      if (c == '+') { pos++; v = check(v + parseTerm()); }
      else if (c == '-') { pos++; v = check(v - parseTerm()); }
      else break;
    }
    return v;
  }

  bool startsImplicitFactor() {
    char c = peek();
    if (c == '\0') return false;
    if (isdigit((unsigned char)c) || c == '.' || c == '(') return true;
    if (isalpha((unsigned char)c)) return true;  // pi, e, x, Ans, sin(...
    return false;
  }

  double parseTerm() {
    double v = parseUnary();
    while (!failed) {
      char c = peek();
      if (c == '*') { pos++; v = check(v * parseUnary()); }
      else if (c == '/') {
        pos++;
        double d = parseUnary();
        if (!failed) {
          if (d == 0.0) { fail("Divide by 0"); return 0; }
          v = check(v / d);
        }
      } else if (c == '%') {
        pos++;
        char nc = peek();
        // Dual role: "10%3" = modulo, but "50%" / "50%+10" = percent.
        if (nc == '\0' || nc == ')' || nc == '+' || nc == '-' ||
            nc == '*' || nc == '/' || nc == '^' || nc == '!') {
          v = check(v / 100.0);
        } else {
          double d = parseUnary();
          if (!failed) {
            if (d == 0.0) { fail("Divide by 0"); return 0; }
            v = check(fmod(v, d));
          }
        }
      } else if (startsImplicitFactor()) {
        v = check(v * parseUnary());  // implicit multiplication
      } else break;
    }
    return v;
  }

  double parseUnary() {
    char c = peek();
    if (c == '-' || c == '+') {
      bool neg = (c == '-');
      pos++;
      if (!enterDepth()) return 0;
      double v = parseUnary();
      leaveDepth();
      if (failed) return 0;
      return neg ? check(-v) : v;
    }
    return parsePower();
  }

  double safePow(double base, double ex) {
    if (base == 0.0 && ex <= 0.0) { fail("Math ERROR"); return 0; }
    if (base < 0.0) {
      double r = round(ex);
      if (fabs(ex - r) > 1e-12) { fail("Domain ERROR"); return 0; }
    }
    return check(pow(base, ex));
  }

  double parsePower() {
    double base = parsePostfix();
    if (!failed && peek() == '^') {
      pos++;
      if (!enterDepth()) return 0;
      double ex = parseUnary();  // right-assoc + allows 2^-3
      leaveDepth();
      if (!failed) return safePow(base, ex);
      return 0;
    }
    return base;
  }

  static unsigned long long ullGcd(unsigned long long a, unsigned long long b) {
    while (b) { unsigned long long tmp = a % b; a = b; b = tmp; }
    return a;
  }

  double factorial(double v) {
    double r = round(v);
    if (v < 0.0 && fabs(v - r) < 1e-12) { fail("Domain ERROR"); return 0; }
    // Exact integer factorials 0!..20! in 64-bit integer arithmetic
    // (tgamma can be 1 ulp off; 20! = 2432902008176640000 fits in u64).
    if (fabs(v - r) < 1e-12 && r >= 0.0 && r <= 20.0) {
      unsigned long long acc = 1;
      for (unsigned long long i = 2; i <= (unsigned long long)r; i++) acc *= i;
      return (double)acc;
    }
    return check(tgamma(v + 1.0));  // Gamma(n+1) for non-integers / n>20
  }

  double parsePostfix() {
    double v = parsePrimary();
    while (!failed && peek() == '!') {
      pos++;
      v = factorial(v);
    }
    return v;
  }

  double applyFunction(const char *name, double a) {
    const double toRad = deg ? (kPi / 180.0) : 1.0;
    const double fromRad = deg ? (180.0 / kPi) : 1.0;
    if (!strcmp(name, "sin")) return check(sin(a * toRad));
    if (!strcmp(name, "cos")) return check(cos(a * toRad));
    if (!strcmp(name, "tan")) {
      double c = cos(a * toRad);
      if (fabs(c) < 1e-15) { fail("Domain ERROR"); return 0; }
      return check(tan(a * toRad));
    }
    if (!strcmp(name, "asin")) {
      if (a < -1.0 || a > 1.0) { fail("Domain ERROR"); return 0; }
      return check(asin(a) * fromRad);
    }
    if (!strcmp(name, "acos")) {
      if (a < -1.0 || a > 1.0) { fail("Domain ERROR"); return 0; }
      return check(acos(a) * fromRad);
    }
    if (!strcmp(name, "atan")) return check(atan(a) * fromRad);
    if (!strcmp(name, "sinh")) return check(sinh(a));
    if (!strcmp(name, "cosh")) return check(cosh(a));
    if (!strcmp(name, "tanh")) return check(tanh(a));
    if (!strcmp(name, "ln")) {
      if (a <= 0.0) { fail("Domain ERROR"); return 0; }
      return check(log(a));
    }
    if (!strcmp(name, "log")) {
      if (a <= 0.0) { fail("Domain ERROR"); return 0; }
      return check(log10(a));
    }
    if (!strcmp(name, "log2")) {
      if (a <= 0.0) { fail("Domain ERROR"); return 0; }
      return check(log2(a));
    }
    if (!strcmp(name, "sqrt")) {
      if (a < 0.0) { fail("Domain ERROR"); return 0; }
      return check(sqrt(a));
    }
    if (!strcmp(name, "cbrt")) return check(cbrt(a));
    if (!strcmp(name, "abs")) return fabs(a);
    if (!strcmp(name, "exp")) return check(exp(a));
    // reciprocal trig
    if (!strcmp(name, "sec")) {
      double c = cos(a * toRad);
      if (fabs(c) < 1e-15) { fail("Domain ERROR"); return 0; }
      return check(1.0 / c);
    }
    if (!strcmp(name, "csc")) {
      double s = sin(a * toRad);
      if (fabs(s) < 1e-15) { fail("Domain ERROR"); return 0; }
      return check(1.0 / s);
    }
    if (!strcmp(name, "cot")) {
      double s = sin(a * toRad);
      if (fabs(s) < 1e-15) { fail("Domain ERROR"); return 0; }
      return check(cos(a * toRad) / s);
    }
    if (!strcmp(name, "asec")) {
      if (a > -1.0 && a < 1.0) { fail("Domain ERROR"); return 0; }
      return check(acos(1.0 / a) * fromRad);
    }
    if (!strcmp(name, "acsc")) {
      if (a > -1.0 && a < 1.0) { fail("Domain ERROR"); return 0; }
      return check(asin(1.0 / a) * fromRad);
    }
    if (!strcmp(name, "acot")) {
      if (a == 0.0) return check((kPi * 0.5) * fromRad);  // limit, no 1/0
      return check(atan(1.0 / a) * fromRad);
    }
    // hyperbolic inverses
    if (!strcmp(name, "asinh")) return check(asinh(a));
    if (!strcmp(name, "acosh")) {
      if (a < 1.0) { fail("Domain ERROR"); return 0; }
      return check(acosh(a));
    }
    if (!strcmp(name, "atanh")) {
      if (a <= -1.0 || a >= 1.0) { fail("Domain ERROR"); return 0; }
      return check(atanh(a));
    }
    // rounding / integer functions
    if (!strcmp(name, "floor")) return check(floor(a));
    if (!strcmp(name, "ceil")) return check(ceil(a));
    if (!strcmp(name, "round")) return check(round(a));
    if (!strcmp(name, "trunc")) return check(trunc(a));
    if (!strcmp(name, "frac")) return check(a - trunc(a));
    if (!strcmp(name, "sgn")) return (a > 0) ? 1.0 : ((a < 0) ? -1.0 : 0.0);
    fail("Syntax ERROR");
    return 0;
  }

  double applyFunction2(const char *name, double a, double b) {
    if (!strcmp(name, "min")) return a < b ? a : b;
    if (!strcmp(name, "max")) return a > b ? a : b;
    if (!strcmp(name, "mod")) {
      if (b == 0.0) { fail("Divide by 0"); return 0; }
      return check(fmod(a, b));
    }
    if (!strcmp(name, "hypot")) return check(hypot(a, b));
    if (!strcmp(name, "atan2")) {
      const double fromRad = deg ? (180.0 / kPi) : 1.0;
      return check(atan2(a, b) * fromRad);
    }
    if (!strcmp(name, "gcd") || !strcmp(name, "lcm")) {
      double ra = round(a), rb = round(b);
      if (fabs(a - ra) > 1e-9 || fabs(b - rb) > 1e-9 || fabs(ra) > 1e12 || fabs(rb) > 1e12) {
        fail("Domain ERROR"); return 0;
      }
      unsigned long long x = (unsigned long long)fabsl((long double)ra);
      unsigned long long y = (unsigned long long)fabsl((long double)rb);
      unsigned long long g = ullGcd(x, y);
      if (!strcmp(name, "gcd")) return (double)g;  // gcd(0,0)=0 (std::gcd convention)
      if (g == 0) return 0;                        // lcm(0,0)=0
      unsigned long long lx = x / g;
      if (y != 0 && lx > ~0ULL / y) { fail("Overflow ERROR"); return 0; }
      return (double)(lx * y);
    }
    if (!strcmp(name, "nPr") || !strcmp(name, "nCr")) {
      double rn = round(a), rr = round(b);
      if (fabs(a - rn) > 1e-9 || fabs(b - rr) > 1e-9 || rn < 0 || rr < 0 || rr > rn ||
          rn > 10000.0) {  // explicit bound: keeps loops and integers sane
        fail("Domain ERROR"); return 0;
      }
      // Exact 64-bit integer arithmetic (no floating-point factorials).
      // If the true result exceeds 2^64-1 that is an Overflow ERROR,
      // never a silently rounded "integer". Above 2^53 the returned
      // double is the nearest representable value (displayed as such).
      unsigned long long n = (unsigned long long)rn;
      unsigned long long r = (unsigned long long)rr;
      unsigned long long res = 1;
      if (!strcmp(name, "nPr")) {
        for (unsigned long long i = 0; i < r; i++) {
          unsigned long long term = n - i;
          if (term != 0 && res > ~0ULL / term) { fail("Overflow ERROR"); return 0; }
          res *= term;
        }
        return (double)res;
      }
      unsigned long long k = r < (n - r) ? r : (n - r);
      for (unsigned long long i = 1; i <= k; i++) {
        unsigned long long numTerm = n - k + i, den = i;
        unsigned long long g1 = ullGcd(res, den); res /= g1; den /= g1;
        unsigned long long g2 = ullGcd(numTerm, den); numTerm /= g2; den /= g2;
        if (den != 1) { fail("Math ERROR"); return 0; }  // unreachable for valid C(n,k)
        if (numTerm != 0 && res > ~0ULL / numTerm) { fail("Overflow ERROR"); return 0; }
        res *= numTerm;
      }
      return (double)res;
    }
    fail("Syntax ERROR");
    return 0;
  }

  double parsePrimary() {
    char c = peek();
    if (c == '\0') { fail("Syntax ERROR"); return 0; }

    if (c == '(') {
      pos++;
      if (!enterDepth()) return 0;
      double v = parseExpression();
      leaveDepth();
      if (failed) return 0;
      if (!eat(')')) { fail("Missing )"); return 0; }
      return v;
    }

    if (isdigit((unsigned char)c) || c == '.') {
      // Reject hex floats ("0x10") that strtod would otherwise accept.
      if (c == '0' && pos + 1 < len && (s[pos + 1] == 'x' || s[pos + 1] == 'X')) {
        fail("Syntax ERROR"); return 0;
      }
      const char *start = s + pos;
      char *end = nullptr;
      double v = strtod(start, &end);
      if (end == start) { fail("Syntax ERROR"); return 0; }
      pos += (int)(end - start);
      if (pos < len && s[pos] == '.') { fail("Syntax ERROR"); return 0; }
      return check(v);
    }

    if (isalpha((unsigned char)c)) {
      char name[12];
      int n = 0;
      bool tooLong = false;
      while (pos < len && isalnum((unsigned char)s[pos])) {  // log2, atan2
        if (n < (int)sizeof(name) - 1) name[n++] = s[pos];
        else tooLong = true;  // never silently truncate an identifier
        pos++;
      }
      name[n] = '\0';
      if (tooLong) { fail("Unknown name"); return 0; }
      if (!strcmp(name, "pi")) return kPi;
      if (!strcmp(name, "tau")) return 2.0 * kPi;
      if (!strcmp(name, "e")) return exp(1.0);
      if (!strcmp(name, "phi")) return 1.6180339887498949;
      if (!strcmp(name, "Ans")) return ansVal;
      if (!strcmp(name, "x")) return xVal;
      static const char *kFunc1[] = {
        "sin","cos","tan","asin","acos","atan","sec","csc","cot","asec","acsc","acot",
        "sinh","cosh","tanh","asinh","acosh","atanh","ln","log","log2","sqrt","cbrt",
        "abs","exp","floor","ceil","round","trunc","frac","sgn"
      };
      static const char *kFunc2[] = {
        "min","max","mod","hypot","atan2","gcd","lcm","nPr","nCr"
      };
      int kind = 0;
      for (size_t i = 0; i < sizeof(kFunc1)/sizeof(kFunc1[0]); i++)
        if (!strcmp(name, kFunc1[i])) kind = 1;
      for (size_t i = 0; i < sizeof(kFunc2)/sizeof(kFunc2[0]); i++)
        if (!strcmp(name, kFunc2[i])) kind = 2;
      if (kind == 0) { fail("Unknown name"); return 0; }
      if (!eat('(')) { fail("Syntax ERROR"); return 0; }
      if (!enterDepth()) return 0;
      double arg = parseExpression();
      if (failed) { leaveDepth(); return 0; }
      if (kind == 2) {
        if (!eat(',')) { fail("Wrong arguments"); leaveDepth(); return 0; }
        double arg2 = parseExpression();
        leaveDepth();
        if (failed) return 0;
        if (!eat(')')) { fail("Missing )"); return 0; }
        return applyFunction2(name, arg, arg2);
      }
      leaveDepth();
      if (eat(',')) { fail("Wrong arguments"); return 0; }
      if (!eat(')')) { fail("Missing )"); return 0; }
      return applyFunction(name, arg);
    }

    fail("Syntax ERROR");
    return 0;
  }
};

}  // namespace

bool evaluateExpression(const char *expr, double xValue, double ansValue,
                        bool degreeMode, EvalResult &out) {
  out.ok = false;
  out.value = 0;
  out.error[0] = '\0';
  if (!expr || !expr[0]) {
    strncpy(out.error, "Empty", sizeof(out.error) - 1);
    return false;
  }
  int exprLen = (int)strlen(expr);
  if (exprLen > 256) {  // engine hard limit (the UI editor caps at 110)
    strncpy(out.error, "Too long", sizeof(out.error) - 1);
    out.error[sizeof(out.error) - 1] = '\0';
    return false;
  }
  Parser p;
  p.s = expr;
  p.pos = 0;
  p.depth = 0;
  p.len = exprLen;
  p.xVal = xValue;
  p.ansVal = ansValue;
  p.deg = degreeMode;
  p.failed = false;
  p.msg[0] = '\0';
  double v = p.parseExpression();
  if (!p.failed && p.peek() != '\0') p.fail("Syntax ERROR");
  if (p.failed || isnan(v) || isinf(v)) {
    strncpy(out.error, p.failed ? p.msg : "Math ERROR", sizeof(out.error) - 1);
    out.error[sizeof(out.error) - 1] = '\0';
    return false;
  }
  out.ok = true;
  out.value = v;
  return true;
}

bool doubleToFraction(double value, long long &num, long long &den) {
  num = 0; den = 1;
  if (!isfinite(value)) return false;
  if (value == 0.0) { num = 0; den = 1; return true; }
  int sign = value < 0 ? -1 : 1;
  long double v = fabsl((long double)value);

  long double nearest = roundl(v);
  long double tolInt = 1e-12L * (v > 1.0L ? v : 1.0L);
  if (fabsl(v - nearest) <= tolInt && nearest < 9.0e15L) {
    num = (long long)(sign * (long long)nearest);
    den = 1;
    return true;
  }
  if (v > 9.0e15L) return false;

  // Continued fractions. Accumulators are long double because the
  // Xtensa toolchain has no __int128; acceptance is capped so the
  // final cast to long long is always exact and in range.
  long double pPrev2 = 0, pPrev1 = 1;
  long double qPrev2 = 1, qPrev1 = 0;
  long double x = v;
  for (int i = 0; i < 40; i++) {
    long double aLd = floorl(x);
    if (aLd > 9.0e15L) return false;
    long double p = aLd * pPrev1 + pPrev2;
    long double q = aLd * qPrev1 + qPrev2;
    if (q > 100000.0L || p > 9.0e15L) return false;
    pPrev2 = pPrev1; pPrev1 = p;
    qPrev2 = qPrev1; qPrev1 = q;
    if (q > 0) {
      long double approx = p / q;
      long double err = fabsl(approx - v);
      long double tol = 1e-12L * (v > 1.0L ? v : 1.0L);
      if (err <= tol) {
        num = (long long)(sign * (long long)p);
        den = (long long)q;
        return true;
      }
    }
    long double frac = x - aLd;
    if (frac < 1e-15L) return false;
    x = 1.0L / frac;
  }
  return false;
}

void formatDecimal(double value, char *out, size_t outSize) {
  if (!out || outSize == 0) return;
  if (isnan(value) || isinf(value)) {
    snprintf(out, outSize, "Math ERROR");
    return;
  }
  if (value == 0.0) value = 0.0;  // avoid "-0"
  snprintf(out, outSize, "%.10g", value);
}

void formatSmart(double value, bool preferFraction,
                 char *primary, size_t primarySize,
                 char *secondary, size_t secondarySize) {
  if (primary && primarySize) primary[0] = '\0';
  if (secondary && secondarySize) secondary[0] = '\0';
  if (!primary || primarySize == 0) return;
  if (isnan(value) || isinf(value)) {
    snprintf(primary, primarySize, "Math ERROR");
    return;
  }
  long long n = 0, d = 1;
  bool hasFrac = doubleToFraction(value, n, d);
  char fracBuf[48] = "";
  char decBuf[40] = "";
  formatDecimal(value, decBuf, sizeof(decBuf));
  if (hasFrac) {
    if (d == 1) snprintf(fracBuf, sizeof(fracBuf), "%lld", n);
    else snprintf(fracBuf, sizeof(fracBuf), "%lld/%lld", n, d);
  }
  if (preferFraction && hasFrac) {
    snprintf(primary, primarySize, "%s", fracBuf);
    if (d != 1 && secondary && secondarySize)
      snprintf(secondary, secondarySize, "= %s", decBuf);
  } else {
    snprintf(primary, primarySize, "%s", decBuf);
    if (hasFrac && d != 1 && secondary && secondarySize)
      snprintf(secondary, secondarySize, "= %s", fracBuf);
  }
}
