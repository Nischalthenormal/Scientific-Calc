#pragma once
#include <stddef.h>

// Pure calculation engine: no Arduino / TFT dependencies.
// This makes the maths independently testable and keeps UI bugs out of it.
//
// DOCUMENTED SEMANTICS (verified by tests/engine_tests.cpp):
//  Grammar (loosest to tightest):
//    expr    := term (('+'|'-') term)*
//    term    := unary (('*'|'/'|'%'|implicit) unary)*
//    unary   := ('-'|'+') unary | power
//    power   := postfix ('^' unary)?          // right-associative
//    postfix := primary '!'*
//    primary := number | '(' expr ')' | constant | variable |
//               func1 '(' expr ')' | func2 '(' expr ',' expr ')'
//  Consequences: -2^2 = -(2^2) = -4;  2^3^2 = 2^(3^2) = 512;  2^-3 = 1/8.
//  '%' is dual-role: with an operand following it is modulo (10%3 = 1);
//  at end / before an operator it is percent (50% = 1/2, 50%+10 = 10.5).
//  '!' is the Gamma extension for non-integers, exact u64 factorials
//  for integers 0..20. 0^0 is a Math ERROR (indeterminate).
//  Identifiers are case-sensitive (Ans, nPr, nCr are mixed-case on
//  purpose; PI / Sin are rejected), never truncated, and unknown names
//  fail with "Unknown name". Wrong argument counts fail, too.
//  Numbers are decimal (scientific notation allowed; hex rejected).
//  Limits: expression length <= 256 chars, nesting depth <= 20
//  ('(' / function / unary / '^' each consume one unit).

struct EvalResult {
  bool ok;
  double value;
  char error[28];
};

bool evaluateExpression(const char *expr, double xValue, double ansValue,
                        bool degreeMode, EvalResult &out);

// Exact rational approximation by continued fractions.
// Accepts p/q only when it reproduces 'value' to ~1e-12 relative error
// with denominator <= 100000, so sqrt(2) / pi are NOT faked as fractions.
bool doubleToFraction(double value, long long &num, long long &den);

// Plain decimal: %.10g style, trailing clutter removed by printf %g.
void formatDecimal(double value, char *out, size_t outSize);

// Smart Casio-style formatting:
//  - preferFraction=true : primary is p/q (or integer), secondary is decimal.
//  - preferFraction=false: primary is decimal,      secondary is p/q.
// If no trustworthy fraction exists, primary is decimal and secondary empty.
void formatSmart(double value, bool preferFraction,
                 char *primary, size_t primarySize,
                 char *secondary, size_t secondarySize);
