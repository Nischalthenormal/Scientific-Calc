#pragma once
#include <stddef.h>

// ---------------------------------------------------------------------
// Syntax tree for DISPLAY. The evaluator (CalcEngine) stays the single
// authority for numbers; this parser only recovers the *structure* of
// the same ASCII expression so it can be typeset mathematically
// (fractions, radicals, superscripts) instead of printed as raw text.
//
// It intentionally mirrors the CalcEngine grammar and limits:
//   expr := term (('+'|'-') term)*
//   term := unary (('*'|'/'|'%'|implicit) unary)*
//   unary := ('-'|'+') unary | power
//   power := postfix ('^' unary)?        (right-associative)
//   postfix := primary '!'*
// Differences from the evaluator, by design (display only):
//   - any single letter (a, b, theta-variables...) is a SYMBOL node;
//   - unknown multi-letter names followed by '(' become generic FUNC
//     nodes, so a half-typed function still renders gracefully.
// Invalid structure returns false; callers fall back to plain text.
// Node pool and depth are bounded: no heap, no unbounded recursion.
// ---------------------------------------------------------------------

#define MATH_MAX_NODES 160
#define MATH_MAX_DEPTH 20

enum MNodeType {
  MN_NUM = 0,   // numeric literal          (span = literal text)
  MN_CONST,     // pi tau e phi Ans         (name holds the token)
  MN_VAR,       // x
  MN_SYMBOL,    // any other single letter  (name holds it)
  MN_NEG,       // unary minus              (a)
  MN_ADD, MN_SUB, MN_MUL, MN_DIV, MN_MOD,  // (a, b); MUL flag: implicit
  MN_POW,       // (a=base, b=exponent)
  MN_FACT,      // (a)
  MN_PERCENT,   // (a)   postfix %
  MN_FUNC,      // name(arg)                (a)
  MN_FUNC2      // name(arg1, arg2)         (a, b)
};

#define MFLAG_IMPLICIT 0x01

struct MNode {
  unsigned char type;
  unsigned char flags;
  int a, b;          // child node indices, -1 = none
  int start, end;    // source span [start, end) for cursor mapping
  char name[12];     // function / constant / symbol token
};

struct MathTree {
  MNode nodes[MATH_MAX_NODES];
  int count;
  int root;
  char error[28];
};

// Parses src into tree (bounded pool owned by caller). Returns false and
// sets tree.error on invalid structure. tree.root is valid on success.
bool parseMathTree(const char *src, MathTree &tree);
