#include "MathTree.h"
#include <ctype.h>
#include <string.h>

namespace {

struct TParser {
  const char *s;
  int pos, len, depth;
  MathTree *tree;
  bool failed;

  void fail(const char *m) {
    if (!failed) {
      failed = true;
      strncpy(tree->error, m, sizeof(tree->error) - 1);
      tree->error[sizeof(tree->error) - 1] = '\0';
    }
  }
  bool enter() {
    if (++depth > MATH_MAX_DEPTH) { fail("Too deep"); return false; }
    return true;
  }
  void skip() { while (pos < len && (s[pos] == ' ' || s[pos] == '\t')) pos++; }
  char peek() { skip(); return pos < len ? s[pos] : '\0'; }

  int makeNode(int type, int a, int b, int start, int end, const char *name) {
    if (tree->count >= MATH_MAX_NODES) { fail("Too complex"); return -1; }
    int idx = tree->count++;
    MNode &n = tree->nodes[idx];
    n.type = (unsigned char)type;
    n.flags = 0;
    n.a = a; n.b = b;
    n.start = start; n.end = end;
    n.name[0] = '\0';
    if (name) {
      strncpy(n.name, name, sizeof(n.name) - 1);
      n.name[sizeof(n.name) - 1] = '\0';
    }
    return idx;
  }

  int parseExpression() {
    int left = parseTerm();
    if (left < 0) return -1;
    while (!failed) {
      char c = peek();
      if (c != '+' && c != '-') break;
      pos++;
      int start = tree->nodes[left].start;
      int right = parseTerm();
      if (right < 0) return -1;
      left = makeNode(c == '+' ? MN_ADD : MN_SUB, left, right,
                      start, tree->nodes[right].end, nullptr);
      if (left < 0) return -1;
    }
    return failed ? -1 : left;
  }

  bool startsFactor() {
    char c = peek();
    return c && (isdigit((unsigned char)c) || c == '.' || c == '(' || isalpha((unsigned char)c));
  }

  int parseTerm() {
    int left = parseUnary();
    if (left < 0) return -1;
    while (!failed) {
      char c = peek();
      if (c == '*' || c == '/') {
        pos++;
        int start = tree->nodes[left].start;
        int right = parseUnary();
        if (right < 0) return -1;
        left = makeNode(c == '*' ? MN_MUL : MN_DIV, left, right,
                        start, tree->nodes[right].end, nullptr);
        if (left < 0) return -1;
      } else if (c == '%') {
        // Mirror the evaluator's dual role: operand following = modulo,
        // otherwise postfix percent.
        int save = pos;
        pos++;
        char nc = peek();
        if (nc == '\0' || nc == ')' || nc == '+' || nc == '-' || nc == '*' ||
            nc == '/' || nc == '^' || nc == '!') {
          int start = tree->nodes[left].start;
          left = makeNode(MN_PERCENT, left, -1, start, pos, nullptr);
          if (left < 0) return -1;
        } else {
          int start = tree->nodes[left].start;
          int right = parseUnary();
          if (right < 0) return -1;
          left = makeNode(MN_MOD, left, right, start, tree->nodes[right].end, nullptr);
          if (left < 0) return -1;
        }
        (void)save;
      } else if (startsFactor()) {
        int start = tree->nodes[left].start;
        int right = parseUnary();
        if (right < 0) return -1;
        int node = makeNode(MN_MUL, left, right, start, tree->nodes[right].end, nullptr);
        if (node < 0) return -1;
        tree->nodes[node].flags |= MFLAG_IMPLICIT;
        left = node;
      } else break;
    }
    return failed ? -1 : left;
  }

  int parseUnary() {
    char c = peek();
    if (c == '-' || c == '+') {
      int start = pos;
      pos++;
      if (c == '+') return parseUnary();  // unary plus: no node needed
      if (!enter()) return -1;
      int operand = parseUnary();
      depth--;
      if (operand < 0) return -1;
      return makeNode(MN_NEG, operand, -1, start, tree->nodes[operand].end, nullptr);
    }
    return parsePower();
  }

  int parsePower() {
    int base = parsePostfix();
    if (base < 0) return -1;
    if (!failed && peek() == '^') {
      pos++;
      if (!enter()) return -1;
      int ex = parseUnary();  // right-associative
      depth--;
      if (ex < 0) return -1;
      return makeNode(MN_POW, base, ex, tree->nodes[base].start,
                      tree->nodes[ex].end, nullptr);
    }
    return base;
  }

  int parsePostfix() {
    int v = parsePrimary();
    if (v < 0) return -1;
    while (!failed && peek() == '!') {
      pos++;
      v = makeNode(MN_FACT, v, -1, tree->nodes[v].start, pos, nullptr);
      if (v < 0) return -1;
    }
    return failed ? -1 : v;
  }

  int parsePrimary() {
    char c = peek();
    int start = pos;
    if (c == '\0') { fail("Syntax ERROR"); return -1; }

    if (c == '(') {
      pos++;
      if (!enter()) return -1;
      int inner = parseExpression();
      depth--;
      if (inner < 0) return -1;
      skip();
      // Display-only tolerance: a ')' missing at END of input is an
      // implied close (sqrt(x renders as a radical while typing).
      // The evaluator stays strict and rejects this on '='.
      if (pos < len) {
        if (s[pos] != ')') { fail("Missing )"); return -1; }
        pos++;
      }
      tree->nodes[inner].start = start;  // span covers the parentheses,
      tree->nodes[inner].end = pos;      // which the renderer may redraw
      return inner;
    }

    if (isdigit((unsigned char)c) || c == '.') {
      // Manual number scan mirroring strtod's span (incl. 2e3, 1.5e-2),
      // rejecting hex and multi-dot forms exactly like the evaluator.
      if (c == '0' && pos + 1 < len && (s[pos + 1] == 'x' || s[pos + 1] == 'X')) {
        fail("Syntax ERROR"); return -1;
      }
      bool anyDigit = false, seenDot = false;
      while (pos < len && (isdigit((unsigned char)s[pos]) || s[pos] == '.')) {
        if (s[pos] == '.') { if (seenDot) { fail("Syntax ERROR"); return -1; } seenDot = true; }
        else anyDigit = true;
        pos++;
      }
      if (!anyDigit) { fail("Syntax ERROR"); return -1; }
      if (pos < len && (s[pos] == 'e' || s[pos] == 'E')) {
        int save = pos;
        pos++;
        if (pos < len && (s[pos] == '+' || s[pos] == '-')) pos++;
        if (pos < len && isdigit((unsigned char)s[pos])) {
          while (pos < len && isdigit((unsigned char)s[pos])) pos++;
        } else pos = save;  // "1e" leaves 'e' as the constant, like strtod
      }
      return makeNode(MN_NUM, -1, -1, start, pos, nullptr);
    }

    if (isalpha((unsigned char)c)) {
      char name[12];
      int n = 0;
      bool tooLong = false;
      while (pos < len && isalnum((unsigned char)s[pos])) {
        if (n < (int)sizeof(name) - 1) name[n++] = s[pos];
        else tooLong = true;
        pos++;
      }
      name[n] = '\0';
      if (tooLong) { fail("Unknown name"); return -1; }
      if (!strcmp(name, "pi") || !strcmp(name, "tau") || !strcmp(name, "e") ||
          !strcmp(name, "phi") || !strcmp(name, "Ans"))
        return makeNode(MN_CONST, -1, -1, start, pos, name);
      if (!strcmp(name, "x")) return makeNode(MN_VAR, -1, -1, start, pos, name);
      if (n == 1) return makeNode(MN_SYMBOL, -1, -1, start, pos, name);
      // function call (known or generic, for graceful half-typed input)
      skip();
      if (pos >= len || s[pos] != '(') { fail("Syntax ERROR"); return -1; }
      pos++;
      if (!enter()) return -1;
      int arg1 = parseExpression();
      if (arg1 < 0) { depth--; return -1; }
      int node;
      if (peek() == ',') {
        pos++;
        int arg2 = parseExpression();
        depth--;
        if (arg2 < 0) return -1;
        skip();
        if (pos < len) {  // implied close at EOF, as above
          if (s[pos] != ')') { fail("Missing )"); return -1; }
          pos++;
        }
        node = makeNode(MN_FUNC2, arg1, arg2, start, pos, name);
      } else {
        depth--;
        skip();
        if (pos < len) {
          if (s[pos] != ')') { fail("Missing )"); return -1; }
          pos++;
        }
        node = makeNode(MN_FUNC, arg1, -1, start, pos, name);
      }
      return node;
    }

    fail("Syntax ERROR");
    return -1;
  }
};

}  // namespace

bool parseMathTree(const char *src, MathTree &tree) {
  tree.count = 0;
  tree.root = -1;
  tree.error[0] = '\0';
  if (!src || !src[0]) {
    strncpy(tree.error, "Empty", sizeof(tree.error) - 1);
    return false;
  }
  if ((int)strlen(src) > 256) {
    strncpy(tree.error, "Too long", sizeof(tree.error) - 1);
    return false;
  }
  TParser p;
  p.s = src; p.pos = 0; p.len = (int)strlen(src);
  p.depth = 0; p.tree = &tree; p.failed = false;
  int root = p.parseExpression();
  if (!p.failed && root >= 0) {
    p.skip();
    if (p.pos < p.len) p.fail("Syntax ERROR");
  }
  if (p.failed || root < 0) {
    if (!tree.error[0]) strncpy(tree.error, "Syntax ERROR", sizeof(tree.error) - 1);
    tree.error[sizeof(tree.error) - 1] = '\0';
    return false;
  }
  tree.root = root;
  return true;
}
