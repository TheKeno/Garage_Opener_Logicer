// Test runner.  ./run_tests [-v] [filter...]
//   -v       echo the sketch's serial output while tests run
//   filter   only run tests whose "group.name" contains one of these
#include "check.h"

#include <cstring>
#include <exception>

namespace check {

std::vector<Test> &registry() {
  static std::vector<Test> tests;
  return tests;
}

static std::vector<std::string> failures;

void fail(const char *file, int line, const std::string &what) {
  std::ostringstream o;
  o << file << ":" << line << ": " << what;
  if (sim::current) {
    auto *s = sim::current;
    o << "\n        at t=" << s->secs() << " s, LCD [" << s->lcd(0) << "] [" << s->lcd(1) << "]";
  }
  failures.push_back(o.str());
}

}  // namespace check

int main(int argc, char **argv) {
  bool verbose = false;
  std::vector<std::string> filters;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "-v")) verbose = sim::echo_serial = true;
    else filters.push_back(argv[i]);
  }

  int run = 0, failed = 0, known = 0, unexpectedly_ok = 0;
  const char *group = "";
  for (auto &t : check::registry()) {
    std::string full = std::string(t.group) + "." + t.name;
    bool wanted = filters.empty();
    for (auto &f : filters) wanted |= full.find(f) != std::string::npos;
    if (!wanted) continue;

    if (strcmp(group, t.group)) printf("%s\n", group = t.group);
    check::failures.clear();
    try {
      if (verbose) printf("  -- %s\n", t.name);
      t.fn();
    } catch (const std::exception &e) {
      check::failures.push_back(std::string("exception: ") + e.what());
    }
    if (verbose) fflush(stdout);
    run++;

    bool ok = check::failures.empty();
    const char *verdict;
    if (t.known_issue) {
      verdict = ok ? "FIXED? (known issue no longer reproduces)" : "known issue";
      ok ? unexpectedly_ok++ : known++;
    } else {
      verdict = ok ? "ok" : "FAIL";
      if (!ok) failed++;
    }
    printf("  %-58s %s\n", t.name, verdict);
    if (t.known_issue && !ok) printf("      %s\n", t.known_issue);
    if (!t.known_issue || verbose)
      for (auto &f : check::failures) printf("      %s\n", f.c_str());
  }

  printf("\n%d tests, %d failed", run, failed);
  if (known) printf(", %d known issue%s", known, known == 1 ? "" : "s");
  if (unexpectedly_ok) printf(", %d known issue%s now passing", unexpectedly_ok, unexpectedly_ok == 1 ? "" : "s");
  printf("\n");
  return failed ? 1 : 0;
}
