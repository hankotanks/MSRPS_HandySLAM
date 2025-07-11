#include "PyScript.h"

int main() {
    PyScript test_script("testing");
    test_script.call("print_message", "s", "Hello, World!");
    return 0;
}