// Host-side tests of the parts that need no SDK and no radio: make test.
#include "check.h"

int checks, failures;

int main(void) {
    test_protocol();
    test_crc();
    test_frame();
    test_json();
    test_ring();
    test_settings();
    printf("%d checks, %d failed\n", checks, failures);
    return failures != 0;
}
