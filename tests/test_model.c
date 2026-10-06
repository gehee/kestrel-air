#include <stdio.h>
#include <stdlib.h>

#include "unit/model.h"
#include "check.h"

static const char *write_file(const char *text) {
    static char path[] = "/tmp/ka-test-board-type";
    FILE *f = fopen(path, "w");
    fputs(text, f);
    fclose(f);
    return path;
}

void test_model(void) {
    unsetenv("KA_BOARD_TYPE");
    CHECK(model_init("/nonexistent") == 0);     // nothing at all: a Lite
    CHECK(model_board_type() == 482 && model_prj() == 4 && !model_lite_plus());

    CHECK(model_init(write_file("472\n")) == 0); // stock's file
    CHECK(model_board_type() == 472 && model_prj() == 7 && model_lite_plus());

    setenv("KA_BOARD_TYPE", "482", 1);          // the variable wins over the file
    CHECK(model_init(write_file("472\n")) == 0);
    CHECK(model_board_type() == 482 && !model_lite_plus());

    setenv("KA_BOARD_TYPE", "472", 1);
    CHECK(model_init("/nonexistent") == 0 && model_prj() == 7);

    setenv("KA_BOARD_TYPE", "4861", 1);         // a GT: not ours
    CHECK(model_init("/nonexistent") == -1);
    unsetenv("KA_BOARD_TYPE");
    CHECK(model_init(write_file("492\n")) == -1);   // the RC: not ours either
    remove("/tmp/ka-test-board-type");
}
