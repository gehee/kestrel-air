#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "check.h"
#include "common/json.h"

void test_json(void) {
    const char *doc = "{ \"a\": 12, \"list\": [1, \"0x10\", \"25\"], \"s\": \"x y\", \"o\": {} }";
    const char *p = doc;
    json_node *root = json_parse(&p);
    CHECK(root->type == 'o');
    json_node *a = root->kids, *list = a->next, *s = list->next, *o = s->next;
    CHECK(!strcmp(a->key, "a") && a->type == 'n' && json_num(a) == 12);
    CHECK(list->type == 'a' && json_num(list->kids) == 1);
    CHECK(json_num(list->kids->next) == 0x10);
    CHECK(json_num(list->kids->next->next) == 0x25);   // quoted digits read as hex, as stock does
    CHECK(s->type == 's' && !strcmp(s->text, "x y"));
    CHECK(o->type == 'o' && !o->kids && !o->next);

    // Written back as jansson does it: four-space indent, no trailing newline.
    char path[] = "/tmp/kestrel-air-test-XXXXXX";
    int fd = mkstemp(path);
    CHECK(fd >= 0);
    close(fd);
    CHECK(json_write(path, root) == 0);
    char *out = json_slurp(path);
    CHECK(out && !strcmp(out, "{\n    \"a\": 12,\n    \"list\": [\n        1,\n        \"0x10\",\n"
                             "        \"25\"\n    ],\n    \"s\": \"x y\",\n    \"o\": {}\n}"));
    free(out);
    unlink(path);
    json_free(root);
    CHECK(json_slurp("/nonexistent/kestrel-air") == NULL);
}
