// A small JSON tree, enough for the unit's settings files (see json.c).
#pragma once

typedef struct json_node {
    char type;              // 'o' object, 'a' array, 's' string, 'n' number/literal
    char *key;              // member name inside an object
    char *text;             // string (without quotes) or number/literal text
    struct json_node *kids, *next;
} json_node;

char      *json_slurp(const char *path);              // the whole file, NUL-terminated; NULL if missing
json_node *json_parse(const char **pp);               // parse at *pp, advancing it
void       json_free(json_node *n);                   // a node and everything after it
int        json_write(const char *path, const json_node *root);   // and sync; -1 on failure
int        json_num(const json_node *n);              // a number, bare or quoted (see json.c)
