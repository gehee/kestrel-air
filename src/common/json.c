// A small JSON tree, enough for the unit's settings files: parse, write back
// jansson-style (four-space indent, no trailing newline), keep member order.
#include "json.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

char *json_slurp(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *b = malloc(n + 1);
    if (b && fread(b, 1, n, f) != (size_t)n) { free(b); b = NULL; }
    if (b) b[n] = 0;
    fclose(f);
    return b;
}

void json_free(json_node *n) {
    while (n) {
        json_node *nx = n->next;
        json_free(n->kids);
        free(n->key);
        free(n->text);
        free(n);
        n = nx;
    }
}

static const char *skip(const char *p) {
    while (*p && isspace((unsigned char)*p)) p++;
    return p;
}

static char *jstring(const char **pp) {
    const char *p = *pp + 1, *s = p;
    while (*p && *p != '"') p += *p == '\\' && p[1] ? 2 : 1;
    char *out = strndup(s, p - s);
    *pp = *p ? p + 1 : p;
    return out;
}

json_node *json_parse(const char **pp) {
    const char *p = skip(*pp);
    json_node *n = calloc(1, sizeof(*n)), **tail = &n->kids;
    if (*p == '{' || *p == '[') {
        n->type = *p == '{' ? 'o' : 'a';
        char end = *p == '{' ? '}' : ']';
        p = skip(p + 1);
        while (*p && *p != end) {
            char *key = NULL;
            if (n->type == 'o') {
                if (*p != '"') break;
                key = jstring(&p);
                p = skip(p);
                if (*p == ':') p++;
            }
            json_node *k = json_parse(&p);
            k->key = key;
            *tail = k;
            tail = &k->next;
            p = skip(p);
            if (*p == ',') p = skip(p + 1);
        }
        if (*p == end) p++;
    } else if (*p == '"') {
        n->type = 's';
        n->text = jstring(&p);
    } else {
        const char *s = p;
        while (*p && !strchr(",}] \t\r\n", *p)) p++;
        n->type = 'n';
        n->text = strndup(s, p - s);
    }
    *pp = p;
    return n;
}

// jansson's JSON_INDENT(4): one member or element per line.
static void jdump(FILE *f, const json_node *n, int depth) {
    if (n->type == 's') { fprintf(f, "\"%s\"", n->text); return; }
    if (n->type == 'n') { fputs(n->text, f); return; }
    char open = n->type == 'o' ? '{' : '[', close = n->type == 'o' ? '}' : ']';
    if (!n->kids) { fprintf(f, "%c%c", open, close); return; }
    fprintf(f, "%c\n", open);
    for (const json_node *k = n->kids; k; k = k->next) {
        fprintf(f, "%*s", (depth + 1) * 4, "");
        if (n->type == 'o') fprintf(f, "\"%s\": ", k->key);
        jdump(f, k, depth + 1);
        fputs(k->next ? ",\n" : "\n", f);
    }
    fprintf(f, "%*s%c", depth * 4, "", close);
}

int json_write(const char *path, const json_node *root) {
    FILE *f = fopen(path, "w");
    if (!f) return -1;
    jdump(f, root, 0);
    fflush(f);
    fclose(f);
    sync();
    return 0;
}

// One number, bare or quoted: "0x66" as hex; a quoted string without the
// prefix is read as hex too (digits only), as stock's reader does.
int json_num(const json_node *n) {
    if (n->type == 'n') return (int)strtol(n->text, NULL, 0);
    const char *s = n->text;
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) return (int)strtol(s, NULL, 16);
    int v = 0;
    for (; *s >= '0' && *s <= '9'; s++) v = v * 16 + (*s - '0');
    return v;
}
