// A minimal test harness: CHECK counts failures and says where.
#pragma once
#include <stdio.h>

extern int checks, failures;
#define CHECK(c) do { checks++; if (!(c)) { failures++; \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

void test_protocol(void);
void test_crc(void);
void test_frame(void);
void test_json(void);
void test_ring(void);
void test_settings(void);
void test_model(void);
void test_slices(void);
