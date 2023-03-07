/*
 * test_double.c -- program to generate contents
 *                  of double variabels in hexadecimal
 *
 * compile with: gcc -o test_double test_double.c -lm
 *
 * Note: RISC-V must be Little Endian
 *
 */

#include <stdio.h>
#include <math.h>

union {
	double x;
	unsigned long long int y;
} t;

int main(void) {


	t.x = 0.57;
	t.x = sin(t.x);
	printf("%.16f = %016llx\n", t.x, t.y);

	t.x = 0.57;
	t.x = asin(t.x);
	printf("%.16f = %016llx\n", t.x, t.y);

	t.x = 0.57;
	t.x = tan(t.x);
	printf("%.16f = %016llx\n", t.x, t.y);
}
