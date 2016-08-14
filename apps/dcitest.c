#include <stdio.h>
#include <stdlib.h>
#include "../src/dci.h"

int main(int argc, char **argv)
{
	int rbs;

	if (argc != 2) {
		printf("%s <number of resource blocks>\n", argv[0]);
		return 0;
	}

	rbs = atoi(argv[1]);

	printf("\n==== FD-LTE ====\n");
	lte_dci_print_sizes(rbs, LTE_MODE_FDD);

	printf("\n==== TD-LTE ====\n");
	lte_dci_print_sizes(rbs, LTE_MODE_TDD);

	return 0;
}
